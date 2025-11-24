use core::marker::PhantomData;

use nrf_pac::{P0, gpio::vals::Dir};

use crate::task::{OngoingTask, StepResult, Task};

struct RadioDriver<State> {
    state: PhantomData<State>,
}

trait RadioState: Sized {
    fn transition(&mut self) -> StepResult<(), ()>;
    fn complete(self) -> StepResult<Self, ()>;
}

trait OffState {
    fn schedule_rx(&mut self);
    fn schedule_tx(&mut self);
}

trait RxState {
    fn schedule_tx(&mut self);
    fn schedule_off(&mut self);
}

trait TxState {
    fn schedule_rx(&mut self);
    fn schedule_off(&mut self);
}

impl<State> RadioDriver<State> {
    const OFF_LED: usize = 13;
    const RX_LED: usize = 14;
    const TX_LED: usize = 15;
    const TRANSITION_LED: usize = 16;

    fn led_on(&mut self, pin: usize) {
        P0.outclr().write(|w| w.set_pin(pin, true));
    }

    fn led_off(&mut self, pin: usize) {
        P0.outset().write(|w| w.set_pin(pin, true));
    }

    fn off_enter(&mut self) {
        self.led_on(Self::OFF_LED);
    }

    fn off_exit(&mut self) {
        self.led_off(Self::OFF_LED);
    }

    fn rx_enter(&mut self) {
        self.led_on(Self::RX_LED);
    }

    fn rx_exit(&mut self) {
        self.led_off(Self::RX_LED);
    }

    fn tx_enter(&mut self) {
        self.led_on(Self::TX_LED);
    }

    fn tx_exit(&mut self) {
        self.led_off(Self::TX_LED);
    }

    fn start_transition(&mut self) {
        self.led_on(Self::TRANSITION_LED);
    }

    fn end_transition(&mut self) {
        self.led_off(Self::TRANSITION_LED);
    }
}

struct Off;
impl RadioDriver<Off> {
    fn new() -> Self {
        for led in [
            Self::OFF_LED,
            Self::RX_LED,
            Self::TX_LED,
            Self::TRANSITION_LED,
        ] {
            P0.outset().write(|w| w.set_pin(led, true));
            P0.pin_cnf(led).write(|w| w.set_dir(Dir::OUTPUT));
        }
        Self { state: PhantomData }
    }
}

impl RadioState for RadioDriver<Off> {
    fn complete(mut self) -> StepResult<Self, ()> {
        self.off_exit();
        StepResult::Completed(())
    }

    fn transition(&mut self) -> StepResult<(), ()> {
        self.end_transition();
        self.off_enter();
        StepResult::Completed(())
    }
}

impl OffState for RadioDriver<Off> {
    fn schedule_rx(&mut self) {
        self.start_transition();
    }

    fn schedule_tx(&mut self) {
        self.start_transition();
    }
}

struct Rx;
impl RadioState for RadioDriver<Rx> {
    fn complete(mut self) -> StepResult<Self, ()> {
        self.rx_exit();
        StepResult::Completed(())
    }

    fn transition(&mut self) -> StepResult<(), ()> {
        self.end_transition();
        self.rx_enter();
        StepResult::Completed(())
    }
}

impl RxState for RadioDriver<Rx> {
    fn schedule_tx(&mut self) {
        self.start_transition();
    }

    fn schedule_off(&mut self) {
        self.start_transition();
    }
}

struct Tx;
impl RadioState for RadioDriver<Tx> {
    fn complete(mut self) -> StepResult<Self, ()> {
        self.tx_exit();
        StepResult::Completed(())
    }

    fn transition(&mut self) -> StepResult<(), ()> {
        self.end_transition();
        self.tx_enter();
        StepResult::Completed(())
    }
}

impl TxState for RadioDriver<Tx> {
    fn schedule_rx(&mut self) {
        self.start_transition();
    }

    fn schedule_off(&mut self) {
        self.start_transition();
    }
}

#[allow(private_interfaces)]
pub enum RadioDriverState {
    Off(RadioDriver<Off>),
    OffToRx(RadioDriverTransitionState<Off, Rx>),
    OffToTx(RadioDriverTransitionState<Off, Tx>),
    Rx(RadioDriver<Rx>),
    RxToTx(RadioDriverTransitionState<Rx, Tx>),
    RxToOff(RadioDriverTransitionState<Rx, Off>),
    Tx(RadioDriver<Tx>),
    TxToRx(RadioDriverTransitionState<Tx, Rx>),
    TxToOff(RadioDriverTransitionState<Tx, Off>),
}

impl RadioDriverState {
    pub fn new() -> Self {
        Self::Off(RadioDriver::new())
    }
}

#[derive(Clone, Copy)]
pub enum RadioDriverEvent {
    ScheduleRx,
    ScheduleTx,
    ScheduleOff,
    Interrupt,
}

impl OngoingTask<RadioDriverEvent> for RadioDriverState {
    fn step(self, event: RadioDriverEvent) -> Self {
        use RadioDriverEvent as E;
        use RadioDriverTransitionState as Transition;
        use StepResult::*;
        match event {
            E::ScheduleRx => match self {
                Self::Off(mut driver) => {
                    driver.schedule_rx();
                    Self::OffToRx(Transition::CompletingSourceState(driver))
                }
                Self::Tx(mut driver) => {
                    driver.schedule_rx();
                    Self::TxToRx(Transition::CompletingSourceState(driver))
                }
                _ => unreachable!(),
            },
            E::ScheduleTx => match self {
                Self::Off(mut driver) => {
                    driver.schedule_tx();
                    Self::OffToTx(Transition::CompletingSourceState(driver))
                }
                Self::Rx(mut driver) => {
                    driver.schedule_tx();
                    Self::RxToTx(Transition::CompletingSourceState(driver))
                }
                _ => unreachable!(),
            },
            E::ScheduleOff => match self {
                Self::Tx(mut driver) => {
                    driver.schedule_off();
                    Self::TxToOff(Transition::CompletingSourceState(driver))
                }
                Self::Rx(mut driver) => {
                    driver.schedule_off();
                    Self::RxToOff(Transition::CompletingSourceState(driver))
                }
                _ => unreachable!(),
            },
            E::Interrupt => match self {
                Self::OffToRx(transition) => match transition.step(()) {
                    Continue(transition) => Self::OffToRx(transition),
                    Completed(driver) => Self::Rx(driver),
                },
                Self::OffToTx(transition) => match transition.step(()) {
                    Continue(transition) => Self::OffToTx(transition),
                    Completed(driver) => Self::Tx(driver),
                },
                Self::RxToTx(transition) => match transition.step(()) {
                    Continue(transition) => Self::RxToTx(transition),
                    Completed(driver) => Self::Tx(driver),
                },
                Self::RxToOff(transition) => match transition.step(()) {
                    Continue(transition) => Self::RxToOff(transition),
                    Completed(driver) => Self::Off(driver),
                },
                Self::TxToRx(transition) => match transition.step(()) {
                    Continue(transition) => Self::TxToRx(transition),
                    Completed(driver) => Self::Rx(driver),
                },
                Self::TxToOff(transition) => match transition.step(()) {
                    Continue(transition) => Self::TxToOff(transition),
                    Completed(driver) => Self::Off(driver),
                },
                _ => unreachable!(),
            },
        }
    }
}

#[allow(private_interfaces)]
pub enum RadioDriverTransitionState<Source, Target> {
    CompletingSourceState(RadioDriver<Source>),
    EnteringTargetState(RadioDriver<Target>),
}

impl<Source, Target> Task<(), RadioDriver<Target>> for RadioDriverTransitionState<Source, Target>
where
    RadioDriver<Source>: RadioState,
    RadioDriver<Target>: RadioState,
{
    fn step(self, _: ()) -> StepResult<Self, RadioDriver<Target>> {
        use StepResult::*;
        match self {
            Self::CompletingSourceState(driver) => match driver.complete() {
                Continue(driver) => Continue(Self::CompletingSourceState(driver)),
                Completed(_) => Continue(Self::EnteringTargetState(RadioDriver {
                    state: PhantomData,
                })),
            },
            Self::EnteringTargetState(mut driver) => match driver.transition() {
                Continue(_) => Continue(Self::EnteringTargetState(driver)),
                Completed(_) => Completed(driver),
            },
        }
    }
}
