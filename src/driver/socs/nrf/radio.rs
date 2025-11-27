use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m::{asm::delay, interrupt::InterruptNumber, peripheral::NVIC};
use fugit::ExtU64;
use nrf_pac::{P0, RADIO, gpio::vals::Dir, interrupt};

use crate::{
    driver::{
        radio::{
            DriverConfig, OffState, RadioDriver, RadioDriverCtx, RadioDriverState,
            RadioDriverStateMachine, RadioState, RadioTask, RxState, TxState,
        },
        socs::nrf::{
            NrfExcPrio, NrfPrioBits,
            timer::{NrfSleepTimer, NrfSleepTimerAlarm},
        },
        task::{Interrupt, InterruptContext, InterruptHandler, SharedState, Task},
        timer::SleepTimer,
    },
    util::sync::{cancellation::Cancelable, priority_cell::PriorityCell},
};

#[derive(Clone, Copy)]
pub(crate) struct Nrf;
#[derive(Clone, Copy)]
pub(crate) struct NrfOff;
#[derive(Clone, Copy)]
pub(crate) struct NrfRx;
#[derive(Clone, Copy)]
pub(crate) struct NrfTx;

impl DriverConfig for Nrf {
    type Timer = NrfSleepTimer;
    type OffState = NrfOff;
    type RxState = NrfRx;
    type TxState = NrfTx;
}

impl RadioDriverStateMachine<Nrf> {
    pub(crate) const fn new() -> Self {
        Self::new_unchecked(RadioDriver::new())
    }
}

impl RadioDriver<Nrf> {
    const fn new() -> Self {
        Self::Off(RadioDriverState::new())
    }
}

impl<State> RadioDriverState<Nrf, State> {
    const OFF_LED: usize = 13;
    const RX_LED: usize = 14;
    const TX_LED: usize = 15;
    const TRANSITION_LED: usize = 16;

    fn set_led_on(&mut self, pin: usize) {
        P0.outclr().write(|w| w.set_pin(pin, true));
    }

    fn set_led_off(&mut self, pin: usize) {
        P0.outset().write(|w| w.set_pin(pin, true));
    }

    fn is_led_on(&self, pin: usize) -> bool {
        !P0.out().read().pin(pin)
    }

    fn off_enter(&mut self) {
        self.set_led_on(Self::OFF_LED);
    }

    fn off_exit(&mut self) {
        self.set_led_off(Self::OFF_LED);
    }

    fn is_off(&self) -> bool {
        self.is_led_on(Self::OFF_LED)
    }

    fn rx_enter(&mut self) {
        self.set_led_on(Self::RX_LED);
    }

    fn rx_exit(&mut self) {
        self.set_led_off(Self::RX_LED);
    }

    fn is_rx(&self) -> bool {
        self.is_led_on(Self::RX_LED)
    }

    fn tx_enter(&mut self) {
        self.set_led_on(Self::TX_LED);
    }

    fn tx_exit(&mut self) {
        self.set_led_off(Self::TX_LED);
    }

    fn is_tx(&self) -> bool {
        self.is_led_on(Self::TX_LED)
    }

    fn start_transition(&mut self) {
        self.set_led_on(Self::TRANSITION_LED);
    }

    fn end_transition(&mut self) {
        self.set_led_off(Self::TRANSITION_LED);
    }
}

impl RadioDriverState<Nrf, NrfOff> {
    const fn new() -> Self {
        Self::enter_state(NrfOff)
    }

    fn init(&mut self) {
        for led in [
            Self::OFF_LED,
            Self::RX_LED,
            Self::TX_LED,
            Self::TRANSITION_LED,
        ] {
            P0.outset().write(|w| w.set_pin(led, true));
            P0.pin_cnf(led).write(|w| w.set_dir(Dir::OUTPUT));
        }
    }
}

impl RadioDriverStateMachine<Nrf> {
    pub fn init(&mut self, sleep_timer: NrfSleepTimer) {
        self.ctx.init(sleep_timer);
        if let RadioDriver::Off(radio_driver_state) = &mut self.driver {
            radio_driver_state.init();
        } else {
            unreachable!()
        }
    }
}

// Optimization: Protect the inner function from being inlined (speed->size).
#[inline(never)]
fn execute_delayed<S>(
    this: &mut RadioDriverState<Nrf, S>,
    ctx: &mut RadioDriverCtx<Nrf>,
    f: fn(&mut RadioDriverState<Nrf, S>),
) -> Task<(), ()> {
    fn start_timer(ctx: &mut RadioDriverCtx<Nrf>) {
        let mut timer = ctx.sleep_timer();
        let now = timer.now();
        let alarm = timer
            .wait_until(now + 500.millis(), || NVIC::pend(interrupt::RADIO))
            .unwrap();
        ctx.sleep_timer_alarm = Some(alarm);
    }

    if let Some(alarm) = ctx.sleep_timer_alarm.take() {
        drop(alarm);
        f(this);
        Task::Completed(())
    } else {
        start_timer(ctx);
        Task::Continue(())
    }
}

impl RadioState<Nrf> for RadioDriverState<Nrf, NrfOff> {
    fn complete(mut self, ctx: &mut RadioDriverCtx<Nrf>) -> Task<Self, ()> {
        match execute_delayed(&mut self, ctx, Self::off_exit) {
            Task::Continue(_) => Task::Continue(self),
            Task::Completed(_) => Task::Completed(()),
        }
    }

    fn transition(&mut self, ctx: &mut RadioDriverCtx<Nrf>) -> Task<(), ()> {
        self.start_transition();
        execute_delayed(self, ctx, |this| {
            this.end_transition();
            this.off_enter();
        })
    }
}

impl OffState for RadioDriverState<Nrf, NrfOff> {
    fn schedule_rx(&mut self) {}

    fn schedule_tx(&mut self) {}
}

impl RadioState<Nrf> for RadioDriverState<Nrf, NrfRx> {
    fn complete(mut self, ctx: &mut RadioDriverCtx<Nrf>) -> Task<Self, ()> {
        match execute_delayed(&mut self, ctx, Self::rx_exit) {
            Task::Continue(_) => Task::Continue(self),
            Task::Completed(_) => Task::Completed(()),
        }
    }

    fn transition(&mut self, ctx: &mut RadioDriverCtx<Nrf>) -> Task<(), ()> {
        self.start_transition();
        execute_delayed(self, ctx, |this| {
            this.end_transition();
            this.rx_enter();
        })
    }
}

impl RxState for RadioDriverState<Nrf, NrfRx> {
    fn schedule_tx(&mut self) {}

    fn schedule_off(&mut self) {}
}

impl RadioState<Nrf> for RadioDriverState<Nrf, NrfTx> {
    fn complete(mut self, ctx: &mut RadioDriverCtx<Nrf>) -> Task<Self, ()> {
        match execute_delayed(&mut self, ctx, Self::tx_exit) {
            Task::Continue(_) => Task::Continue(self),
            Task::Completed(_) => Task::Completed(()),
        }
    }

    fn transition(&mut self, ctx: &mut RadioDriverCtx<Nrf>) -> Task<(), ()> {
        self.start_transition();
        execute_delayed(self, ctx, |this| {
            this.end_transition();
            this.tx_enter();
        })
    }
}

impl TxState for RadioDriverState<Nrf, NrfTx> {
    fn schedule_rx(&mut self) {}

    fn schedule_off(&mut self) {}
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum NrfRadioDriverExcCtx {
    Swi0 = NrfExcPrio::LOWEST_INTERRUPT.to_arm_nvic_repr(),
    Radio = NrfExcPrio::LOWEST_INTERRUPT.plus_one().to_arm_nvic_repr(),
}

impl NrfRadioDriverExcCtx {
    const ALL: [Self; 2] = [Self::Swi0, Self::Radio];
}

unsafe impl InterruptNumber for NrfRadioDriverExcCtx {
    fn number(self) -> u16 {
        match self {
            Self::Swi0 => interrupt::EGU0_SWI0.number(),
            Self::Radio => interrupt::RADIO.number(),
        }
    }
}

impl InterruptContext for NrfRadioDriverExcCtx {
    type PB = NrfPrioBits;

    fn arm_nvic_priority(&self) -> u8 {
        *self as u8
    }

    fn interrupt_contexts<'i>() -> &'i [Self] {
        &Self::ALL
    }
}

pub(crate) static RADIO_DRIVER: SharedState<RadioDriverStateMachine<Nrf>, NrfRadioDriverExcCtx> =
    SharedState::new(RadioDriverStateMachine::new());

#[interrupt]
fn EGU0_SWI0() {
    use RadioTask::*;
    const EVENTS: [RadioTask<Nrf>; 3] = [Rx(NrfRx), Tx(NrfTx), Off(NrfOff)];

    static EVENT: AtomicUsize = AtomicUsize::new(0);

    let mut radio_driver = RADIO_DRIVER.borrow_mut();
    let event = EVENT.load(Ordering::Relaxed);
    radio_driver.schedule(EVENTS[event]);
    EVENT.store((event + 1) % EVENTS.len(), Ordering::Relaxed);

    RADIO_DRIVER.release_and_send(radio_driver, NrfRadioDriverExcCtx::Radio);
}

#[interrupt]
fn RADIO() {
    let mut radio_driver = RADIO_DRIVER.borrow_mut();
    if matches!(radio_driver.on_interrupt(()), Interrupt::Completed) {
        RADIO_DRIVER.release_and_send(radio_driver, NrfRadioDriverExcCtx::Swi0);
    }
}
