use core::mem;

use cortex_m::{asm::delay, peripheral::NVIC};
use nrf_pac::{P0, gpio::vals::Dir, interrupt};

use crate::{
    MAIN_PRIO, SWI_PRIO,
    driver::{
        executor::{Interrupt, Task},
        radio::{
            Off, OffState, RadioDriver, RadioDriverState, RadioState, Rx, RxState, Tx, TxState,
        },
    },
    util::sync::priority_cell::PriorityCell,
};

pub(crate) struct Nrf;

pub(crate) static RADIO_DRIVER: PriorityCell<Option<RadioDriverState<Nrf>>> =
    PriorityCell::new(None, MAIN_PRIO);

impl RadioDriverState<Nrf> {
    pub(crate) fn new() -> Self {
        Self::Off(RadioDriver::new())
    }
}

impl<State> RadioDriver<Nrf, State> {
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

impl RadioDriver<Nrf, Off> {
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
        Self::enter_state()
    }
}

impl RadioState for RadioDriver<Nrf, Off> {
    fn complete(mut self) -> Task<Self, ()> {
        if self.is_off() {
            delay(8000000);
            self.off_exit();
            Task::Continue(self)
        } else {
            Task::Completed(())
        }
    }

    fn transition(&mut self) -> Task<(), ()> {
        if self.is_off() {
            Task::Completed(())
        } else {
            delay(8000000);
            self.end_transition();
            self.off_enter();
            Task::Continue(())
        }
    }
}

impl OffState for RadioDriver<Nrf, Off> {
    fn schedule_rx(&mut self) {
        self.start_transition();
    }

    fn schedule_tx(&mut self) {
        self.start_transition();
    }
}

impl RadioState for RadioDriver<Nrf, Rx> {
    fn complete(mut self) -> Task<Self, ()> {
        if self.is_rx() {
            delay(8000000);
            self.rx_exit();
            Task::Continue(self)
        } else {
            Task::Completed(())
        }
    }

    fn transition(&mut self) -> Task<(), ()> {
        if self.is_rx() {
            Task::Completed(())
        } else {
            delay(8000000);
            self.end_transition();
            self.rx_enter();
            Task::Continue(())
        }
    }
}

impl RxState for RadioDriver<Nrf, Rx> {
    fn schedule_tx(&mut self) {
        self.start_transition();
    }

    fn schedule_off(&mut self) {
        self.start_transition();
    }
}

impl RadioState for RadioDriver<Nrf, Tx> {
    fn complete(mut self) -> Task<Self, ()> {
        if self.is_tx() {
            delay(8000000);
            self.tx_exit();
            Task::Continue(self)
        } else {
            Task::Completed(())
        }
    }

    fn transition(&mut self) -> Task<(), ()> {
        if self.is_tx() {
            Task::Completed(())
        } else {
            delay(8000000);
            self.end_transition();
            self.tx_enter();
            Task::Continue(())
        }
    }
}

impl TxState for RadioDriver<Nrf, Tx> {
    fn schedule_rx(&mut self) {
        self.start_transition();
    }

    fn schedule_off(&mut self) {
        self.start_transition();
    }
}

#[interrupt]
fn RADIO() {
    use Interrupt::*;
    use RadioDriverState as R;
    let mut driver_borrow = RADIO_DRIVER.borrow_mut();
    let driver_ref = driver_borrow.as_mut().unwrap();
    let driver = mem::replace(driver_ref, RadioDriverState::Executing);
    let (updated_driver, done) = match driver {
        R::OffToRx(mut transition) => match transition.on_interrupt() {
            Continue => (R::OffToRx(transition), false),
            Completed => (R::Rx(transition.target_state()), true),
        },
        R::OffToTx(mut transition) => match transition.on_interrupt() {
            Continue => (R::OffToTx(transition), false),
            Completed => (R::Tx(transition.target_state()), true),
        },
        R::RxToTx(mut transition) => match transition.on_interrupt() {
            Continue => (R::RxToTx(transition), false),
            Completed => (R::Tx(transition.target_state()), true),
        },
        R::RxToOff(mut transition) => match transition.on_interrupt() {
            Continue => (R::RxToOff(transition), false),
            Completed => (R::Off(transition.target_state()), true),
        },
        R::TxToRx(mut transition) => match transition.on_interrupt() {
            Continue => (R::TxToRx(transition), false),
            Completed => (R::Rx(transition.target_state()), true),
        },
        R::TxToOff(mut transition) => match transition.on_interrupt() {
            Continue => (R::TxToOff(transition), false),
            Completed => (R::Off(transition.target_state()), true),
        },
        _ => unreachable!(),
    };
    *driver_ref = updated_driver;
    if done {
        RADIO_DRIVER.release(driver_borrow, SWI_PRIO);
        NVIC::pend(interrupt::EGU0_SWI0);
    } else {
        NVIC::pend(interrupt::RADIO);
    }
}
