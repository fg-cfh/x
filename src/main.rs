#![no_std]
#![no_main]

mod driver;
mod util;

use core::{
    panic::PanicInfo,
    sync::atomic::{AtomicUsize, Ordering},
};

use cortex_m::{
    Peripherals,
    asm::{delay, wfe},
    interrupt::InterruptNumber,
    peripheral::NVIC,
};
use nrf_pac::{clock::vals::Lfclksrc, interrupt};

use crate::driver::{
    radio::{RadioDriver, RadioDriverStateMachine, RadioTask},
    socs::nrf::{
        NrfExcPrio, NrfPrioBits,
        radio::{Nrf, NrfOff, NrfRadioDriverExcCtx, NrfRx, NrfTx, RADIO_DRIVER},
        timer::sleep_timer,
    },
    task::{InterruptContext, SharedState},
};

/* enum DriverSvcMsg {
    Rx,
    Tx,
}

static P: AtomicPtr<Producer<'static, DriverSvcMsg>> = AtomicPtr::new(null_mut());
static C: AtomicPtr<Consumer<'static, DriverSvcMsg>> = AtomicPtr::new(null_mut()); */

#[cortex_m_rt::entry]
fn main() -> ! {
    /* {
        static mut Q: Queue<DriverSvcMsg, 2> = Queue::new();
        // Safety: The queue is only accessible in this scope and `main` is only
        //         called once.
        #[allow(static_mut_refs)]
        let (p, c) = unsafe { Q.split() };
        p
    }; */

    RADIO_DRIVER.init_and_send(
        |driver| driver.init(sleep_timer()),
        NrfRadioDriverExcCtx::Swi0,
    );
    loop {
        wfe()
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
