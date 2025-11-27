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
    peripheral::NVIC,
};
use nrf_pac::interrupt;

use crate::driver::{
    radio::{RadioDriverState, RadioTask},
    socs::nrf::{
        executor::{LOWEST_HANDLER_PRIO, THREAD_PRIO, arm_system_priority},
        radio::RADIO_DRIVER,
    },
};

/* enum DriverSvcMsg {
    Rx,
    Tx,
}

static P: AtomicPtr<Producer<'static, DriverSvcMsg>> = AtomicPtr::new(null_mut());
static C: AtomicPtr<Consumer<'static, DriverSvcMsg>> = AtomicPtr::new(null_mut()); */

const MAIN_PRIO: u8 = arm_system_priority(THREAD_PRIO);
const SWI_PRIO: u8 = arm_system_priority(LOWEST_HANDLER_PRIO);
const RADIO_PRIO: u8 = arm_system_priority(LOWEST_HANDLER_PRIO + 1);

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

    let mut radio_driver = RADIO_DRIVER.borrow_mut();
    *radio_driver = Some(RadioDriverState::new());
    RADIO_DRIVER.release(radio_driver, SWI_PRIO);

    let mut nvic = Peripherals::take().unwrap().NVIC;
    unsafe {
        // Safety:
        // - We own these priorities exclusively.
        // - We do not rely on priority masking to share resources.
        nvic.set_priority(interrupt::EGU0_SWI0, SWI_PRIO);
        nvic.set_priority(interrupt::RADIO, RADIO_PRIO);

        // Safety: We do not rely on masking to share resources.
        NVIC::unmask(interrupt::EGU0_SWI0);
        NVIC::unmask(interrupt::RADIO);
    }

    NVIC::pend(interrupt::EGU0_SWI0);

    loop {
        wfe()
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[interrupt]
fn EGU0_SWI0() {
    use RadioTask::*;
    const EVENTS: [RadioTask; 3] = [Rx, Tx, Off];

    static EVENT: AtomicUsize = AtomicUsize::new(0);

    let mut radio_driver_borrow = RADIO_DRIVER.borrow_mut();
    let radio_driver_ref = radio_driver_borrow.as_mut().unwrap();
    let event = EVENT.load(Ordering::Relaxed);
    radio_driver_ref.schedule(EVENTS[event]);
    RADIO_DRIVER.release(radio_driver_borrow, RADIO_PRIO);
    NVIC::pend(interrupt::RADIO);
    EVENT.store((event + 1) % EVENTS.len(), Ordering::Relaxed);
    delay(16000000);
}
