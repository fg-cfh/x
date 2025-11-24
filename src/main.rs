#![no_std]
#![no_main]

mod radio;
mod task;

use core::panic::PanicInfo;

use cortex_m::asm::delay;

use crate::{
    radio::{RadioDriverEvent, RadioDriverState},
    task::OngoingTask,
};

#[cortex_m_rt::entry]
fn main() -> ! {
    use RadioDriverEvent::*;
    const EVENTS: [RadioDriverEvent; 9] = [
        ScheduleRx,
        Interrupt,
        Interrupt,
        ScheduleTx,
        Interrupt,
        Interrupt,
        ScheduleOff,
        Interrupt,
        Interrupt,
    ];
    let mut event = 0;
    let mut radio_driver = RadioDriverState::new();
    loop {
        radio_driver = radio_driver.step(EVENTS[event]);
        delay(16000000);
        event = (event + 1) % EVENTS.len();
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
