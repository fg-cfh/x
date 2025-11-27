use nrf_pac::{
    CLOCK,
    clock::vals::{HfclkstatSrc, Lfclksrc},
};

pub fn start_hf_oscillator() {
    debug_assert!(CLOCK.hfclkstat().read().src() != HfclkstatSrc::XTAL);
    CLOCK.events_hfclkstarted().write_value(0);
    CLOCK.tasks_hfclkstart().write_value(0x1);
    while CLOCK.events_hfclkstarted().read() != 0x1 {}
}

pub fn start_lf_clock(sleep_timer_clk_src: Lfclksrc) {
    // When debugging, the LF clock may continue to run across restarts.
    stop_lf_clock();
    match sleep_timer_clk_src {
        Lfclksrc::RC => CLOCK.lfclksrc().write(|w| {
            w.set_src(Lfclksrc::RC);
            w.set_external(false);
            w.set_bypass(false);
        }),
        Lfclksrc::SYNTH => CLOCK.lfclksrc().write(|w| {
            w.set_src(Lfclksrc::SYNTH);
            w.set_external(false);
            w.set_bypass(false);
        }),
        #[cfg(not(feature = "ext-lf-clk"))]
        Lfclksrc::XTAL => CLOCK.lfclksrc().write(|w| {
            w.set_src(Lfclksrc::XTAL);
            w.set_external(false);
            w.set_bypass(false);
        }),
        #[cfg(feature = "ext-lf-clk")]
        Lfclksrc::XTAL => CLOCK.lfclksrc().write(|w| {
            w.set_src(Lfclksrc::XTAL);
            w.set_external(true);
            w.set_bypass(true);
        }),
        _ => unreachable!(),
    }
    CLOCK.events_lfclkstarted().write_value(0);
    CLOCK.tasks_lfclkstart().write_value(0x1);
    // When connected to an external clock source, this loop will block
    // indefinitely until the external clock has been started.
    while CLOCK.events_lfclkstarted().read() != 0x1 {}
}

pub fn stop_lf_clock() {
    CLOCK.tasks_lfclkstop().write_value(0x1);
}
