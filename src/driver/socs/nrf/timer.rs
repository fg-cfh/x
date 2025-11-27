//! Timer implementation for nRF SoCs.

use core::{
    array::from_fn,
    cell::{Cell, UnsafeCell},
    num::{NonZero, NonZeroU32},
    sync::atomic::{AtomicU8, AtomicU32, AtomicUsize, Ordering, compiler_fence},
};

use cortex_m::{
    asm::wfe,
    peripheral::{NVIC, Peripherals as CorePeripherals},
};
use fugit::TimerRateU64;
#[cfg(feature = "timer-trace")]
use nrf_pac::GPIOTE;
use nrf_pac::{
    PPI, RADIO, RTC0, TIMER0,
    clock::vals::Lfclksrc,
    interrupt,
    timer::vals::{Bitmode, Mode},
};

use crate::{
    driver::{
        socs::nrf::NrfExcPrio,
        timer::{
            HardwareEvent, HardwareSignal, HighPrecisionTimer, NsDuration, NsInstant,
            OptionalNsInstant, SleepTimer, TimedSignal, TimerError,
        },
    },
    util::sync::{cancellation::Cancelable, init_cell::InitCell},
};

use super::clock::{start_hf_oscillator, start_lf_clock};

/// Alarm channels represent shared resources that need to be synchronized
/// between scheduling context and interrupts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlarmChannel {
    HighPrecisionTimer = 0,
    SleepTimer,
    NumRtcChannels,
}

/// Flag representing the current state of an alarm.
///
/// # Safety
///
/// This flag synchronizes ownership of shared resources between the RTC
/// interrupt and scheduling context:
///
/// - While the alarm is active, an RTC interrupt may fire at any time, preempt
///   the scheduling context and access and mutate the alarm as well as related
///   resources (e.g. alarm state or peripheral registers).
///
/// - While the alarm is pending or after it fired, shared resources must not be
///   accessed from RTC interrupt context. The scheduling context can then
///   access and mutate the alarm as well as related resources.
///
/// Shared resources must not be accessed or mutated from any other than
/// scheduling or interrupt context.
///
/// Considered alternatives, that don't work:
///
/// Synchronization via intenclr/set:
/// - LDREX/STREX are disallowed on device memory
/// - RTC interrupts remain disabled while we wait for the active half period
/// - Depending on the configuration, several distinct RTC interrupts may be
///   involved.
/// - Several instances of the sleep timer may concurrently access RTC alarms.
///
/// Synchronization via a special value of the RTC tick (Self::OFF):
/// - The overflow-protected RTC tick is 64 bits wide. A 64 bit value cannot
///   be accessed atomically on a 32 bit platform. Portable atomics would
///   introduce critical sections which we want to avoid.
///
/// Note that the safety conditions of the [`SleepTimer`] require the RTC
/// interrupt handler to run at a higher priority than the scheduling thread.
/// This guarantees that any ongoing interrupt execution continues to own shared
/// resources even if it releases them by changing the alarm state. Ownership
/// transfer takes place at the end of such an interrupt execution as it is
/// atomic from the perspective of scheduling context.
///
/// In case of TIMER interrupts, we can use the interrupt register to
/// synchronize resources: Only a globally unique instance of the high precision
/// timer and only a single interrupt can be active.
#[repr(u8)]
enum AlarmState {
    /// The alarm is currently unused and may be acquired by any scheduling
    /// context.
    Unused,
    /// The alarm has been acquired by some scheduling context. It is owned by
    /// that scheduling context.
    Pending,
    /// High-precision synchronization channel only: The high-precision timer is
    /// currently being synchronized to the RTC. The alarm is exclusively owned
    /// by interrupt context. It must be ensured that only a single interrupt
    /// handler is accessing the alarm.
    Synchronizing,
    /// The alarm is currently running and exclusively owned by interrupt
    /// context. Interrupts may preempt scheduling context at any time. It must
    /// be ensured that only a single interrupt handler is accessing the alarm.
    Active,
    /// The alarm has fired and exclusive ownership was transferred back to the
    /// scheduling context.
    Fired,
}

// Resources shared and synchronized between scheduling context and RTC
// interrupt context.
struct Alarm {
    /// The current alarm state. See [`AlarmState`] for details and safety
    /// considerations.
    state: AtomicU8,

    /// The RTC tick of a pending alarm.
    ///
    /// Safety: Access is synchronized via the alarm state, see above.
    rtc_tick: UnsafeCell<u64>,

    /// Callback for the current alarm (if any).
    ///
    /// Safety: In case of an RTC alarm, access is synchronized via the alarm
    ///         state, see [`AlarmState`] for details. This is required as
    ///         canceling the alarm races with firing it (i.e. calling back).
    cb: UnsafeCell<Option<fn()>>,
}

/// Safety: See safety comments of [`Alarm`].
unsafe impl Sync for Alarm {}

impl Alarm {
    const fn new() -> Self {
        Self {
            // The state is initially 'unused' to signal that the scheduling
            // thread has exclusive access to this alarm but still needs to
            // program it.
            state: AtomicU8::new(AlarmState::Unused as u8),
            rtc_tick: UnsafeCell::new(0),
            cb: UnsafeCell::new(None),
        }
    }

    /// Safety: Requires the current execution context to own the alarm, see the
    ///         corresponding safety documentation in [`Alarm`].
    unsafe fn take_cb(&self) -> Option<fn()> {
        (unsafe { *self.cb.get() }).take()
    }

    /// Safety: Requires the current execution context to own the alarm, see the
    ///         corresponding safety documentation in [`Alarm`].
    unsafe fn set_cb(&self, cb: fn()) {
        unsafe { *self.cb.get() = Some(cb) };
    }

    /// Safety: Requires the current execution context to own the alarm, see the
    ///         corresponding safety documentation in [`Alarm`].
    unsafe fn rtc_tick(&self) -> u64 {
        unsafe { *self.rtc_tick.get() }
    }

    /// Safety: Requires the current execution context to own the alarm, see the
    ///         corresponding safety documentation in [`Alarm`].
    unsafe fn set_rtc_tick(&self, rtc_tick: u64) {
        unsafe { *self.rtc_tick.get() = rtc_tick };
    }
}

const NUM_RTC_CHANNELS: usize = AlarmChannel::NumRtcChannels as usize;
const RTC_CHANNELS: [AlarmChannel; NUM_RTC_CHANNELS] =
    [AlarmChannel::HighPrecisionTimer, AlarmChannel::SleepTimer];

/// The nRF radio timer implements a local (i.e. non-syntonized), monotonic,
/// overflow-protected uptime counter. It combines a low-energy RTC sleep timer
/// with a high-precision wake-up TIMER.
///
/// The timer can trigger asynchronous CPU wake-ups, fire PPI-backed hardware
/// signals or capture hardware event timestamps.
///
/// Safety: As we are on the single-core nRF platform we don't need to
///         synchronize atomic operations via CPU memory barriers. It is
///         sufficient to place appropriate compiler fences.
struct State {
    /// Number of half counter periods elapsed since boot.
    ///
    /// Safety: This needs to be atomic as it will be shared between interrupt
    ///         and application contexts.
    half_period: AtomicU32,

    /// Independent resource synchronization channels supported by the RTC
    /// driver.
    ///
    /// Safety: RTC alarms are Sync.
    alarms: [Alarm; NUM_RTC_CHANNELS],

    /// Contains the reference overflow-protected RTC tick (losslessly
    /// represented as the corresponding radio clock instant) at which the
    /// high-precision timer was started.
    ///
    /// This value is initialized while the [`AlarmChannel::HighPrecisionTimer`]
    /// channel is in state [`AlarmState::Pending`] (when synchronizing to a
    /// specific RTC tick) or [`AlarmState::Synchronizing`] (when synchronizing
    /// to the next RTC tick). It is only valid to read in state
    /// [`AlarmState::Active`].
    ///
    /// Safety: Guarded by the [`AlarmChannel::HighPrecisionTimer`] alarm's
    ///         [`AlarmState`]: May be written from scheduling context while
    ///         that state is neither synchronizing nor active. May be written
    ///         from interrupt context while synchronizing. Read-only access
    ///         from all contexts while active.
    timer_epoch: Cell<NsInstant>,

    /// GPIOTE channel used for GPIO signal triggering.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    #[cfg(feature = "timer-trace")]
    gpiote_out_channel: AtomicUsize,

    /// GPIOTE channel used for GPIO event capturing.
    ///
    /// Will only be accessed from scheduling context but is atomic to satisfy
    /// the type system.
    #[cfg(feature = "timer-trace")]
    gpiote_in_channel: AtomicUsize,

    /// Primary PPI channel used for signal triggering (timer synchronization,
    /// gpio toggle) and event capturing (rx/tx enabled).
    ///
    /// We assume that these signals and events will not be used concurrently.
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel1: AtomicUsize,

    /// Secondary PPI channel used for event capturing (frame started,
    /// radio disabled, gpio toggled).
    ///
    /// We assume that these events will not be used concurrently.
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel2: AtomicUsize,

    /// PPI channel group used for signal triggering (disable unless
    /// framestart).
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel_group: AtomicUsize,

    /// The PPI channel mask containing all PPI channels used by this driver.
    ///
    /// May be accessed from scheduling and interrupt context.
    ppi_channel_mask: AtomicU32,
}

/// Safety: See safety comments in the implementation.
unsafe impl Sync for State {}

impl State {
    // See the high-precision timer trait implementation docs for resource
    // assignments.

    const RTC_THREE_QUARTERS_PERIOD: u64 = 0xc00000;
    const RTC_GUARD_TICKS: u64 = 2;

    // Pre-assigned timer channels.
    const TIMER_TEST_CHANNEL: usize = 2;
    const TIMER_OVERFLOW_PROTECTION_CHANNEL: usize = 3;
    const RTC_OVERFLOW_PROTECTION_CHANNEL: usize = 2;

    // Pre-programmed PPI channels.
    const TIMER_CC0_RADIO_TXEN_CHANNEL: usize = 20;
    const TIMER_CC0_RADIO_RXEN_CHANNEL: usize = 21;
    const TIMER_CC1_RADIO_DISABLE_CHANNEL: usize = 22;
    const RTC_CC0_TIMER_START_CHANNEL: usize = 31;

    const PPI_CHANNEL_MASK: u32 = 1 << Self::TIMER_CC0_RADIO_TXEN_CHANNEL
        | 1 << Self::TIMER_CC0_RADIO_RXEN_CHANNEL
        | 1 << Self::TIMER_CC1_RADIO_DISABLE_CHANNEL
        | 1 << Self::RTC_CC0_TIMER_START_CHANNEL;

    const TIMER_INT_PRIO: NrfExcPrio = NrfExcPrio::HIGHEST.minus_one();
    const RTC_INT_PRIO: NrfExcPrio = Self::TIMER_INT_PRIO.minus_one();

    const fn new(config: NrfTimerConfig) -> Self {
        let [ppi_channel1, ppi_channel2] = config.ppi_channels;
        assert!(ppi_channel1 <= 19);
        assert!(ppi_channel2 <= 19);
        assert!(ppi_channel1 != ppi_channel2);
        let ppi_channel_mask = 1 << ppi_channel1 | 1 << ppi_channel2 | Self::PPI_CHANNEL_MASK;

        assert!(config.ppi_channel_group <= 5);

        #[cfg(feature = "timer-trace")]
        let NrfTimerTracingConfig {
            gpiote_out_channel,
            gpiote_in_channel,
            gpiote_tick_channel,
            ppi_tick_channel,
        } = config.tracing_config;

        #[cfg(feature = "timer-trace")]
        {
            assert!(gpiote_out_channel <= 7);
            assert!(gpiote_in_channel <= 7);
            assert!(gpiote_tick_channel <= 7);
            assert!(gpiote_in_channel != gpiote_out_channel);
            assert!(gpiote_in_channel != gpiote_tick_channel);
            assert!(gpiote_tick_channel != gpiote_out_channel);

            assert!(ppi_tick_channel <= 19);
            assert!(ppi_channel1 != ppi_tick_channel);
            assert!(ppi_channel2 != ppi_tick_channel);
        }

        Self {
            half_period: AtomicU32::new(0),
            alarms: [Alarm::new(), Alarm::new()],
            timer_epoch: Cell::new(NsInstant::from_ticks(0)),
            #[cfg(feature = "timer-trace")]
            gpiote_out_channel: AtomicUsize::new(config.tracing_config.gpiote_out_channel),
            #[cfg(feature = "timer-trace")]
            gpiote_in_channel: AtomicUsize::new(config.tracing_config.gpiote_in_channel),
            ppi_channel1: AtomicUsize::new(ppi_channel1),
            ppi_channel2: AtomicUsize::new(ppi_channel2),
            ppi_channel_group: AtomicUsize::new(config.ppi_channel_group),
            ppi_channel_mask: AtomicU32::new(ppi_channel_mask),
        }
    }

    fn init(&self, config: NrfTimerConfig) {
        // The radio requires an external HF crystal oscillator.
        start_hf_oscillator();
        // The hybrid timer requires the LF oscillator to be continuously enabled.
        start_lf_clock(config.sleep_timer_clk_src);

        #[cfg(feature = "timer-trace")]
        {
            PPI.ch(config.tracing_config.ppi_tick_channel)
                .eep()
                .write_value(RTC0.events_tick().as_ptr() as u32);
            PPI.ch(config.tracing_config.ppi_tick_channel)
                .tep()
                .write_value(
                    GPIOTE
                        .tasks_out(config.tracing_config.gpiote_tick_channel)
                        .as_ptr() as u32,
                );
            PPI.chenset()
                .write(|w| w.set_ch(config.tracing_config.ppi_tick_channel, true));

            RTC0.evtenset().write(|w| w.set_tick(true));
        }

        TIMER0.tasks_stop().write_value(0x1);
        TIMER0.mode().write(|w| w.set_mode(Mode::TIMER));
        TIMER0.bitmode().write(|w| w.set_bitmode(Bitmode::_32BIT));
        TIMER0.prescaler().write(|w| w.set_prescaler(0));
        TIMER0.tasks_clear().write_value(0x1);

        // The timer must not overflow: stop it when it reaches its max value.
        TIMER0
            .cc(Self::TIMER_OVERFLOW_PROTECTION_CHANNEL)
            .write_value(u32::MAX);
        TIMER0
            .shorts()
            .write(|w| w.set_compare_stop(Self::TIMER_OVERFLOW_PROTECTION_CHANNEL, true));

        RTC0.tasks_stop().write_value(0x1);
        RTC0.prescaler().write(|w| w.set_prescaler(0));

        // Set up half-period counting.
        RTC0.cc(Self::RTC_OVERFLOW_PROTECTION_CHANNEL)
            .write(|w| w.set_compare(0x800000));
        RTC0.intenset().write(|w| {
            w.set_ovrflw(true);
            w.set_compare(Self::RTC_OVERFLOW_PROTECTION_CHANNEL, true);
        });

        RTC0.tasks_clear().write_value(0x1);
        while RTC0.counter().read().counter() != 0 {}

        RTC0.tasks_start().write_value(0x1);
        while RTC0.counter().read().counter() == 0 {}

        // Safety: We don't rely on priority masking for synchronization.
        let mut nvic = unsafe { CorePeripherals::steal() }.NVIC;
        unsafe {
            nvic.set_priority(interrupt::TIMER0, Self::TIMER_INT_PRIO.to_arm_nvic_repr());
            nvic.set_priority(interrupt::RTC0, Self::RTC_INT_PRIO.to_arm_nvic_repr());
        };

        NVIC::unpend(interrupt::RTC0);
        NVIC::unpend(interrupt::TIMER0);

        // Safety: We don't rely on interrupt masking for synchronization.
        unsafe { NVIC::unmask(interrupt::RTC0) };
        unsafe { NVIC::unmask(interrupt::TIMER0) };
    }

    /// Sets an RTC alarm's tick value.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// # Safety
    ///
    /// - The alarm state must be owned by the calling context.
    unsafe fn set_rtc_alarm_tick(&self, channel: AlarmChannel, rtc_tick: u64) {
        let alarm = &self.alarms[channel as usize];
        // Safety: See method comment.
        unsafe { alarm.set_rtc_tick(rtc_tick) };
    }

    /// Read the alarm's RTC tick.
    ///
    /// # Safety
    ///
    /// - The alarm state must be owned by the calling context.
    /// - Compiler fences are required to acquire/release this value.
    unsafe fn rtc_alarm_tick(&self, channel: AlarmChannel) -> u64 {
        // Safety: See method comment.
        unsafe { self.alarms[channel as usize].rtc_tick() }
    }

    /// Returns `true` while the RTC alarm is active or synchronizing (and
    /// therefore owned by interrupt context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_rtc_alarm_owned_by_interrupt(&self, channel: AlarmChannel) -> bool {
        let state = self.alarms[channel as usize].state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Active as u8 || state == AlarmState::Synchronizing as u8
    }

    /// Returns `true` while the RTC alarm is pending (and owned by scheduling
    /// context).
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_rtc_alarm_pending(&self, channel: AlarmChannel) -> bool {
        let state = self.alarms[channel as usize].state.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Pending as u8
    }

    /// Returns 'true' while the high-precision timer is being synchronized to
    /// the RTC via the "next tick" synchronization approach.
    ///
    /// Acquires alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn is_rtc_pending_immediate_synchronization(&self) -> bool {
        let state = self.alarms[AlarmChannel::HighPrecisionTimer as usize]
            .state
            .load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        state == AlarmState::Synchronizing as u8
    }

    /// Mark the RTC alarm as pending if it is unused.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    fn rtc_try_acquire_alarm(&self, channel: AlarmChannel) -> Result<(), TimerError> {
        compiler_fence(Ordering::Release);
        let result = self.alarms[channel as usize].state.compare_exchange_weak(
            AlarmState::Unused as u8,
            AlarmState::Pending as u8,
            Ordering::Relaxed,
            Ordering::Relaxed,
        );
        compiler_fence(Ordering::Acquire);
        if result.is_ok() {
            Ok(())
        } else {
            Err(TimerError::Busy)
        }
    }

    /// Transfer ownership of the RTC alarm to interrupt context to synchronize
    /// the high-precision timer via the "next tick" approach.
    ///
    /// Releases the high-precision timer alarm memory.
    ///
    /// Called exclusively from scheduling context.
    fn rtc_start_immediate_timer_synchronization(&self) {
        compiler_fence(Ordering::Release);
        self.alarms[AlarmChannel::HighPrecisionTimer as usize]
            .state
            .store(AlarmState::Synchronizing as u8, Ordering::Relaxed);
    }

    /// Transfer ownership of the RTC alarm to interrupt context.
    ///
    /// Releases alarm memory.
    ///
    /// Called exclusively from scheduling context.
    ///
    /// Note: This does _not_ also activate interrupts. These may have to remain
    ///       inactive if we've not yet reached the target timer period.
    fn rtc_activate_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .store(AlarmState::Active as u8, Ordering::Relaxed);
    }

    /// Disables timer interrupts and signals to the scheduling context that the
    /// given RTC alarm has been fired and is now inactive.
    ///
    /// Transfers ownership of the RTC alarm from interrupt context to
    /// scheduling context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    ///
    /// Must not be called on unused timers. If the timer had been fired before,
    /// then this is a no-op.
    fn rtc_fire_and_inactivate_alarm(&self, channel: AlarmChannel) {
        // Safety: We need to disable the interrupt before we transfer
        //         ownership of the alarm to the scheduling context. We disable
        //         the interrupt early, as it may take up to four cycles before
        //         this operation takes effect. Should the interrupt be
        //         spuriously woken it will additionally check alarm state.
        match channel {
            AlarmChannel::HighPrecisionTimer => RTC0
                .evtenclr()
                .write(|w| w.set_compare(channel as usize, true)),
            AlarmChannel::SleepTimer => RTC0
                .intenclr()
                .write(|w| w.set_compare(channel as usize, true)),
            _ => unreachable!(),
        };
        self.rtc_fire_alarm(channel);
    }

    /// Mark the RTC alarm as fired.
    ///
    /// Transfers ownership of the alarm from interrupt context to scheduling
    /// context and releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    fn rtc_fire_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .store(AlarmState::Fired as u8, Ordering::Relaxed);
    }

    /// Mark the RTC alarm as unused. Expects the alarm to be fired.
    ///
    /// Acquires, then releases alarm memory.
    ///
    /// May be called from both, interrupt and scheduling context.
    ///
    /// # Panics
    ///
    /// Panics if the alarm was not previously fired.
    fn rtc_release_alarm(&self, channel: AlarmChannel) {
        compiler_fence(Ordering::Release);
        self.alarms[channel as usize]
            .state
            .compare_exchange_weak(
                AlarmState::Fired as u8,
                AlarmState::Unused as u8,
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .unwrap();
        compiler_fence(Ordering::Acquire);
    }

    /// Reads the high-precision timer's epoch.
    ///
    /// # Safety
    ///
    /// - Shared read-only access while the [`AlarmChannel::HighPrecisionTimer`]
    ///   is active.
    /// - Must not be called in any other state.
    unsafe fn timer_epoch(&self) -> NsInstant {
        debug_assert_eq!(
            self.alarms[AlarmChannel::HighPrecisionTimer as usize]
                .state
                .load(Ordering::Relaxed),
            AlarmState::Active as u8
        );
        self.timer_epoch.get()
    }

    /// Sets the high-precision timer's epoch from an overflow-protected RTC
    /// tick.
    ///
    /// # Safety
    ///
    /// - Writable from scheduling context while the
    ///   [`AlarmChannel::HighPrecisionTimer`] is neither synchronizing nor
    ///   active.
    /// - Writable from interrupt context while the channel is synchronizing.
    /// - Must not be called in active state.
    unsafe fn set_timer_epoch(&self, rtc_tick: u64) {
        debug_assert!(
            self.is_rtc_alarm_pending(AlarmChannel::HighPrecisionTimer)
                || self.is_rtc_pending_immediate_synchronization()
        );
        self.timer_epoch
            .set(TickConversion::rtc_tick_to_instant(rtc_tick));
    }

    /// Retrieve a captured timer value and disables the corresponding PPI
    /// channel. If the event was not observed, the function returns zero and
    /// leaves the PPI channel enabled.
    fn timer_get_and_clear_captured_ticks(&self, event: HardwareEvent) -> u32 {
        let event_timer_channel = Self::timer_event_channel(event);

        // Safety: Reading a single 32 bit register is atomic.
        let mut timer_ticks = TIMER0.cc(event_timer_channel).read();

        if timer_ticks > 0 {
            // An event has been observed and the corresponding PPI channel may
            // be re-used.
            let event_ppi_channel = self.ppi_event_channel(event);
            PPI.chenclr().write(|w| w.set_ch(event_ppi_channel, true));

            // Account for PPI delay, see nRF52840 PS, section 6.16: The events we
            // observe will be delayed by up to one 16 MHz clock period (i.e. one
            // timer tick). This is not exact science as radio events may not be
            // aligned to the 16 MHz clock. The excess probabilistic reduction
            // somewhat counters the probabilistic delay introduced by RTC-to-timer
            // synchronization, so at least errors shouldn't add up.
            timer_ticks -= 1;
        }

        timer_ticks
    }

    // Called exclusively from interrupt context.
    fn on_rtc_interrupt(&self) {
        // Performance: We're betting that only a single event is pending. If
        //              that's generally true, then the interrupt handler will
        //              execute faster because it doesn't unnecessarily check
        //              for other conditions. If events overlap, then we'll
        //              cause the handler to be re-invoked unnecessarily,
        //              though.
        let channel = AlarmChannel::SleepTimer;
        if RTC0.events_compare(channel as usize).read() == 0x1 {
            RTC0.events_compare(channel as usize).write_value(0);
            self.rtc_trigger_alarm(channel);
            return;
        }

        if RTC0.events_ovrflw().read() == 0x1 {
            RTC0.events_ovrflw().write_value(0);
            self.rtc_increment_half_period();
        } else if RTC0
            .events_compare(Self::RTC_OVERFLOW_PROTECTION_CHANNEL)
            .read()
            == 0x1
        {
            RTC0.events_compare(Self::RTC_OVERFLOW_PROTECTION_CHANNEL)
                .write_value(0);
            self.rtc_increment_half_period();
        }
    }

    // Called exclusively from interrupt context.
    fn on_timer_interrupt(&self) {
        if self.is_rtc_pending_immediate_synchronization() {
            self.timer_synchronize();

            // While we synchronize, no other interrupts must be enabled.
            debug_assert_eq!(TIMER0.intenset().read().0, 0);

            return;
        }

        let enabled_interrupts_mask = TIMER0.intenset().read().0;

        // Ensure that we've been triggered by a capture event from appropriate
        // timer channels which asserts that we own shared memory.
        debug_assert_ne!(enabled_interrupts_mask & (0b11 << 16), 0);
        debug_assert_eq!(enabled_interrupts_mask & !(0b11 << 16), 0);

        // Safety: Interrupt context still owns the alarm.
        let callback = unsafe { self.alarms[AlarmChannel::HighPrecisionTimer as usize].take_cb() };
        if let Some(callback) = callback {
            callback();
        }

        // Disable all interrupts so the interrupt won't be pended again. We
        // leave the compare event active to ensure that subsequent invocations
        // of the wait method won't trigger another interrupt execution.

        // Note: The intenclr and intenset registers have the same layout.
        TIMER0.intenclr().write(|w| w.0 = enabled_interrupts_mask);

        // Safety: Exiting the interrupt handler should take more than four
        //         cycles (POP of four regs including PC: 1 + N + P >= 6), see
        //         #interrupt implementation, ARM M4 technical reference manual
        //         and nRF52840 PS, section 6.1.8, "interrupt clearing" for
        //         context.
    }

    /// Calculate the timestamp from the period count and the tick count.
    ///
    /// The RTC counter is 24 bit. Ticking at 32768 Hz, it overflows every ~8
    /// minutes. This is too short. We must protect it against overflow.
    ///
    /// The obvious way would be to count overflow periods. Every time the
    /// counter overflows, increase a `periods` variable. `now()` simply does
    /// `periods << 24 + counter`. So, the logic around an overflow would look
    /// like this:
    ///
    /// ```not_rust
    /// periods = 1, counter = 0xFF_FFFE --> now = 0x1FF_FFFE
    /// periods = 1, counter = 0xFF_FFFF --> now = 0x1FF_FFFF
    /// **OVERFLOW**
    /// periods = 2, counter = 0x00_0000 --> now = 0x200_0000
    /// periods = 2, counter = 0x00_0001 --> now = 0x200_0001
    /// ```
    ///
    /// The problem is that this is vulnerable to race conditions if `now()`
    /// runs at the exact time an overflow happens.
    ///
    /// If `now()` reads `periods` first and `counter` later, and overflow
    /// happens between the reads, it would return a wrong value:
    ///
    /// ```not_rust
    /// periods = 1 (OLD), counter = 0x00_0000 (NEW) --> now = 0x100_0000 -> WRONG
    /// ```
    ///
    /// It fails similarly if it reads `counter` first and `periods` second.
    ///
    /// To fix this, we define a "half period" to be 2^23 ticks (instead of
    /// 2^24). One "overflow cycle" is 2 periods.
    ///
    /// - `half period` is incremented on overflow (at counter value 0)
    /// - `half period` is incremented "midway" between overflows (at counter
    ///   value 0x80_0000)
    ///
    /// Therefore, when `half period` is even, the counter is expected to be in
    /// the range 0..0x7f_ffff, when odd, in the range 0x80_0000..0xff_ffff.
    ///
    /// To get `now()`, the `half period` is read first, then the `counter`. If
    /// the counter value range matches the expected `half period` parity, we're
    /// done. If it doesn't, we know that a new half period has started between
    /// reading `period` and `counter`. We then assume that the `counter` value
    /// corresponds to the next half period.
    ///
    /// The `half period` has 32 bits and a single half period is represented by
    /// 23 bits. The counter ticks at 32768 Hz. The overflow protected counter
    /// therefore wraps after (2^55-1) / 32768 seconds of uptime, which
    /// corresponds to ~34865 years.
    ///
    /// May be called from both, interrupt and scheduling context.
    ///
    /// Note: This overflow protection approach was shamelessly copied from
    ///       embassy_nrf. Kudos to the embassy contributors!
    fn rtc_now_tick(&self) -> u64 {
        // The `half_period` MUST be read before `counter`, see method docs.
        let half_period = self.half_period.load(Ordering::Relaxed);
        compiler_fence(Ordering::Acquire);
        let counter = RTC0.counter().read().counter();
        ((half_period as u64) << 23) + ((counter ^ ((half_period & 1) << 23)) as u64)
    }

    // Called exclusively from interrupt context.
    fn rtc_increment_half_period(&self) {
        let next_half_period = self.half_period.load(Ordering::Relaxed) + 1;
        // Note: The acquire part of the fence protects the read to the alarm's
        //       RTC tick below. The release part ensures that the updated
        //       period becomes visible to all clients. Inside the interrupt
        //       this fence is not strictly necessary but we add it as it is
        //       essentially free, documents intent and protects us from UB.
        compiler_fence(Ordering::AcqRel);
        self.half_period.store(next_half_period, Ordering::Relaxed);
        let next_half_period_start_tick = (next_half_period as u64) << 23;

        for channel in RTC_CHANNELS {
            // Safety: Ensure that we own the alarm before accessing it.
            if self.is_rtc_alarm_owned_by_interrupt(channel) {
                // Safety: The call to `is_rtc_alarm_owned_by_interrupt()`
                //         atomically acquires the RTC tick value and ensures
                //         exclusive access.
                let pending_rtc_tick = unsafe { self.rtc_alarm_tick(channel) };
                if pending_rtc_tick < next_half_period_start_tick + Self::RTC_THREE_QUARTERS_PERIOD
                {
                    // Just enable the compare interrupt. The correct CC value
                    // has already been set when scheduling the alarm.
                    match channel {
                        AlarmChannel::HighPrecisionTimer => RTC0
                            .evtenset()
                            .write(|w| w.set_compare(channel as usize, true)),
                        AlarmChannel::SleepTimer => RTC0
                            .intenset()
                            .write(|w| w.set_compare(channel as usize, true)),
                        _ => unreachable!(),
                    }
                }
            }
        }
    }

    // Called exclusively from interrupt context.
    //
    // Note: May be preempted by higher-priority interrupts but _not_ by the
    //       scheduling context.
    //
    // Tuning notes:
    fn rtc_trigger_alarm(&self, channel: AlarmChannel) {
        // Performance:
        // - Switching if/else below doesn't yield a measurable improvement.
        // - Using a callback over pending the SWI directly costs us ~60ns.

        // Safety: Acquires alarm memory and ensures exclusive ownership. As the
        //         scheduling context runs at a lower priority, the interrupt
        //         operates atomically on alarm memory. We can therefore safely
        //         access the alarm until the interrupt handler ends.
        if !self.is_rtc_alarm_owned_by_interrupt(channel) {
            // Spurious compare interrupt, possibly due to a race on disabling
            // the interrupt when an overdue alarm is discovered.
            return;
        }

        // Safety: We acquired alarm memory and ensured ownership above.
        if self.rtc_now_tick() < unsafe { self.rtc_alarm_tick(channel) } {
            // Spurious compare interrupt: If the COUNTER is N and the
            // current CC register value is N+1 or N+2 when a new CC value
            // is written, a match may trigger on the previous CC value
            // before the new value takes effect, see nRF product
            // specification.
            return;
        }

        // Safety: Interrupt context still owns the alarm.
        let callback = unsafe { self.alarms[channel as usize].take_cb() };
        if let Some(callback) = callback {
            callback();
        }

        // Return ownership to the scheduling context before we leave interrupt
        // context.
        self.rtc_fire_and_inactivate_alarm(channel);

        if callback.is_none() {
            self.rtc_release_alarm(channel);
        }
    }

    // Called exclusively from scheduling context.
    fn rtc_program_cc(&self, rtc_tick: u64, channel: AlarmChannel) -> Result<u64, ()> {
        // Safety: Ensure that the scheduling context exclusively owns the alarm
        //         and corresponding registers.
        debug_assert!(self.is_rtc_alarm_pending(channel));

        // The nRF product spec says: If the COUNTER is N, writing N or N+1 to a
        // CC register may not trigger a COMPARE event.
        //
        // To work around this, we never program a tick smaller than N+3. N+2
        // is not safe because the RTC can tick from N to N+1 between calling
        // now() and writing to the CC register.
        let rtc_now_tick = self.rtc_now_tick();
        if rtc_tick <= rtc_now_tick + State::RTC_GUARD_TICKS {
            return Err(());
        }

        // Safety: We checked above that we exclusively own the channel.
        unsafe { self.set_rtc_alarm_tick(channel, rtc_tick) };
        RTC0.cc(channel as usize)
            .write(|w| w.set_compare(rtc_tick as u32 & 0xFFFFFF));

        Ok(rtc_now_tick)
    }

    fn rtc_safely_schedule_alarm(
        &self,
        rtc_tick: u64,
        rtc_now_tick: u64,
        channel: AlarmChannel,
    ) -> Result<(), ()> {
        // Safety: Releases alarm memory to interrupt context. From this point
        //         on we may no longer access the alarm except to enable
        //         interrupts.
        self.rtc_activate_alarm(channel);

        if rtc_tick - rtc_now_tick < Self::RTC_THREE_QUARTERS_PERIOD {
            // If the alarm is imminent (i.e. safely within the currently
            // running RTC period), enable the timer interrupt (resp. event)
            // right away.

            match channel {
                AlarmChannel::HighPrecisionTimer => RTC0
                    .evtenset()
                    .write(|w| w.set_compare(channel as usize, true)),
                AlarmChannel::SleepTimer => RTC0
                    .intenset()
                    .write(|w| w.set_compare(channel as usize, true)),
                _ => unreachable!(),
            }

            // Safety: This method may have been preempted by higher-priority
            //         interrupts. Also, its execution time depends on compiler
            //         optimization. Therefore we need to ensure that the alarm
            //         was safely scheduled _after_ enabling the corresponding
            //         interrupt.
            let was_safely_scheduled = self.rtc_now_tick() + Self::RTC_GUARD_TICKS <= rtc_tick;
            if !was_safely_scheduled {
                // Safety: The alarm may or may not have already fired at this
                //         point. It may even spuriously fire later as disabling
                //         interrupts is not immediate. Therefore the interrupt
                //         handler additionally synchronizes on alarm state.
                self.rtc_fire_and_inactivate_alarm(channel);
                Err(())
            } else {
                Ok(())
            }
        } else {
            // If the alarm is too far into the future, don't enable the compare
            // interrupt yet. It will be enabled by `next_period()`.
            Ok(())
        }
    }

    // Called exclusively from scheduling context.
    fn rtc_try_activate_alarm(&self, rtc_tick: u64, channel: AlarmChannel) -> Result<(), ()> {
        let rtc_now_tick = self.rtc_program_cc(rtc_tick, channel)?;
        self.rtc_safely_schedule_alarm(rtc_tick, rtc_now_tick, channel)
    }

    // Called exclusively from scheduling context.
    //
    // The given channel must have been acquired (i.e. pending) before calling
    // this method.
    fn rtc_wait_until(
        &self,
        channel: AlarmChannel,
        instant: NsInstant,
        cb: fn(),
    ) -> Result<NrfSleepTimerAlarm, TimerError> {
        let rtc_tick = TickConversion::instant_to_rtc_tick(instant);

        // Safety: We exclusively own the alarm at this point.
        unsafe { self.alarms[channel as usize].set_cb(cb) };

        // Safety: To avoid a data race, we may only activate the alarm once
        //         we're sure that the callback has been safely installed.
        //         Activating the alarm establishes a happens-before
        //         relationship with all prior memory accesses and transfers
        //         ownership of the alarm to interrupt context.
        match self.rtc_try_activate_alarm(rtc_tick, channel) {
            Ok(_) => Ok(NrfSleepTimerAlarm { channel }),
            Err(_) => Err(TimerError::Overdue(instant)),
        }
    }

    // Called exclusively from scheduling context.
    fn ppi_program_timed_signal(
        &self,
        signal: HardwareSignal,
        disabled_by_event: Option<HardwareEvent>,
    ) -> Result<
        (
            // ppi channel
            usize,
            // timer channel
            usize,
        ),
        TimerError,
    > {
        let (ppi_channel, timer_channel) = self.timer_signal_channels(signal);

        let previous_timer_fired = TIMER0.events_compare(timer_channel).read() == 0x1;
        if previous_timer_fired {
            // The previously programmed signal was already emitted, the timer
            // is programmed not to overflow, i.e. the compare event cannot be
            // triggered again at this point. We reset the compare event but
            // leave the channel enabled so that a new compare value can be
            // programmed subsequently.
            TIMER0.events_compare(timer_channel).write_value(0);
            debug_assert!(PPI.chen().read().ch(ppi_channel));
        } else {
            let timer_channel_busy = TIMER0.cc(timer_channel).read() != 0;
            if timer_channel_busy {
                if matches!(disabled_by_event, Some(HardwareEvent::RadioFrameStarted))
                    && RADIO.events_framestart().read() == 0x1
                {
                    // We share a single timer channel between the disable
                    // signal and the framestarted event.
                    return Err(TimerError::Already);
                };
                return Err(TimerError::Busy);
            }

            // We've checked that the cc register is zero. The timer is programmed
            // not to overflow, i.e. the compare event cannot be triggered at
            // this point. Therefore we don't risk a race by enabling the
            // channel and resetting the compare event here.
            //
            // Safety: The ppi channel has been asserted to be in range and not
            //         currently in use.
            debug_assert!(!PPI.chen().read().ch(ppi_channel));
            PPI.chenset().write(|w| w.set_ch(ppi_channel, true));
        }

        #[cfg(feature = "timer-trace")]
        if matches!(signal, HardwareSignal::GpioToggle) {
            let ch = PPI.ch(ppi_channel);

            let cc_event = TIMER0.events_compare(timer_channel).as_ptr();
            ch.eep().write_value(cc_event as u32);

            let gpiote_channel = self.gpiote_out_channel.load(Ordering::Relaxed);
            let gpiote_out_task = GPIOTE.tasks_out(gpiote_channel).as_ptr();
            ch.tep().write_value(gpiote_out_task as u32);
            PPI.fork(ppi_channel).tep().write_value(0);
        }

        // Safety: The ppi channel is always non-zero, see above.
        Ok((ppi_channel, timer_channel))
    }

    fn ppi_event_channel(&self, event: HardwareEvent) -> usize {
        let ppi_channel1 = || self.ppi_channel1.load(Ordering::Relaxed);
        let ppi_channel2 = || self.ppi_channel2.load(Ordering::Relaxed);
        match event {
            HardwareEvent::RadioRxEnabled | HardwareEvent::RadioDisabled => ppi_channel1(),
            HardwareEvent::RadioFrameStarted => ppi_channel2(),
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => ppi_channel2(),
        }
    }

    // Called exclusively from scheduling context.
    fn ppi_program_event(
        &self,
        event: HardwareEvent,
        disables_signal_ppi_channel: Option<usize>,
    ) -> Result<(), TimerError> {
        // See the high-precision timer trait implementation docs for resource
        // assignments.

        // We run in two distinct modes that allocate distinct resources.
        enum UseCase {
            EventDisablesSignal(
                // The PPI channel representing the signal to be disabled.
                usize,
                // The PPI channel group.
                usize,
            ),
            ObservesEvent(
                // The timer channel used to capture the event.
                usize,
            ),
        }
        use UseCase::*;

        let use_case = if let Some(disables_signal_ppi_channel) = disables_signal_ppi_channel {
            let disables_signal_ppi_channel_group = self.ppi_channel_group.load(Ordering::Relaxed);
            // Safety: The ppi channel group has been asserted to be in range.
            let ppi_chg_busy = PPI.chg(disables_signal_ppi_channel_group).read().0 != 0;
            if ppi_chg_busy {
                return Err(TimerError::Busy);
            }
            EventDisablesSignal(
                disables_signal_ppi_channel,
                disables_signal_ppi_channel_group,
            )
        } else {
            // Corresponding signals and events (e.g. txen and tx enabled) use
            // the same timer channels. Therefore, we cannot identify duplicate
            // use by checking whether the cc register is zero: The register may
            // already contain a signal deadline.
            //
            // As we couple PPI channel 1 with CC0 and PPI channel 2 with CC1,
            // it is sufficient that we check availability of the PPI channel
            // registers instead, see below.
            ObservesEvent(Self::timer_event_channel(event))
        };

        #[cfg(feature = "timer-trace")]
        let gpiote_channel = self.gpiote_in_channel.load(Ordering::Relaxed);
        let event_ptr = match event {
            HardwareEvent::RadioRxEnabled => RADIO.events_rxready().as_ptr(),
            HardwareEvent::RadioFrameStarted => RADIO.events_framestart().as_ptr(),
            HardwareEvent::RadioDisabled => RADIO.events_disabled().as_ptr(),
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => GPIOTE.events_in(gpiote_channel).as_ptr(),
        };

        let event_ppi_channel = self.ppi_event_channel(event);
        let event_ch = PPI.ch(event_ppi_channel);
        let event_fork = PPI.fork(event_ppi_channel);

        let event_ppi_channel_enabled = PPI.chen().read().ch(event_ppi_channel);
        if event_ppi_channel_enabled {
            // If the channel is already enabled it must observe the correct
            // event.
            let current_event = event_ch.eep().read();
            if current_event != event_ptr as u32 {
                return Err(TimerError::Busy);
            }

            // The task register we're trying to program must be available.
            match use_case {
                EventDisablesSignal(..) => {
                    let disable_task = event_fork.tep().read();
                    if disable_task != 0 {
                        return Err(TimerError::Busy);
                    }
                }
                ObservesEvent(..) => {
                    let event_capture_task = event_ch.tep().read();
                    if event_capture_task != 0 {
                        return Err(TimerError::Busy);
                    }
                }
            }
        } else {
            event_ch.eep().write_value(event_ptr as u32);
            if matches!(use_case, EventDisablesSignal(..)) {
                event_ch.tep().write_value(0);
            } else {
                event_fork.tep().write_value(0);
            }
        }

        match use_case {
            EventDisablesSignal(disables_signal_ppi_channel, disables_signal_ppi_channel_group) => {
                // Safety: The ppi channel group has been asserted to be in range and
                //         not in use. The event channel is not yet enabled.
                PPI.chg(disables_signal_ppi_channel_group)
                    .write(|w| w.set_ch(disables_signal_ppi_channel, true));
                let signal_disable_task = PPI
                    .tasks_chg(disables_signal_ppi_channel_group)
                    .dis()
                    .as_ptr();

                // Safety:
                // - The fork must be programmed _after_ the signal channel was already
                //   enabled: This ensures that we don't race to enable the signal
                //   channel if the event occurs after enabling the fork.
                // - The fork must be programmed _before_ the signal may actually
                //   be triggered (i.e. the compare register must be zero): This ensures
                //   that we don't race to enable the fork before the signal is
                //   triggered.
                event_fork.tep().write_value(signal_disable_task as u32);
            }
            ObservesEvent(timer_channel) => {
                let event_capture_task = TIMER0.tasks_capture(timer_channel).as_ptr();
                event_ch.tep().write_value(event_capture_task as u32);
            }
        }

        if !event_ppi_channel_enabled {
            // Safety: The ppi channel is in range an not in use. At this point
            //         eep, ch.tep and fork.tep are guaranteed to be correctly
            //         programmed.
            PPI.chenset().write(|w| w.set_ch(event_ppi_channel, true));
        }

        // Now that the fork is in place and the channel enabled we can check
        // whether the event may have been observed before we programmed the
        // fork and then disable the signal manually.
        let event_has_been_observed = match event {
            HardwareEvent::RadioRxEnabled => RADIO.events_rxready().read(),
            HardwareEvent::RadioFrameStarted => RADIO.events_framestart().read(),
            HardwareEvent::RadioDisabled => RADIO.events_disabled().read(),
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => GPIOTE.events_in(gpiote_channel).read(),
        } == 0x1;

        if event_has_been_observed {
            match use_case {
                EventDisablesSignal(_, disables_signal_ppi_channel_group) => {
                    // The event had been observed before we programmed the
                    // fork: We have to disable the signal manually.
                    PPI.tasks_chg(disables_signal_ppi_channel_group)
                        .dis()
                        .write_value(0x1);
                }
                ObservesEvent(_) => return Err(TimerError::Already),
            }
        }

        Ok(())
    }

    // Called exclusively from interrupt context.
    // The high-precision timer must be synchronizing.
    fn timer_synchronize(&self) {
        let rtc_tick = self.rtc_now_tick();

        // We capture the timer at each tick event to ensure that we read
        // the counter value before it ticked again. If we captured zero,
        // then we can be sure that the rtc tick is valid. Note that there
        // is no way to capture the RTC counter directly.
        assert_eq!(TIMER0.cc(0).read(), 0);

        // Safety: If we're not tracing, the tick interrupt is only used to
        //         synchronize the timer with the RTC.
        #[cfg(not(feature = "timer-trace"))]
        RTC0.evtenclr().write(|w| w.set_tick(true));

        // Clean up timer registers that we used for synchronization.
        TIMER0.intenclr().write(|w| w.set_compare(1, true));
        TIMER0.events_compare(1).write_value(0);
        TIMER0.cc(1).write_value(0);

        // Safety: The channel has been asserted to be in range.
        let ppi_channel = self.ppi_channel1.load(Ordering::Relaxed);
        PPI.chenclr().write(|w| w.set_ch(ppi_channel, true));

        // Safety: This method is only called while the alarm is synchronizing.
        //         Therefore interrupt context has exclusive write access.
        unsafe { self.set_timer_epoch(rtc_tick) };
        self.rtc_activate_alarm(AlarmChannel::HighPrecisionTimer);
    }

    // Called exclusively from scheduling context.
    // The high-precision timer must be pending.
    fn timer_try_start_at_rtc_tick(&self, rtc_tick: u64) -> Result<(), ()> {
        // Safety: We're exclusively called from scheduling context which
        //         currently owns the high-precision timer alarm channel, see
        //         the check in program_rtc_cc() which must be called _after_.
        unsafe { self.set_timer_epoch(rtc_tick) };
        let rtc_now_tick = self.rtc_program_cc(rtc_tick, AlarmChannel::HighPrecisionTimer)?;

        // Safety: This is a pre-programmed PPI channel.
        PPI.chenset()
            .write(|w| w.set_ch(Self::RTC_CC0_TIMER_START_CHANNEL, true));

        self.rtc_safely_schedule_alarm(rtc_tick, rtc_now_tick, AlarmChannel::HighPrecisionTimer)
    }

    // Called exclusively from scheduling context.
    // The high-precision timer must be pending.
    fn timer_start_at_next_rtc_tick(&self) {
        debug_assert!(self.is_rtc_alarm_pending(AlarmChannel::HighPrecisionTimer));

        let ppi_channel = self.ppi_channel1.load(Ordering::Relaxed);
        let ch = PPI.ch(ppi_channel);

        // Mask the tick event while we're setting synchronization up to avoid
        // race conditions.
        #[cfg(feature = "timer-trace")]
        RTC0.evtenclr().write(|w| w.set_tick(true));

        ch.eep().write_value(RTC0.events_tick().as_ptr() as u32);
        ch.tep().write_value(TIMER0.tasks_start().as_ptr() as u32);

        // To ensure that we read the RTC counter value before it ticks again
        // we'll check that the timer's capture register is still zero after we
        // read the counter. Note that the tick event occurs 5 16MHz clock ticks
        // before the counter register actually wraps.
        debug_assert_eq!(TIMER0.cc(0).read(), 0);
        PPI.fork(ppi_channel)
            .tep()
            .write_value(TIMER0.tasks_capture(0).as_ptr() as u32);

        // Safety: The channel has been asserted to be in range.
        PPI.chenset().write(|w| w.set_ch(ppi_channel, true));

        // We trigger from the timer's compare interrupt rather than from the
        // rtc's tick interrupt. This allows us to avoid a race condition when
        // the tick comes precisely between ungating the tick event line and
        // enabling the tick interrupt: One fires and the other not in that
        // case. Thus, un-gating the RTC event line will be the single atomic
        // operation that activates the combined process.
        TIMER0.cc(1).write_value(1);
        debug_assert!(TIMER0.events_compare(1).read() == 0);
        TIMER0.intenset().write(|w| w.set_compare(1, true));

        // Safety: Alarm ownership must be transferred to interrupt context
        //         before enabling event routing.
        self.rtc_start_immediate_timer_synchronization();
        RTC0.evtenset().write(|w| w.set_tick(true));
    }

    fn timer_signal_channels(
        &self,
        signal: HardwareSignal,
    ) -> (
        // PPI channel
        usize,
        // timer channel
        usize,
    ) {
        // See the high-precision timer trait implementation docs for resource
        // assignments.
        match signal {
            HardwareSignal::RadioRxEnable => (Self::TIMER_CC0_RADIO_RXEN_CHANNEL, 0),
            HardwareSignal::RadioTxEnable => (Self::TIMER_CC0_RADIO_TXEN_CHANNEL, 0),
            HardwareSignal::RadioDisable => (Self::TIMER_CC1_RADIO_DISABLE_CHANNEL, 1),
            #[cfg(feature = "timer-trace")]
            HardwareSignal::GpioToggle => (self.ppi_channel1.load(Ordering::Relaxed), 0),
        }
    }

    // Called exclusively from scheduling context.
    fn timer_event_channel(event: HardwareEvent) -> usize {
        // See the high-precision timer trait implementation docs for resource
        // assignments.
        match event {
            HardwareEvent::RadioRxEnabled | HardwareEvent::RadioDisabled => 0,
            HardwareEvent::RadioFrameStarted => 1,
            #[cfg(feature = "timer-trace")]
            HardwareEvent::GpioToggled => 1,
        }
    }

    fn timer_safely_schedule_signal(
        &self,
        instant: NsInstant,
        timer_ticks: NonZeroU32,
        timer_channel: usize,
        signal_ppi_channel: usize,
    ) -> Result<(), TimerError> {
        // Setting the compare register atomically activates the signal.
        TIMER0.cc(timer_channel).write_value(timer_ticks.get());

        // We capture the timer value _after_ activating the signal to check for
        // race conditions (see below).
        TIMER0
            .tasks_capture(Self::TIMER_TEST_CHANNEL)
            .write_value(0x1);
        let timer_now = TIMER0.cc(Self::TIMER_TEST_CHANNEL).read();

        // If the captured timer counter is less than the programmed timer tick,
        // we safely scheduled the timed signal into the future. This is the
        // expected case and should therefore be checked first.
        if timer_now < timer_ticks.get() {
            return Ok(());
        }

        // Otherwise we may encounter two situations:
        if TIMER0.events_compare(timer_channel).read() == 0x1 {
            // 1. The compare event was triggered immediately. This is ok,
            //    because then the timed signal was also correctly triggered.
            Ok(())
        } else {
            // 2. The compare event was not triggered: This means that it was
            //    programmed too late and won't be triggered any more. We can
            //    safely reset state w/o risking a race.
            PPI.chenclr().write(|w| w.ch(signal_ppi_channel));
            TIMER0.cc(timer_channel).write_value(0);
            Err(TimerError::Overdue(instant))
        }
    }

    // Called exclusively from scheduling context.
    fn timer_try_schedule_signal(
        &self,
        instant: NsInstant,
        timer_ticks: NonZeroU32,
        signal: HardwareSignal,
    ) -> Result<(), TimerError> {
        let (signal_ppi_channel_mask, timer_channel) =
            self.ppi_program_timed_signal(signal, None)?;
        self.timer_safely_schedule_signal(
            instant,
            timer_ticks,
            timer_channel,
            signal_ppi_channel_mask,
        )
    }

    // Called exclusively from scheduling context.
    fn timer_try_schedule_signal_unless(
        &self,
        instant: NsInstant,
        timer_ticks: NonZeroU32,
        signal: HardwareSignal,
        event: HardwareEvent,
    ) -> Result<(), TimerError> {
        // Safety: We need to program the timed signal w/o actually allowing the
        //         timer to trigger it before we connect the event to disable
        //         the signal. See more detailed safety conditions in
        //         sub-methods.
        let (signal_ppi_channel_mask, timer_channel) =
            self.ppi_program_timed_signal(signal, Some(event))?;
        self.ppi_program_event(event, Some(signal_ppi_channel_mask))?;

        self.timer_safely_schedule_signal(
            instant,
            timer_ticks,
            timer_channel,
            signal_ppi_channel_mask,
        )
    }

    // Called exclusively from scheduling context.
    fn timer_try_observe_event(&self, event: HardwareEvent) -> Result<(), TimerError> {
        self.ppi_program_event(event, None)
    }

    // Called exclusively from scheduling context.
    fn timer_wait_for(&self, signal: HardwareSignal, cb: fn()) -> NrfHighPrecisionTimerAlarm {
        let (ppi_channel, timer_channel) = self.timer_signal_channels(signal);
        assert!(PPI.chen().read().ch(ppi_channel));

        let event_ptr = TIMER0.events_compare(timer_channel).as_ptr();
        let event_ok = PPI.ch(ppi_channel).eep().read() == event_ptr as u32;
        assert!(event_ok);

        // We don't support concurrently waiting clients, only a single callback
        // can be stored.
        assert_eq!(TIMER0.intenset().read().0, 0);

        if TIMER0.events_compare(timer_channel).read() == 0x1 {
            // We don't reset the event so that the method can be called
            // repeatedly without involving another interrupt.
            cb();
        } else {
            // Safety: We just found that no interrupt is active, so the
            //         scheduling context cannot be interrupted and can safely set
            //         the callback. (This differs from our approach to
            //         synchronizing access to RTC alarms as the sleep timer can be
            //         shared across tasks.)
            unsafe { self.alarms[AlarmChannel::HighPrecisionTimer as usize].set_cb(cb) };

            // Ensure that setting the callback happens-before enabling the
            // interrupt.
            compiler_fence(Ordering::Release);

            // If the timer fired between our initial check of the event flag
            // and now, the interrupt will fire immediately, i.e. we don't cause
            // a race although we don't check the event flag again.
            TIMER0
                .intenset()
                .write(|w| w.set_compare(timer_channel, true));
        }

        NrfHighPrecisionTimerAlarm { timer_channel }
    }
}

#[cfg(feature = "timer-trace")]
struct NrfTimerTracingConfig {
    gpiote_out_channel: usize,
    gpiote_in_channel: usize,
    gpiote_tick_channel: usize,
    ppi_tick_channel: usize,
}
#[cfg(feature = "timer-trace")]
const TIMER_TRACING_CONFIG: NrfTimerTracingConfig = NrfTimerTracingConfig {
    gpiote_out_channel: 0,
    gpiote_in_channel: 1,
    gpiote_tick_channel: 2,
    ppi_tick_channel: 2,
};

struct NrfTimerConfig {
    sleep_timer_clk_src: Lfclksrc,
    ppi_channels: [usize; 2],
    ppi_channel_group: usize,
    #[cfg(feature = "timer-trace")]
    tracing_config: NrfTimerTracingConfig,
}
const TIMER_CONFIG: NrfTimerConfig = NrfTimerConfig {
    sleep_timer_clk_src: Lfclksrc::XTAL,
    ppi_channels: [0, 1],
    ppi_channel_group: 0,
    #[cfg(feature = "timer-trace")]
    tracing_config: TIMER_TRACING_CONFIG,
};

static STATE: InitCell<State> = InitCell::new(State::new(TIMER_CONFIG));

/// Instantiate the timer for the first time. Further copies can then be created.
pub fn sleep_timer() -> NrfSleepTimer {
    STATE.init(|mut state| state.init(TIMER_CONFIG));
    NrfSleepTimer { private: () }
}

pub struct NrfSleepTimerAlarm {
    channel: AlarmChannel,
}

impl NrfSleepTimerAlarm {
    fn cancel_alarm(&mut self) {
        // Safety: Clearing the interrupt is not immediate. It might still fire.
        //         That's why interrupt context additionally synchronizes on
        //         alarm state.
        STATE.rtc_fire_and_inactivate_alarm(self.channel);
        STATE.rtc_release_alarm(self.channel);
    }
}

impl Cancelable for NrfSleepTimerAlarm {}

impl Drop for NrfSleepTimerAlarm {
    fn drop(&mut self) {
        self.cancel_alarm();
    }
}

struct NrfHighPrecisionTimerAlarm {
    timer_channel: usize,
}

impl NrfHighPrecisionTimerAlarm {
    fn cancel_alarm(&mut self) {
        // Safety: Inactivating the interrupt is atomic and idempotent.
        compiler_fence(Ordering::Release);
        TIMER0
            .intenclr()
            .write(|w| w.set_compare(self.timer_channel, true));
    }
}

impl Cancelable for NrfHighPrecisionTimerAlarm {}

impl Drop for NrfHighPrecisionTimerAlarm {
    fn drop(&mut self) {
        self.cancel_alarm();
    }
}

#[interrupt]
fn RTC0() {
    STATE.on_rtc_interrupt();
}

#[interrupt]
fn TIMER0() {
    STATE.on_timer_interrupt();
}

// Tick-to-ns conversion (and back).
struct TickConversion;
impl TickConversion {
    const NS_PER_S: u128 = 1_000_000_000;

    // The max number of RTC ticks representable in nanoseconds (~584 years):
    // max_ticks = ((2^64-1) ns / 10^9 ns/s) * rtc_frequency
    const MAX_RTC_INSTANT: NsInstant = NsInstant::from_ticks(u64::MAX);
    const MAX_RTC_TICKS: u64 = ((Self::MAX_RTC_INSTANT.ticks() as u128
        * NrfSleepTimer::FREQUENCY.to_Hz() as u128)
        / Self::NS_PER_S) as u64;

    // The max duration convertible with a maximum rounding error of one tick at
    // 64 bit precision. See inline comments in `duration_to_timer_ticks()` for
    // more details.
    const MAX_TIMER_DURATION: NsDuration = NsDuration::micros(33554432);

    const fn instant_to_rtc_tick(instant: NsInstant) -> u64 {
        // To keep ns-to-tick conversion cheap we avoid division while
        // minimizing rounding errors:
        //
        // rtc_ticks = (timestamp_ns / (10^9 ns/s)) * rtc_frequency_hz
        //           = (timestamp_ns / (10^9 ns/s)) * 32768 Hz
        //           = timestamp_ns * (2^15 / 10^9 ns)
        //           = timestamp_ns * (2^6 / 5^9 ns)
        //           = timestamp_ns * ((2^6 * 2^N) / (5^9 * 2^N ns))
        //           = (timestamp_ns * (2^(6+N) / 5^9 ns)) >> N
        //           = (timestamp_ns * M(N)) >> N where M(N) := 2^(6+N) / 5^9 ns
        //
        // We can now choose M(N) such that it provides maximum precision, i.e.
        // the largest N is chosen such that timestamp_ns_max * M(N) remains
        // representable. Calculating in 64 bits is not possible as we want to
        // be able to convert u64::MAX. It turns out that the largest N
        // representable in 128 bits is 78.

        // TODO: Trace 128bit math performance implications and measure
        //       alternatives.
        const N: u32 = 78;
        const MULTIPLIER: u128 = 2_u128.pow(6 + N) / 5_u128.pow(9);

        // Safety: We asserted above that the max representable instant in
        //         nanoseconds times the MULTIPLIER does not overflow.
        let integer_fraction = instant.ticks() as u128 * MULTIPLIER;

        // Safety: We can represent less nanoseconds in 64 bits than ticks, so
        //         casting down the end result is always safe.
        (integer_fraction >> N) as u64
    }

    const fn duration_to_timer_ticks(duration: NsDuration) -> u32 {
        debug_assert!(duration.ticks() <= TickConversion::MAX_TIMER_DURATION.ticks());

        // To keep ns-to-tick conversion cheap we avoid division while
        // compromising between range and rounding error:
        //
        // timer_ticks = (timestamp_ns / (10^9 ns/s)) * timer_frequency_hz
        //             = (timestamp_ns / (10^9 ns/s)) * 16 MHz
        //             = timestamp_ns * ((2^4 * 10^6) / (2^3 * 5^3 * 10^6) ns)
        //             = timestamp_ns * (2 / 5^3 ns)
        //             = timestamp_ns * ((2 * 2^N) / (5^3 * 2^N ns))
        //             = (timestamp_ns * (2^(1+N) / 5^3 ns)) >> N
        //             = (timestamp_ns * M(N)) >> N where M(N) := 2^(1+N) / 5^3 ns
        //
        //
        // As 128 bit multiplication is much more expensive than 64 bit
        // multiplication and high-precision timer activation should not exceed
        // a few seconds anyway, we choose M(N) and timestamp_ns_max such that
        // timestamp_ns_max * M(N) remains representable in 64 bit while
        // rounding error is not more than a single tick over the full range.
        //
        // A value of N:=35 and timestamp_ns_max ~ 33 seconds is the best
        // compromise that can be achieved under such restrictions.

        // TODO: Trace 128bit math performance implications and measure
        //       alternatives.
        const N: u32 = 35;
        const MULTIPLIER: u128 = 2_u128.pow(1 + N) / 5_u128.pow(3);

        // Safety: The max timer duration was chosen so that multiplying it with
        //         the MULTIPLIER does not overflow.
        let integer_fraction = duration.ticks() as u128 * MULTIPLIER;

        // Safety: We can represent less nanoseconds in 64 bits than ticks, so
        //         casting down the end result is always safe.
        (integer_fraction >> N) as u32
    }

    const fn instant_to_timer_ticks_with_epoch(timer_epoch: NsInstant, instant: NsInstant) -> u32 {
        debug_assert!(instant.ticks() > timer_epoch.ticks());
        Self::duration_to_timer_ticks(NsDuration::from_ticks(
            instant.ticks() - timer_epoch.ticks(),
        ))
    }

    const fn rtc_tick_to_instant(rtc_tick: u64) -> NsInstant {
        debug_assert!(rtc_tick <= Self::MAX_RTC_TICKS);

        // To keep tick-to-ns conversion cheap we avoid division:
        //
        // timestamp_ns = ticks * (1 / rtc_frequency_hz) * 10^9 ns/s
        //              = ticks * (1 / 32768 Hz) * 10^9 ns/s
        //              = (ticks * (10^9 / 2^15)) ns
        //              = (ticks * (5^9 / 2^6)) ns
        //              = ((ticks * 5^9) >> 6) ns
        const _: () = assert!(NrfSleepTimer::FREQUENCY.to_Hz() == 2_u64.pow(15));

        // Safety: Representing MAX_RTC_TICKS requires 50 bits. Multiplying by
        //         5^9 requires another 21 bits. We therefore have to calculate
        //         in 128 bits to ensure that the calculation cannot overflow.
        const MULTIPLIER: u128 = 5_u128.pow(9);
        let ns = (rtc_tick as u128 * MULTIPLIER) >> 6;

        // Safety: We checked above that the number of ticks given is less than
        //         the max ticks that are still representable in nanoseconds.
        //         Therefore casting down will always succeed.
        NsInstant::from_ticks(ns as u64)
    }

    const fn timer_ticks_to_duration(timer_ticks: u32) -> NsDuration {
        // To keep tick-to-ns conversion cheap we avoid division:
        //
        // timestamp_ns = ticks * (1 / timer_frequency_hz) * 10^9 ns/s
        //              = ticks * (1 / 16 MHz) * 10^9 ns/s
        //              = (ticks * ((2^3 * 5^3 * 10^6) / (2^4 * 10^6))) ns
        //              = (ticks * (5^3 / 2)) ns
        //              = ((ticks * 5^3) >> 1) ns

        // Safety: Representing the timer ticks requires 32 bits. Multiplying by
        //         5^3 requires another 7 bits. Calculating in 64 bits is
        //         therefore sufficient to ensure that the calculation cannot
        //         overflow. Note that the max timer ticks represent ~268s.
        const MULTIPLIER: u64 = 5_u64.pow(3);
        let ns = (timer_ticks as u64 * MULTIPLIER) >> 1;

        // Safety: We checked above that the number of ticks given is less than
        //         the max ticks that are still representable in nanoseconds.
        //         Therefore casting down will always succeed.
        NsDuration::from_ticks(ns)
    }

    const fn timer_ticks_to_instant_with_epoch(
        timer_epoch: NsInstant,
        timer_ticks: u32,
    ) -> NsInstant {
        let timer_duration = Self::timer_ticks_to_duration(timer_ticks);
        timer_epoch.checked_add_duration(timer_duration).unwrap()
    }
}

// Test conversion.
//
// Note: We do this in a const expression rather than a test so that we can also
//       prove proper "constification" of the conversion functions.
const _: () = {
    let max_rtc_tick = TickConversion::instant_to_rtc_tick(NsInstant::from_ticks(u64::MAX));
    assert!(max_rtc_tick == TickConversion::MAX_RTC_TICKS);

    // One RTC tick is ~30517 ns, the rounding error must be less.
    const EXPECTED_REMAINDER_RTC_NS: u64 = 17924;
    let rtc_tick_ns = TickConversion::rtc_tick_to_instant(max_rtc_tick).ticks();
    assert!(rtc_tick_ns == u64::MAX - EXPECTED_REMAINDER_RTC_NS);

    const EXPECTED_REMAINDER_TIMER_TICKS: u32 = ((NrfHighPrecisionTimer::FREQUENCY.to_Hz()
        * EXPECTED_REMAINDER_RTC_NS)
        / TickConversion::NS_PER_S as u64) as u32;
    let timer_ticks =
        TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(EXPECTED_REMAINDER_RTC_NS));
    assert!(timer_ticks == EXPECTED_REMAINDER_TIMER_TICKS);

    // One TIMER tick is 62.5 ns, the remaining rounding error must be less.
    const EXPECTED_REMAINDER_TIMER_NS: u64 = 49;
    let timer_ticks_ns = TickConversion::timer_ticks_to_duration(timer_ticks).ticks();
    assert!(u64::MAX - rtc_tick_ns - timer_ticks_ns == EXPECTED_REMAINDER_TIMER_NS);

    const EXPECTED_MAX_TIMER_TICKS: u32 = ((TickConversion::MAX_TIMER_DURATION.ticks()
        * NrfHighPrecisionTimer::FREQUENCY.to_Hz())
        / TickConversion::NS_PER_S as u64) as u32;
    let max_timer_ticks = TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(
        TickConversion::MAX_TIMER_DURATION.ticks(),
    ));
    // We accept a rounding error of one tick.
    assert!(max_timer_ticks == EXPECTED_MAX_TIMER_TICKS - 1);

    assert!(TickConversion::duration_to_timer_ticks(NsDuration::from_ticks(875)) == 13);
};

#[derive(Clone, Copy, Debug)]
pub struct NrfSleepTimer {
    // Private field to block direct instantiation.
    private: (),
}

impl NrfSleepTimer {
    pub const FREQUENCY: TimerRateU64<32_768> = TimerRateU64::from_raw(1);
}

impl SleepTimer for NrfSleepTimer {
    const TICK_PERIOD: NsDuration = Self::FREQUENCY.into_duration();
    const GUARD_TIME: NsDuration = NsDuration::micros(150);
    type Alarm = NrfSleepTimerAlarm;

    fn now(&self) -> NsInstant {
        let rtc_tick = STATE.rtc_now_tick();
        TickConversion::rtc_tick_to_instant(rtc_tick)
    }

    fn wait_until(&mut self, instant: NsInstant, cb: fn()) -> Result<Self::Alarm, TimerError> {
        let channel = AlarmChannel::SleepTimer;
        STATE.rtc_try_acquire_alarm(channel)?;
        STATE.rtc_wait_until(channel, instant, cb).inspect_err(|_| {
            STATE.rtc_release_alarm(channel);
        })
    }

    fn start_high_precision_timer(
        &self,
        at: OptionalNsInstant,
    ) -> Result<impl HighPrecisionTimer, TimerError> {
        STATE.rtc_try_acquire_alarm(AlarmChannel::HighPrecisionTimer)?;

        if let Some(at) = at.into() {
            // We need to start at least two timer ticks earlier so we can
            // schedule signals at non-zero ticks even accounting for a one-tick
            // PPI delay.
            const TIMER_OFFSET: NsDuration = TickConversion::timer_ticks_to_duration(2)
                // Adding 1ns to avoid rounding errors.
                .checked_add(NsDuration::from_ticks(1))
                .unwrap();

            let rtc_tick = TickConversion::instant_to_rtc_tick(at - TIMER_OFFSET);

            STATE
                .timer_try_start_at_rtc_tick(rtc_tick)
                .map_err(|_| TimerError::Overdue(at))?;
        } else {
            STATE.timer_start_at_next_rtc_tick();
        }

        // The high-precision timer will automatically be stopped and the
        // corresponding alarm channel released when dropping the running
        // radio timer instance.
        Ok(NrfHighPrecisionTimer { private: () })
    }
}

pub struct NrfHighPrecisionTimer {
    // Private field to block direct instantiation.
    private: (),
}

impl NrfHighPrecisionTimer {
    pub const FREQUENCY: TimerRateU64<16_000_000> = TimerRateU64::from_raw(1);

    fn timer_epoch() -> NsInstant {
        // Note: This only works if the timer interrupt is running at a higher
        //       priority than the scheduling context. As we own the interrupt,
        //       it won't be masked. Immediate synchronization blocks for at
        //       most one RTC tick.
        debug_assert!(NrfExcPrio::current() < State::RTC_INT_PRIO);
        while STATE.is_rtc_pending_immediate_synchronization() {
            wfe();
        }

        // Safety: We waited until the timer is synchronized.
        unsafe { STATE.timer_epoch() }
    }

    fn timer_ticks(instant: NsInstant) -> u32 {
        let timer_epoch = Self::timer_epoch();
        debug_assert_ne!(timer_epoch.ticks(), 0);

        let timer_ticks = TickConversion::instant_to_timer_ticks_with_epoch(timer_epoch, instant);

        // Account for PPI delay, see nRF52840 PS, section 6.16: The timer is
        // synchronized to the 16 MHz clock and runs at 16 MHz, therefore PPI
        // will delay the compare event by one timer tick.
        timer_ticks - 1
    }
}

/// The timer implementation is a compromise between resource usage and
/// flexibility.
///
/// The radio currently requires the following combinations of signals and
/// events (read the "\" character as "unless"):
///
/// STATE | METHOD         | SIGNALS              | EVENTS
/// =========================================================================
/// off   | sched_rx       | rxen                 | rx_enabled, framestarted
/// off   | sched_tx       | txen                 | framestarted
/// off   | switch_off     |                      | disabled
/// rx    | stop_listening | disable\framestarted | framestarted, disabled
/// rx    | sched_rx       |                      | rx_enabled*, framestarted
/// rx    | sched_tx       | disable, txen        | framestarted
/// rx    | sched_off      | disable**            | disabled**
/// tx    | sched_rx       |                      | rx_enabled, framestarted
/// tx    | sched_tx       |                      | framestarted
/// tx    | sched_off      |                      | disabled
///
/// *) If an IFS is given we disable and re-enable the radio.
///
/// **) Only if a frame was observed, otherwise the disabled event has already
///     been observed by the `stop_listening()` method.
///
/// Additionally we implement a GPIO test mode that requires a GPIO toggle
/// signal and a GPIO toggled event, possibly used concurrently.
///
/// For the radio, we allocate the following resources:
///
/// signals:
/// - rxen/txen: pre-configured PPI-Chs 20/21 + CC0
/// - disable: pre-configured PPI-Ch 22 + CC1
/// - disable unless...: like "disable" + PPI-ChGroup
///
/// events:
/// - rx_enabled/disabled: PPI-Ch1.tep + CC0
/// - framestarted: PPI-Ch2.tep + CC1
/// - ...unless framestarted: like "framestarted" + PPI-Ch2.fork
///
/// In test mode we use:
/// gpio toggle signal: PPI-Ch1 + CC0
/// gpio toggled event: PPI-Ch2 + CC1
///
/// As we only have four channels available, we need to assign several signals
/// and events to the same channels:
///
/// 1. Events will always occur after the corresponding signal, i.e. the signal
///    channel is systematically released before the event occurs. This allows
///    us to re-use the same channel for the rxen signal and the rx enabled
///    event pair (CC0).
///
/// 2. The rx enabled and disabled event pair is mutually exclusive which allows
///    us to let them occupy the same channel CC0.
///
/// 3. In the "disable unless framestart" case, CC1 initially contains the
///    timeout for the disable signal. If the framestart event triggers, it'll
///    overwrite the contents of CC1 which is then no longer needed.
///
/// We use one timer channel (CC2) to ensure that programmed timers are in the
/// future. We do so by capturing the current timer value _after_ programming
/// the signal.
///
/// The fourth timer channel (CC3) is used to ensure that the timer will never
/// overflow. It shorts a compare event for the max counter value with the stop
/// task.
///
/// We use the ppi channel enabled and timer interrupt bits to synchronize
/// resource access. This is possible as we ensure that there can be only a
/// single instance of the high-precision timer:
/// - We synchronize access to the high-precision timer via the sleep timer
///   interface.
/// - The high-precision timer cannot be cloned or copied.
///
/// Notably we have to ensure that programming a "signal unless event" plus also
/// programming an observation of the same event does not produce a race,
/// independently of the order in which these calls are made.
///
/// The following diagrams show in which order registers need to be programmed
/// to avoid races.
///
/// First consider the case where the call to `schedule_timed_signal_unless()`
/// precedes the call to `observe_event()`:
///
/// event_* ──> eep (3) --> chen (4) ─┬> tep (9) ──> task_capture
///                                   │
///                                   └> fork (3) ──> chg.task_dis ┐
///                                ┌───────────────────────────────┘
///                                ˅
/// event_compare ──> eep (1) ──> chen (2) ──> tep (1) ──> task_*
///  ˄
///  │
/// cc (0/5)
///
/// Initially (0) the compare register responsible for triggering the signal
/// must be zero.
///
/// Then we program (1) and enable (2) the PPI channel responsible for
/// triggering the signal. As the compare register is zero and the timer cannot
/// overflow, the signal cannot fire although the PPI channel is already
/// enabled.
///
/// Next we set up (3) and enable (4) the PPI channel that captures the event
/// and disables the signal. We use the fork register of the PPI channel to
/// disable the signal via a channel group.
///
/// If the event occurs while we finish setting up the remaining registers, it
/// will disable the PPI channel that fires the signal thereby avoiding a race.
/// This is the reason why we need to enable the signal path before we set up
/// the event path.
///
/// In a last step we can now set the compare register responsible for
/// triggering the signal. This will atomically enable the signal unless the
/// compare event lies in the past or the PPI channel has already been disabled
/// by an incoming event.
///
/// The following call to "observe event" will use the event path's "tep"
/// register to atomically start capturing the event, independently of the
/// "signal unless event" path.
///
/// Now consider calling `observe_event()` before the call to
/// `schedule_timed_signal_unless()`:
///
/// event_* ──> eep (1) --> chen (2) ─┬> tep (1) ──> task_capture
///                                   │
///                                   └> fork (5) ──> chg.task_dis ┐
///                                ┌───────────────────────────────┘
///                                ˅
/// event_compare ──> eep (3) ──> chen (4) ──> tep (3) ──> task_*
///  ˄
///  │
/// cc (0/6)
///
/// In this case the capturing task will first be connected to the event source
/// which immediately starts observing the event (1, 2).
///
/// Programming the signal is prepared like in the first case by initially
/// leaving the cc register zero (0), then setting up and enabling the signal
/// path (3, 4).
///
/// Next, setting the fork register (5) will ensure that an incoming event will
/// immediately disable the signal path. And finally, setting the compare
/// register for the signal (6) will enable it atomically unless the event had
/// occurred already.
impl HighPrecisionTimer for NrfHighPrecisionTimer {
    const TICK_PERIOD: NsDuration = Self::FREQUENCY.into_duration();

    fn schedule_timed_signal(&self, timed_signal: TimedSignal) -> Result<&Self, TimerError> {
        let TimedSignal { instant, signal } = timed_signal;
        let timer_ticks = Self::timer_ticks(instant);
        let timer_ticks = NonZero::new(timer_ticks).ok_or(TimerError::Overdue(instant))?;
        STATE
            .timer_try_schedule_signal(instant, timer_ticks, signal)
            .map(|_| self)
    }

    fn schedule_timed_signal_unless(
        &self,
        timed_signal: TimedSignal,
        event: HardwareEvent,
    ) -> Result<&Self, TimerError> {
        let TimedSignal { instant, signal } = timed_signal;
        let timer_ticks = Self::timer_ticks(instant);
        let timer_ticks = NonZero::new(timer_ticks).ok_or(TimerError::Overdue(instant))?;
        STATE
            .timer_try_schedule_signal_unless(instant, timer_ticks, signal, event)
            .map(|_| self)
    }

    fn wait_for(&mut self, signal: HardwareSignal, cb: fn()) -> impl Cancelable {
        STATE.timer_wait_for(signal, cb)
    }

    fn observe_event(&self, event: HardwareEvent) -> Result<&Self, TimerError> {
        // Note: This only works if the timer interrupt is running at a higher
        //       priority than the scheduling context. As we own the interrupt,
        //       it won't be masked. Immediate synchronization blocks for at
        //       most one RTC tick.
        debug_assert!(NrfExcPrio::current() < State::RTC_INT_PRIO);
        while STATE.is_rtc_pending_immediate_synchronization() {
            wfe();
        }

        STATE.timer_try_observe_event(event).map(|_| self)
    }

    fn poll_event(&self, event: HardwareEvent) -> OptionalNsInstant {
        let timer_ticks = STATE.timer_get_and_clear_captured_ticks(event);
        if timer_ticks > 0 {
            TickConversion::timer_ticks_to_instant_with_epoch(Self::timer_epoch(), timer_ticks)
                .into()
        } else {
            None.into()
        }
    }

    fn reset(&self) {
        // First disable all PPI channels so that no further signals and events
        // can be generated.
        PPI.chenclr()
            .write(|w| w.0 = STATE.ppi_channel_mask.load(Ordering::Relaxed));

        // Clean up timer resources.
        for timer_channel in 0..2 {
            TIMER0.events_compare(timer_channel).write_value(0);
            TIMER0.cc(timer_channel).write_value(0);
        }
        #[cfg(feature = "timer-trace")]
        {
            let gpiote_channel = STATE.gpiote_in_channel.load(Ordering::Relaxed);
            GPIOTE.events_in(gpiote_channel).write_value(0);
        }

        TIMER0.tasks_clear().write_value(0x1);

        // Release the PPI channel group.
        let ppi_channel_group = STATE.ppi_channel_group.load(Ordering::Relaxed);
        PPI.chg(ppi_channel_group).write(|w| w.0 = 0);
    }
}

impl Drop for NrfHighPrecisionTimer {
    fn drop(&mut self) {
        // First of all stop the timer to ensure that further signals won't be
        // generated. This is atomic and doesn't have to be synchronized.
        TIMER0.tasks_stop().write_value(0x1);

        // Atomically cancel ongoing timer synchronization and re-acquire alarm
        // memory to scheduling context in which the drop handler runs.
        STATE.rtc_fire_and_inactivate_alarm(AlarmChannel::HighPrecisionTimer);

        // In case synchronization was interrupted: Clean up registers that were
        // used for synchronization.
        #[cfg(not(feature = "timer-trace"))]
        RTC0.evtenclr().write(|w| w.set_tick(true));
        TIMER0.intenclr().write(|w| w.set_compare(1, true));

        // Clean up timer state.
        self.reset();

        // Atomically de-allocate the high precision timer.
        STATE.rtc_release_alarm(AlarmChannel::HighPrecisionTimer)
    }
}
