//! This module exposes a generic hybrid timer that fuses an always-on sleep
//! timer with a high-precision timer. The module also contains supporting time
//! structures.
//!
//! Hybrid timer:
//!
//! - [`SleepTimer`] represents the cloneable entry point to the always-on
//!   sleep-timer part of the hybrid radio timer. It can be used as a
//!   coarse-grained timer on its own and exposes a method to start the
//!   high-precision timer off of one of its own ticks.
//!
//! - [`HighPrecisionTimer`] exposes various methods to schedule timed hardware
//!   signals (see [`TimedSignal`] and [`HardwareSignal`]) and observe the
//!   timing of hardware events (see [`HardwareEvent`]) without CPU
//!   intervention. Signals and events are expressed as hardware-agnostic types.
//!
//! Time structures:
//!
//! - [`NsInstant`] and [`OptionalNsInstant`] are used to represent a
//!   nanosecond-precision point in time.
//! - [`NsDuration`] is used to represent a nanosecond-precision duration.

use core::num::NonZeroU64;

use fugit::NanosDurationU64;

pub mod export {
    pub use fugit::{Duration, ExtU64, Instant};
}

use export::*;

use crate::util::sync::cancellation::Cancelable;

pub type NsInstant = Instant<u64, 1, 1_000_000_000>;
pub type NsDuration = NanosDurationU64;

/// An optional, non-zero instant.
///
/// We often need optional instants. An optional, 64-bit zeroable instant uses
/// 16 bytes.
///
/// Local clock instants will almost never be zero, though. This alternative
/// interprets a zero instant as [`None`] thereby halving the amount of space
/// required for a non-zero optional instant.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct OptionalNsInstant {
    inner: NsInstant,
}

impl OptionalNsInstant {
    #[inline]
    pub const fn none() -> Self {
        Self {
            inner: NsInstant::from_ticks(0),
        }
    }

    #[inline]
    pub const fn from_ticks(ticks: NonZeroU64) -> Self {
        Self {
            inner: NsInstant::from_ticks(ticks.get()),
        }
    }

    #[inline]
    pub const fn try_from_ticks(ticks: u64) -> Self {
        Self {
            inner: NsInstant::from_ticks(ticks),
        }
    }

    #[inline]
    pub const fn is_some(&self) -> bool {
        self.inner.ticks() > 0
    }

    #[inline]
    pub const fn is_none(&self) -> bool {
        !self.is_some()
    }

    #[inline]
    pub const fn set(&mut self, instant: NsInstant) {
        assert!(instant.ticks() > 0);
        self.inner = instant
    }

    #[inline]
    pub const fn unwrap(self) -> NsInstant {
        if self.is_some() { self.inner } else { panic!() }
    }

    #[inline]
    pub fn map<U, F>(self, f: F) -> Option<U>
    where
        F: FnOnce(NsInstant) -> U,
    {
        match self.inner.ticks() {
            0 => None,
            ticks => Some(f(NsInstant::from_ticks(ticks))),
        }
    }

    #[inline]
    pub fn map_or_else<U, D, F>(self, default: D, f: F) -> U
    where
        D: FnOnce() -> U,
        F: FnOnce(NsInstant) -> U,
    {
        match self.inner.ticks() {
            0 => default(),
            ticks => f(NsInstant::from_ticks(ticks)),
        }
    }

    #[inline]
    pub fn ok_or_else<E, F>(self, err: F) -> Result<NsInstant, E>
    where
        F: FnOnce() -> E,
    {
        match self.inner.ticks() {
            0 => Err(err()),
            ticks => Ok(NsInstant::from_ticks(ticks)),
        }
    }
}

impl From<OptionalNsInstant> for Option<NsInstant> {
    #[inline]
    fn from(value: OptionalNsInstant) -> Self {
        let instant = value.inner;
        if instant.ticks() > 0 {
            Some(instant)
        } else {
            None
        }
    }
}

impl From<NsInstant> for OptionalNsInstant {
    #[inline]
    fn from(value: NsInstant) -> Self {
        assert!(value.ticks() > 0);
        Self { inner: value }
    }
}

impl From<Option<NsInstant>> for OptionalNsInstant {
    #[inline]
    fn from(value: Option<NsInstant>) -> Self {
        match value {
            Some(instant) => instant.into(),
            None => OptionalNsInstant::none(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimerError {
    /// The instant or signal could not be safely scheduled, e.g. due to guard
    /// time restrictions or because the scheduled instant is in the past.
    ///
    /// The operation returned at an arbitrary time before or after the
    /// scheduled instant or event occurrence.
    ///
    /// The offending instant is being returned.
    Overdue(NsInstant),

    /// The event cannot be observed as it already occurred.
    Already,

    /// The timer operation could not be scheduled due to lack of resources
    /// (e.g. lack of free timer channels).
    Busy,
}

/// Hardware signals are an abstraction over electrical signals that can be sent
/// across an event bus as usually found on radio hardware.
///
/// Note: The architecture and implementation of hardware signals and event
///       buses varies widely across SoCs and transceivers. A good abstraction
///       needs to emerge over time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum HardwareSignal {
    /// Enable radio reception.
    RadioRxEnable,

    /// Enable radio transmission.
    RadioTxEnable,

    /// Cancel any ongoing radio reception/transmission and transition the radio
    /// into a low-energy state.
    RadioDisable,

    /// Toggle the outbound alarm pin.
    #[cfg(feature = "timer-trace")]
    GpioToggle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum HardwareEvent {
    /// An event indicating that the radio receiver was enabled. Exact timing is
    /// implementation specific.
    RadioRxEnabled,

    /// An event indicating the start of frame reception. Exact timing is
    /// implementation specific.
    RadioFrameStarted,

    /// An event indicating that the radio was disabled. Exact timing is
    /// implementation specific.
    RadioDisabled,

    /// A toggle event on the inbound alarm pin.
    #[cfg(feature = "timer-trace")]
    GpioToggled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TimedSignal {
    pub instant: NsInstant,
    pub signal: HardwareSignal,
}

impl TimedSignal {
    pub const fn new(instant: NsInstant, signal: HardwareSignal) -> Self {
        Self { instant, signal }
    }
}

pub trait SleepTimer: Copy {
    /// Exposes the timer's implementation-specific tick period.
    const TICK_PERIOD: NsDuration;

    /// This is the worst case execution time of the
    /// [`SleepTimer::wait_until()`] method of the underlying sleep timer
    /// implementation. Scheduling a timer at `instant - guard time`` should
    /// always succeed unless a higher-priority interrupt preempts execution.
    const GUARD_TIME: NsDuration;

    /// A type representing a scheduled alarm.
    type Alarm: Cancelable;

    /// Returns a recent instant of the local radio clock's coarse sleep timer.
    ///
    /// Note: This method involves the CPU and therefore will always return a
    ///       past instant while the timer continues to tick concurrently.
    fn now(&self) -> NsInstant;

    /// Allows clients to wait until the given instant using only the sleep
    /// timer.
    ///
    /// Clients register a callback that will be called precisely once when the
    /// scheduled alarm fires.
    ///
    /// The method returns a guard object that, if dropped before the alarm
    /// fires, will cancel the alarm. Dropping the guard after the alarm fired
    /// is a no-op.
    ///
    /// Note: A callback invariably involves CPU activity that cannot be
    ///       controlled by the timer. To reduce latency, eliminate jitter, and
    ///       establish a well-defined WCET, you should drive the timer from a
    ///       dedicated interrupt running at an appropriate priority. The
    ///       callback MAY be called from interrupt context.
    fn wait_until(&mut self, instant: NsInstant, cb: fn()) -> Result<Self::Alarm, TimerError>;

    /// Tries to allocate and start a high precision timer instance that is
    /// synchronized with the sleep clock. Returns an error if no timer instance
    /// can be allocated.
    ///
    /// If a start time is given and the method returns without an error, then
    /// the timer is guaranteed to be started before the given instant. The
    /// start time must observe [`SleepTimer::GUARD_TIME`].
    ///
    /// If no start time is given, then the timer starts as fast as possible,
    /// i.e.  at the next available sleep timer tick. This MAY be faster than
    /// [`SleepTimer::GUARD_TIME`] but SHALL NOT be slower.
    ///
    /// The returned object serves as a token representing the running timer.
    /// Dropping it will cancel, stop and de-allocate the timer.
    ///
    /// Note: The sleep timer MAY be used concurrently with high-precision
    ///       timers.
    ///
    /// # Panics
    ///
    /// The high precision timer SHOULD only run for short time spans (at most a
    /// few milliseconds) as it will not be syntonized. Always start it as late
    /// as possible. The high precision timer does not implement overflow
    /// protection. It MAY panic or stop counting if it saturates the counter
    /// register.
    fn start_high_precision_timer(
        &self,
        at: OptionalNsInstant,
    ) -> Result<impl HighPrecisionTimer, TimerError>;
}

/// Represents a started high-precision timer. MAY be dropped to stop and
/// de-allocate the timer.
pub trait HighPrecisionTimer {
    /// Exposes the timer's implementation-specific tick period.
    const TICK_PERIOD: NsDuration;

    /// Programs a hardware signal to be sent over the event bus at a precise
    /// instant.
    ///
    /// This method provides access to deterministically timed signals at
    /// hardware level without CPU intervention. Exact timing specifications are
    /// implementation dependent.
    ///
    /// Returns an error if the timed signal could not be scheduled, e.g. due to
    /// lack of resources ([`TimerError::Busy`]) or late scheduling
    /// ([`TimerError::Overdue`]).
    fn schedule_timed_signal(&self, timed_signal: TimedSignal) -> Result<&Self, TimerError>;

    /// Programs a hardware signal to be sent over the event bus at a precise
    /// instant unless the given event happens before.
    ///
    /// Returns an error if the timed signal could not be scheduled, e.g. due to
    /// lack of resources ([`TimerError::Busy`]) or late scheduling
    /// ([`TimerError::Overdue`]).
    ///
    /// Returns [`TimerError::Already`] if the event was already pending when
    /// calling the method.
    fn schedule_timed_signal_unless(
        &self,
        timed_signal: TimedSignal,
        event: HardwareEvent,
    ) -> Result<&Self, TimerError>;

    /// Allows clients to wait until a scheduled signal has been executed.
    ///
    /// Clients register a callback that will be called precisely once when the
    /// scheduled alarm fires.
    ///
    /// The method returns a guard object that, if dropped before the alarm
    /// fires, will cancel the callback (but not the scheduled signal).
    /// Dropping the guard after the signal has been asserted is a no-op.
    ///
    /// Note: A callback invariably involves CPU activity that cannot be
    ///       controlled by the timer. To reduce latency, eliminate jitter, and
    ///       establish a well-defined WCET, you should drive the timer from a
    ///       dedicated interrupt running at an appropriate priority. The
    ///       callback MAY be called from interrupt context.
    ///
    /// # Panics
    ///
    /// Panics if neither [`HighPrecisionTimer::schedule_timed_signal()`] nor
    /// [`HighPrecisionTimer::schedule_timed_signal_unless()`] has previously
    /// been called for the given signal.
    fn wait_for(&mut self, signal: HardwareSignal, cb: fn()) -> impl Cancelable;

    /// Prepares the timer to listen for a hardware event and capture the
    /// high-precision timestamp of the event if it occurs.
    ///
    /// Returns [`TimerError::Busy`] if the timer cannot observe the event due
    /// to lack of resources.
    ///
    /// Returns [`TimerError::Already`] if the event was already pending when
    /// calling the method.
    ///
    /// This method SHALL be idempotent, i.e. if the event is already being
    /// observed, then calling this method SHALL be a no-op and return
    /// successfully.
    ///
    /// See [`HighPrecisionTimer::poll_event`].
    fn observe_event(&self, event: HardwareEvent) -> Result<&Self, TimerError>;

    /// Returns the timestamp observed by [`HighPrecisionTimer::observe_event`].
    /// Returns [`None`] if the corresponding event was not observed.
    ///
    /// In case an event was observed, calling this method will release
    /// corresponding timer resources so that they can be re-used by future
    /// calls to [`HighPrecisionTimer::observe_event()`].
    ///
    /// # Panics
    ///
    /// Panics if [`HighPrecisionTimer::observe_event()`] had not been called
    /// for this event or if the event had already been collected by a prior
    /// call to this method.
    fn poll_event(&self, event: HardwareEvent) -> OptionalNsInstant;

    /// Removes all scheduled signals and observed events but leaves the
    /// high-precision timer running.
    fn reset(&self);
}
