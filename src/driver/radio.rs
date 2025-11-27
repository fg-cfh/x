use core::{marker::PhantomData, mem};

use crate::{
    driver::{
        task::{Interrupt, InterruptHandler, Task},
        timer::SleepTimer,
    },
    util::sync::cancellation::Cancelable,
};

pub trait DriverConfig {
    /// Radio timer implementation.
    type Timer: SleepTimer;

    /// Radio driver implementation: off state.
    type OffState;

    /// Radio driver implementation: rx state.
    type RxState;

    /// Radio driver implementation: tx state.
    type TxState;
}

/// The radio driver context is shared by all radio states. It SHALL be
/// instantiated once and MAY thereafter only be mutated.
pub(crate) struct RadioDriverCtx<Config: DriverConfig> {
    /// Sleep timer instance.
    sleep_timer: Option<Config::Timer>,

    /// Currently running sleep timer alarm if any.
    pub(crate) sleep_timer_alarm: Option<<Config::Timer as SleepTimer>::Alarm>,
}

impl<Config: DriverConfig> RadioDriverCtx<Config> {
    pub(crate) fn sleep_timer(&self) -> Config::Timer {
        self.sleep_timer.unwrap()
    }

    pub(crate) fn init(&mut self, sleep_timer: Config::Timer) {
        assert!(self.sleep_timer.replace(sleep_timer).is_none());
    }
}

/// This structure stores radio driver state machine. It is a wrapper around the
/// radio driver context (extended state) and the radio driver state.
pub(crate) struct RadioDriverStateMachine<Config: DriverConfig> {
    /// Driver context is shared across all driver states. This is mutable
    /// extended state structurally shared between all radio states.
    pub(crate) ctx: RadioDriverCtx<Config>,

    /// The current driver state.
    pub(crate) driver: RadioDriver<Config>,
}

impl<Config: DriverConfig> RadioDriverStateMachine<Config> {
    /// The given radio driver SHALL be in "Radio Off" state.
    pub(crate) const fn new_unchecked(driver: RadioDriver<Config>) -> Self {
        Self {
            ctx: RadioDriverCtx {
                sleep_timer: None,
                sleep_timer_alarm: None,
            },
            driver,
        }
    }
}

impl<Config: DriverConfig> RadioDriverStateMachine<Config>
where
    RadioDriverState<Config, Config::OffState>: OffState,
    RadioDriverState<Config, Config::RxState>: RxState,
    RadioDriverState<Config, Config::TxState>: TxState,
{
    /// Delegates to [`RadioDriver::schedule()`]
    pub(crate) fn schedule(&mut self, task: RadioTask<Config>) {
        self.driver.schedule(task);
    }
}

impl<Config: DriverConfig> InterruptHandler<()> for RadioDriverStateMachine<Config>
where
    RadioDriverTransition<Config, Config::OffState, Config::RxState>:
        for<'ctx> InterruptHandler<&'ctx mut RadioDriverCtx<Config>>,
    RadioDriverTransition<Config, Config::OffState, Config::TxState>:
        for<'ctx> InterruptHandler<&'ctx mut RadioDriverCtx<Config>>,
    RadioDriverTransition<Config, Config::RxState, Config::TxState>:
        for<'ctx> InterruptHandler<&'ctx mut RadioDriverCtx<Config>>,
    RadioDriverTransition<Config, Config::RxState, Config::OffState>:
        for<'ctx> InterruptHandler<&'ctx mut RadioDriverCtx<Config>>,
    RadioDriverTransition<Config, Config::TxState, Config::RxState>:
        for<'ctx> InterruptHandler<&'ctx mut RadioDriverCtx<Config>>,
    RadioDriverTransition<Config, Config::TxState, Config::OffState>:
        for<'ctx> InterruptHandler<&'ctx mut RadioDriverCtx<Config>>,
{
    fn on_interrupt(&mut self, _: ()) -> Interrupt {
        use Interrupt::*;
        use RadioDriver as R;
        let ctx = &mut self.ctx;
        let driver_ref = &mut self.driver;
        let driver = mem::replace(driver_ref, RadioDriver::Executing);
        let (updated_driver, result) = match driver {
            R::OffToRx(mut transition) => match transition.on_interrupt(ctx) {
                result @ Continue => (R::OffToRx(transition), result),
                result @ Completed => (R::Rx(transition.target_state()), result),
            },
            R::OffToTx(mut transition) => match transition.on_interrupt(ctx) {
                result @ Continue => (R::OffToTx(transition), result),
                result @ Completed => (R::Tx(transition.target_state()), result),
            },
            R::RxToTx(mut transition) => match transition.on_interrupt(ctx) {
                result @ Continue => (R::RxToTx(transition), result),
                result @ Completed => (R::Tx(transition.target_state()), result),
            },
            R::RxToOff(mut transition) => match transition.on_interrupt(ctx) {
                result @ Continue => (R::RxToOff(transition), result),
                result @ Completed => (R::Off(transition.target_state()), result),
            },
            R::TxToRx(mut transition) => match transition.on_interrupt(ctx) {
                result @ Continue => (R::TxToRx(transition), result),
                result @ Completed => (R::Rx(transition.target_state()), result),
            },
            R::TxToOff(mut transition) => match transition.on_interrupt(ctx) {
                result @ Continue => (R::TxToOff(transition), result),
                result @ Completed => (R::Off(transition.target_state()), result),
            },
            _ => unreachable!(),
        };
        *driver_ref = updated_driver;
        result
    }
}

/// This structure stores radio driver state machine's state-specific state. It
/// will be re-instantiated whenever a new radio state enters.
pub(crate) struct RadioDriverState<Config, State> {
    /// Access to the driver configuration.
    config: PhantomData<Config>,
    /// Implementation specific state.
    state: State,
}

impl<Config, State> RadioDriverState<Config, State> {
    /// This internal method switches the typestate of the radio driver. It must
    /// only be used once an implementation has proven that the new state has
    /// been reached.
    pub(crate) const fn enter_state(state: State) -> Self {
        Self {
            config: PhantomData,
            state,
        }
    }
}

pub(crate) trait RadioState<Config: DriverConfig>: Sized {
    fn transition(&mut self, ctx: &mut RadioDriverCtx<Config>) -> Task<(), ()>;
    fn complete(self, ctx: &mut RadioDriverCtx<Config>) -> Task<Self, ()>;
}

pub(crate) trait OffState {
    fn schedule_rx(&mut self);
    fn schedule_tx(&mut self);
}

pub(crate) trait RxState {
    fn schedule_tx(&mut self);
    fn schedule_off(&mut self);
}

pub(crate) trait TxState {
    fn schedule_rx(&mut self);
    fn schedule_off(&mut self);
}

pub(crate) enum RadioDriver<Config: DriverConfig> {
    Off(RadioDriverState<Config, Config::OffState>),
    OffToRx(RadioDriverTransition<Config, Config::OffState, Config::RxState>),
    OffToTx(RadioDriverTransition<Config, Config::OffState, Config::TxState>),
    Rx(RadioDriverState<Config, Config::RxState>),
    RxToTx(RadioDriverTransition<Config, Config::RxState, Config::TxState>),
    RxToOff(RadioDriverTransition<Config, Config::RxState, Config::OffState>),
    Tx(RadioDriverState<Config, Config::TxState>),
    TxToRx(RadioDriverTransition<Config, Config::TxState, Config::RxState>),
    TxToOff(RadioDriverTransition<Config, Config::TxState, Config::OffState>),

    /// An intermediate state that allows us to take temporary ownership of the
    /// inner value.
    Executing,
}

#[derive(Clone, Copy)]
pub enum RadioTask<Config: DriverConfig> {
    Off(Config::OffState),
    Rx(Config::RxState),
    Tx(Config::TxState),
}

impl<Config: DriverConfig> RadioDriver<Config>
where
    RadioDriverState<Config, Config::OffState>: OffState,
    RadioDriverState<Config, Config::RxState>: RxState,
    RadioDriverState<Config, Config::TxState>: TxState,
{
    pub(crate) fn schedule(&mut self, task: RadioTask<Config>) {
        use RadioDriverTransition as Transition;
        use RadioTask as Task;

        let this = mem::replace(self, Self::Executing);
        let next_state = match task {
            Task::Rx(rx_state) => match this {
                Self::Off(mut driver) => {
                    driver.schedule_rx();
                    Self::OffToRx(Transition::CompletingSourceState(driver, rx_state))
                }
                Self::Tx(mut driver) => {
                    driver.schedule_rx();
                    Self::TxToRx(Transition::CompletingSourceState(driver, rx_state))
                }
                _ => unreachable!(),
            },
            Task::Tx(tx_state) => match this {
                Self::Off(mut driver) => {
                    driver.schedule_tx();
                    Self::OffToTx(Transition::CompletingSourceState(driver, tx_state))
                }
                Self::Rx(mut driver) => {
                    driver.schedule_tx();
                    Self::RxToTx(Transition::CompletingSourceState(driver, tx_state))
                }
                _ => unreachable!(),
            },
            Task::Off(off_state) => match this {
                Self::Tx(mut driver) => {
                    driver.schedule_off();
                    Self::TxToOff(Transition::CompletingSourceState(driver, off_state))
                }
                Self::Rx(mut driver) => {
                    driver.schedule_off();
                    Self::RxToOff(Transition::CompletingSourceState(driver, off_state))
                }
                _ => unreachable!(),
            },
        };
        *self = next_state;
    }
}

#[allow(private_interfaces)]
pub(crate) enum RadioDriverTransition<Config, FromState, ToState> {
    /// We're waiting while the source state is driven to completion.
    CompletingSourceState(RadioDriverState<Config, FromState>, ToState),

    /// We're waiting while the target state is entering.
    EnteringTargetState(RadioDriverState<Config, ToState>),

    /// The final state of the transition.
    EnteredTargetState(RadioDriverState<Config, ToState>),

    /// An intermediate state that allows us to take temporary ownership of the
    /// radio driver.
    Executing,
}

impl<Soc, FromState, ToState> RadioDriverTransition<Soc, FromState, ToState> {
    pub(crate) fn target_state(self) -> RadioDriverState<Soc, ToState> {
        match self {
            RadioDriverTransition::EnteredTargetState(driver) => driver,
            _ => unreachable!(),
        }
    }
}

impl<Config: DriverConfig, FromState, ToState> InterruptHandler<&mut RadioDriverCtx<Config>>
    for RadioDriverTransition<Config, FromState, ToState>
where
    RadioDriverState<Config, FromState>: RadioState<Config>,
    RadioDriverState<Config, ToState>: RadioState<Config>,
{
    fn on_interrupt(&mut self, ctx: &mut RadioDriverCtx<Config>) -> Interrupt {
        loop {
            match mem::replace(self, Self::Executing) {
                Self::CompletingSourceState(driver, to_state) => {
                    match driver.complete(ctx) {
                        Task::Continue(driver) => {
                            *self = Self::CompletingSourceState(driver, to_state);
                            return Interrupt::Continue;
                        }
                        Task::Completed(_) => {
                            *self =
                                Self::EnteringTargetState(RadioDriverState::enter_state(to_state));
                            continue;
                        }
                    };
                }
                Self::EnteringTargetState(mut driver) => match driver.transition(ctx) {
                    Task::Continue(_) => {
                        *self = Self::EnteringTargetState(driver);
                        return Interrupt::Continue;
                    }
                    Task::Completed(_) => {
                        *self = Self::EnteredTargetState(driver);
                        return Interrupt::Completed;
                    }
                },
                _ => unreachable!(),
            }
        }
    }
}
