use core::{marker::PhantomData, mem};

use crate::driver::executor::{Interrupt, Task};

pub(crate) struct RadioDriver<Soc, State> {
    soc: PhantomData<Soc>,
    state: PhantomData<State>,
}

impl<Soc, State> RadioDriver<Soc, State> {
    /// This internal method switches the typestate of the radio driver. It must
    /// only be used once an implementation has proven that the new state has
    /// been reached.
    pub(crate) fn enter_state() -> Self {
        Self {
            soc: PhantomData,
            state: PhantomData,
        }
    }
}

pub(crate) trait RadioState: Sized {
    fn transition(&mut self) -> Task<(), ()>;
    fn complete(self) -> Task<Self, ()>;
}

pub(crate) struct Off;
pub(crate) trait OffState {
    fn schedule_rx(&mut self);
    fn schedule_tx(&mut self);
}

pub(crate) struct Rx;
pub(crate) trait RxState {
    fn schedule_tx(&mut self);
    fn schedule_off(&mut self);
}

pub(crate) struct Tx;
pub(crate) trait TxState {
    fn schedule_rx(&mut self);
    fn schedule_off(&mut self);
}

#[allow(private_interfaces)]
pub(crate) enum RadioDriverState<Soc> {
    Off(RadioDriver<Soc, Off>),
    OffToRx(RadioDriverTransition<Soc, Off, Rx>),
    OffToTx(RadioDriverTransition<Soc, Off, Tx>),
    Rx(RadioDriver<Soc, Rx>),
    RxToTx(RadioDriverTransition<Soc, Rx, Tx>),
    RxToOff(RadioDriverTransition<Soc, Rx, Off>),
    Tx(RadioDriver<Soc, Tx>),
    TxToRx(RadioDriverTransition<Soc, Tx, Rx>),
    TxToOff(RadioDriverTransition<Soc, Tx, Off>),

    /// An intermediate state that allows us to take temporary ownership of the
    /// inner value.
    Executing,
}

#[derive(Clone, Copy)]
pub enum RadioTask {
    Rx,
    Tx,
    Off,
}

impl<Soc> RadioDriverState<Soc>
where
    RadioDriver<Soc, Off>: OffState,
    RadioDriver<Soc, Rx>: RxState,
    RadioDriver<Soc, Tx>: TxState,
{
    pub(crate) fn schedule(&mut self, task: RadioTask) {
        use RadioDriverTransition as Transition;
        use RadioTask as Task;

        let this = mem::replace(self, Self::Executing);
        let next_state = match task {
            Task::Rx => match this {
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
            Task::Tx => match this {
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
            Task::Off => match this {
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
        };
        *self = next_state;
    }
}

#[allow(private_interfaces)]
pub(crate) enum RadioDriverTransition<Soc, FromState, ToState> {
    /// We're waiting while the source state is driven to completion.
    CompletingSourceState(RadioDriver<Soc, FromState>),

    /// We're waiting while the target state is entering.
    EnteringTargetState(RadioDriver<Soc, ToState>),

    /// The final state of the transition.
    EnteredTargetState(RadioDriver<Soc, ToState>),

    /// An intermediate state that allows us to take temporary ownership of the
    /// radio driver.
    Executing,
}

impl<Soc, FromState, ToState> RadioDriverTransition<Soc, FromState, ToState> {
    pub(crate) fn target_state(self) -> RadioDriver<Soc, ToState> {
        match self {
            RadioDriverTransition::EnteredTargetState(driver) => driver,
            _ => unreachable!(),
        }
    }
}

impl<Soc, FromState, ToState> RadioDriverTransition<Soc, FromState, ToState>
where
    RadioDriver<Soc, FromState>: RadioState,
    RadioDriver<Soc, ToState>: RadioState,
{
    pub(crate) fn on_interrupt(&mut self) -> Interrupt {
        match mem::replace(self, Self::Executing) {
            Self::CompletingSourceState(driver) => {
                *self = match driver.complete() {
                    Task::Continue(driver) => Self::CompletingSourceState(driver),
                    Task::Completed(_) => Self::EnteringTargetState(RadioDriver::enter_state()),
                };
                Interrupt::Continue
            }
            Self::EnteringTargetState(mut driver) => match driver.transition() {
                Task::Continue(_) => {
                    *self = Self::EnteringTargetState(driver);
                    Interrupt::Continue
                }
                Task::Completed(_) => {
                    *self = Self::EnteredTargetState(driver);
                    Interrupt::Completed
                }
            },
            _ => unreachable!(),
        }
    }
}
