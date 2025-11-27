use core::{marker::PhantomData, ops::DerefMut};

use cortex_m::{Peripherals, interrupt::InterruptNumber, peripheral::NVIC};

use crate::util::sync::{
    init_cell::InitCell,
    priority::{ExecutionPriority, PriorityBits},
    priority_cell::{PriorityCell, PriorityCellRef},
};

/// A value representing intermediate and final results of an asynchronous task.
///
/// Used to model tasks aka state machines aka generators that (optionally)
/// produce intermediate and/or final results.
pub enum Task<IntermediateOutput, FinalOutput> {
    /// The task/generator has produced an intermediate result but has further
    /// work to do.
    Continue(IntermediateOutput),

    /// The task/generator has completed and produced its final result.
    Completed(FinalOutput),
}

/// A value representing the state of a dynamic interrupt handler.
pub enum Interrupt {
    /// The interrupt handler has more work to do. The interrupt remains
    /// unmasked.
    Continue,

    /// The interrupt handler has finished. It can be de-installed and the
    /// interrupt masked.
    Completed,
}

/// Dynamic interrupt handler.
pub trait InterruptHandler<Ctx> {
    fn on_interrupt(&mut self, ctx: Ctx) -> Interrupt;
}

/// Interrupt execution context.
///
/// Stores the interrupt number and priority of the interrupt.
pub trait InterruptContext: InterruptNumber {
    type PB: PriorityBits;
    fn arm_nvic_priority(&self) -> u8;
    fn interrupt_contexts<'i>() -> &'i [Self];
}

/// Static-friendly storage for state that is initialized at program start from
/// thread context and then jointly used across several interrupt contexts.
pub struct SharedState<T: Send, I: InterruptContext> {
    state: InitCell<PriorityCell<T>>,
    interrupt_context: PhantomData<I>,
}

impl<T: Send, I: InterruptContext> SharedState<T, I> {
    /// Static-friendly constructor of the shared state holder.
    pub const fn new(state: T) -> Self {
        Self {
            state: InitCell::new(PriorityCell::new(
                state,
                ExecutionPriority::<I::PB>::THREAD.to_arm_nvic_repr(),
            )),
            interrupt_context: PhantomData,
        }
    }

    /// Initialize the state and its execution contexts.
    ///
    /// Must be called from thread context.
    pub fn init_and_send(&self, init_state: impl FnOnce(&mut T), interrupt_context: I) {
        fn init_priorities<I: InterruptContext>() {
            unsafe {
                // Safety: We exclusively own the interrupts.
                let mut nvic = Peripherals::steal().NVIC;
                for interrupt_context in I::interrupt_contexts() {
                    // Safety: We do not rely on priority masking to share
                    //         resources.
                    nvic.set_priority(*interrupt_context, interrupt_context.arm_nvic_priority());

                    // Safety: We do not rely on masking to share resources.
                    NVIC::unmask(*interrupt_context);
                }
            }
        }

        self.state.init(|state_cell| {
            init_priorities::<I>();
            let mut state_borrow = state_cell.borrow_mut();
            init_state(state_borrow.deref_mut());
        });

        self.send(interrupt_context);
    }

    /// Exclusively borrows the state from the current execution context.
    ///
    /// See [`PriorityCell::borrow_mut()`].
    pub fn borrow_mut(&self) -> PriorityCellRef<T> {
        self.state.borrow_mut()
    }

    /// Sends the state to a different interrupt and pends that interrupt.
    ///
    /// See [`PriorityCell::release()`].
    pub fn send(&self, interrupt_context: I) {
        self.state.release(
            self.state.borrow_mut(),
            interrupt_context.arm_nvic_priority(),
        );
        NVIC::pend(interrupt_context);
    }

    /// Sends the borrowed state to a different interrupt and pends that
    /// interrupt.
    ///
    /// See [`PriorityCell::release()`].
    pub fn release_and_send(&self, cell_ref: PriorityCellRef<T>, interrupt_context: I) {
        self.state
            .release(cell_ref, interrupt_context.arm_nvic_priority());
        NVIC::pend(interrupt_context);
    }

    /// Sends the state to an interrupt running at a lower priority and pends
    /// that interrupt.
    ///
    /// See [`PriorityCell::release_to_lower()`].
    pub fn send_to_lower(&self, interrupt_context: I) {
        self.state
            .release_to_lower(interrupt_context.arm_nvic_priority());
        NVIC::pend(interrupt_context);
    }
}
