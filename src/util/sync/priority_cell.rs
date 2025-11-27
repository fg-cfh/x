use core::{
    arch::asm,
    cell::UnsafeCell,
    marker::PhantomData,
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
    ptr::NonNull,
};

use cortex_m::peripheral::NVIC;

use crate::util::sync::volatile_cell::SyncVolatileU32;

/// Provides shared, mutually exclusive mutable access to an instance of T to a
/// set of ARMv7/8 Cortex-M execution contexts (Thread or Handler mode) running
/// _on a single core_ with up to seven priority bits.
///
/// Access permission to the resource is always exclusively owned by all
/// handlers executing at a single priority level as these handlers cannot
/// preempt each other. Handlers holding permission to access the resource may
/// pass it on to a different priority level.
///
/// This is a fine-grained, lock-free access strategy that works without
/// disabling interrupts: Clients have to prove at runtime that they are running
/// at the priority level that has been granted permission to access the
/// resource.
///
/// Conceptually this is not a mutex: Execution contexts are not able to acquire
/// access unless they have been granted access by some other handler before.
/// Once a handler decides to pass on access, it cannot acquire it back until
/// some other handler decides to return it. Therefore, the primitive works more
/// like an MPMC one-shot channel where the resource is actively "sent" from one
/// priority level to another.
///
/// The following rules apply:
///
/// - A handler that passes access permission on to a higher priority must
///   immediately give up access to the resource. This will be enforced at
///   compile time by an appropriate RAII guard.
///
/// - A handler passing access permission on to a lower priority retains
///   access until it yields. This is sound as such a handler cannot be
///   preempted by any lower-priority handler.
///
/// - Exclusive access is shared between equal-priority handlers even if they
///   correspond to distinct exception numbers: On a single core they cannot
///   preempt each other.
///
/// # Implementation
///
/// Execution contexts are identified by their exception number. Execution
/// priority is defined via their corresponding NVIC_IPR registers. Thread
/// handler priority is considered lower than any exception priority. System
/// exceptions are not currently supported to keep the code as tight and fast as
/// possible.
///
/// Co-ordination comes at the cost of runtime load and compare instructions to
/// assert priorities.
///
/// # Safety
///
/// This is a safe abstraction. Its safety relies on the fact that changing
/// priorities of execution contexts at runtime is considered unsafe in the
/// cortex-m crate (and otherwise requires unsafe low-level register access).
///
/// It is _not_ safe to change execution priority of the current handler while
/// it is executing. Actual priority will only change on the next invocation of
/// the handler but the priority cell will assume that the new priority is
/// already in effect.
///
/// Access is synchronized atomically, both by the compiler and the platform
/// (see safety notes of the priority field below).
pub struct PriorityCell<T> {
    /// Contains the managed value.
    ///
    /// You may wrap the value inside an option initialized to [`None`] to deal
    /// with values that don't have a const constructor or if you want the
    /// linker to place this part of the priority cell into .bss.
    value: UnsafeCell<T>,

    /// This field stores the ARM priority to which the resource is currently
    /// assigned: The lower the value, the higher the priority, only the leading
    /// priority bits are significant.
    ///
    /// Exception: A value of 0xFF represents thread priority (lowest priority).
    ///
    /// Note: We only support ARMv7 platforms with seven or less priority bits.
    ///
    /// # Optimization
    ///
    /// - Representing the priority in the same format as the processor allows
    ///   for faster comparison on the hot execution path. OTOH, this implies
    ///   that the initial value may often be non-zero and therefore land in the
    ///   .data segment. Rust will keep large zeroed initial values of T
    ///   separately in the .bss segment, though, which is good enough for our
    ///   purposes.
    ///
    /// - We store priority in a u32 which provably generates less code than
    ///   converting from/to u8.
    ///
    /// - We use a [`SyncVolatileU32`] rather than an atomic so the compiler
    ///   doesn't insert redundant memory barriers, see there.
    ///
    /// Impact of these measures has been confirmed and quantified with rustc
    /// v1.93 and the thumbv7em-none-eabi* target.
    ///
    /// # Safety
    ///
    /// We synchronize normal memory accesses (most notably those to the wrapped
    /// value) around priority updates as if it was an atomic:
    ///
    /// - We update the value only before/after Cortex-M context switches which
    ///   imply a memory barrier.
    /// - We use the most significant bit to mark the cell as borrowed. A
    ///   borrowed cell panics when trying to borrow it again.
    /// - Also see safety notes in [`SyncVolatileU32`].
    priority: SyncVolatileU32,
}

pub const THREAD_MODE_PRIORITY: u8 = 0xff;

impl<T> PriorityCell<T> {
    const BORROWED_TAG: u32 = 0x80000000u32;

    pub const fn new(value: T, initial_priority: u8) -> Self {
        Self {
            value: UnsafeCell::new(value),
            priority: SyncVolatileU32::new(initial_priority as u32),
        }
    }

    /// Consumes the access token and immediately releases access to a different
    /// priority level. Trying to access the value from the same context after
    /// this call will fail.
    ///
    /// This is the right method to call when releasing access to a
    /// higher-priority context or if the context does not have to access the
    /// value any more after releasing it.
    #[allow(unused)]
    #[inline]
    pub fn release(&self, cell_ref: PriorityCellRef<T>, to_priority: u8) {
        // Safety: From this point onwards a higher-priority interrupt will be
        //         able to preempt and acquire the resource. The drop guard must
        //         no longer run as it could interfere with a priority update of
        //         another context. Setting the priority will remove the borrow
        //         flag.
        let _ = ManuallyDrop::new(cell_ref);

        // Safety: The provided cell ref proves (and immediately ends) the
        //         permission to access.
        self.priority.set(to_priority as u32);
    }

    /// Releases access to a lower-priority context. The current handler will
    /// retain access to the value until it yields.
    ///
    /// If you do not need further access then calling
    /// [`PriorityCell::release()`] should be preferred as it does not require
    /// an additional runtime check.
    ///
    /// # Panics
    ///
    /// Panics if trying to release the resource to a higher-priority context.
    #[allow(unused)]
    #[inline]
    pub fn release_to_lower(&self, to_priority: u8) {
        let to_priority = to_priority as u32;
        let expected_priority = self.priority.get() & !Self::BORROWED_TAG;
        let current_priority = Self::current_priority();

        // Safety: We need to re-assert that the call is entitled to change the
        //         priority plus that the target priority is lower than the
        //         current priority.
        assert!(expected_priority == current_priority && to_priority > current_priority);

        // Safety: We proved that the target priority is lower than the current
        //         priority, so the current execution context retains permission
        //         to access the cell until we yield. The drop guard becomes a
        //         no-op from here on which does not compromise safety as
        //         re-borrowing won't succeed due to the updated priority. This
        //         clears the borrow flag but as the priority is also changed, a
        //         re-borrow is no longer possible after calling this method.
        self.priority.set(to_priority);
    }

    /// Provides access to the content of the cell assuming it is accessible by
    /// the current execution context.
    ///
    /// # Panics
    ///
    /// Panics if the current execution context does not have access to the
    /// resource.
    #[allow(unused)]
    #[inline]
    pub fn borrow_mut(&self) -> PriorityCellRef<'_, T> {
        assert!(self.try_borrow());

        // Safety:
        // - The underlying unsafe cell guarantees a non-null pointer.
        // - Checking the priority proved that we have exclusive access.
        let value_ptr = unsafe { NonNull::new_unchecked(self.value.get()) };
        PriorityCellRef {
            value_ptr,
            priority_ref: &self.priority,
            invariance: PhantomData,
        }
    }

    /// Provides access to the content of the cell if it is accessible by the
    /// current execution context. Otherwise returns [`None`].
    #[allow(unused)]
    #[inline]
    pub fn try_borrow_mut(&self) -> Option<PriorityCellRef<'_, T>> {
        if self.try_borrow() {
            // Safety:
            // - The underlying unsafe cell guarantees a non-null pointer.
            // - Checking the priority proved that we have exclusive access.
            let value_ptr = unsafe { NonNull::new_unchecked(self.value.get()) };
            Some(PriorityCellRef {
                value_ptr,
                priority_ref: &self.priority,
                invariance: PhantomData,
            })
        } else {
            None
        }
    }

    #[inline]
    fn try_borrow(&self) -> bool {
        let expected_priority = self.priority.get();
        if expected_priority == Self::current_priority() {
            self.priority.set(expected_priority | Self::BORROWED_TAG);
            true
        } else {
            false
        }
    }

    /// Determine the priority of the current execution context. The returned
    /// value conforms to ARM priority representation with one exception: 0xff
    /// represents thread context priority which is considered to be lower than
    /// any exception or interrupt priority.
    #[inline(always)]
    fn current_priority() -> u32 {
        // Note: We currently only support ARMv7/8.
        let exception_nr = Self::ipsr() as usize;
        match exception_nr {
            // Match cases are evaluated in order: hottest path first.
            16.. => {
                // Safety: The index is bounded to the number of available
                //         execution contexts by system design.
                unsafe { (*NVIC::PTR).ipr.get_unchecked(exception_nr - 16).read() as u32 }
            }
            0 => THREAD_MODE_PRIORITY as u32,
            _ => {
                // Other exception handlers are currently not supported.
                unreachable!()
            }
        }
    }

    /// Reads the Cortex-M core IPSR register which is not exposed by the
    /// cortex-m crate. This is faster than masking the VECTACTIVE field of the
    /// SCB's ICSR register.
    #[inline(always)]
    pub fn ipsr() -> u32 {
        let ipsr;
        unsafe {
            asm!("mrs {}, IPSR", out(reg) ipsr, options(nomem, nostack, preserves_flags));
        }
        ipsr
    }
}

// Safety: See the considerations in PriorityCell's documentation.
unsafe impl<T> Sync for PriorityCell<T> where T: Send {}

/// Represents exclusive access to the underlying resource by the current
/// execution context.
///
/// Note: This type is [!Send](`Send`).
pub struct PriorityCellRef<'value, T: 'value> {
    // We cannot use &'value mut T here as the guard only represents access to T
    // during its own lifetime, not that of 'value. Wrapping NonNull also
    // ensures that the type is !Send.
    value_ptr: NonNull<T>,

    // Reference to the priority, updated when the guard is dropped.
    priority_ref: &'value SyncVolatileU32,

    // The primitive represents interior mutability but NonNull is covariant by
    // default, therefore we need to re-establish invariance.
    invariance: PhantomData<&'value mut T>,
}

impl<T> Drop for PriorityCellRef<'_, T> {
    fn drop(&mut self) {
        let priority = self.priority_ref.get();
        self.priority_ref
            .set(priority & !PriorityCell::<T>::BORROWED_TAG);
    }
}

impl<T> Deref for PriorityCellRef<'_, T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        // Safety: The value is accessible for the lifetime of this wrapper.
        unsafe { self.value_ptr.as_ref() }
    }
}

impl<T> DerefMut for PriorityCellRef<'_, T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        // Safety: The value is accessible for the lifetime of this wrapper.
        unsafe { self.value_ptr.as_mut() }
    }
}
