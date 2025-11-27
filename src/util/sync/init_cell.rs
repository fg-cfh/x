#[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
use core::sync::atomic::AtomicUsize;
use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    ptr::NonNull,
    sync::atomic::Ordering,
};

/// Initialization state.
enum InitializationState {
    /// Not yet initialized.
    Uninitialized,
    /// Currently initializing.
    Initializing,
    /// Already initialized.
    Initialized,
}

/// Static-friendly wrapper for an object that requires initialization at
/// program start.
///
/// This should only be used, where initialization happens once at the very
/// start of the program so that for all practical purposes it can be considered
/// "always initialized". In all other cases the type may behave unexpectedly as
/// dereferencing it may panic.
///
/// Inherits [`Send`] and [`Sync`] from T.
///
/// # Panics
///
/// Panics when dropped or dereferenced before initialization. When the
/// 'optimize' feature is enabled, then dereferencing before initialization
/// triggers UB instead.
pub struct InitCell<T> {
    /// Initialization state, see [`InitializationState`].
    #[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
    init_state: AtomicUsize,

    /// The inner object that requires initialization.
    inner: UnsafeCell<T>,
}

// Safety: Access to UnsafeCell is mediated via an atomic.
unsafe impl<T: Send> Send for InitCell<T> {}

// Safety: Access to UnsafeCell is mediated via an atomic.
unsafe impl<T: Sync> Sync for InitCell<T> {}

impl<T> InitCell<T> {
    pub const fn new(value: T) -> Self {
        Self {
            #[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
            init_state: AtomicUsize::new(InitializationState::Uninitialized as usize),
            inner: UnsafeCell::new(value),
        }
    }

    #[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
    pub fn is_initialized(&self) -> bool {
        self.init_state.load(Ordering::Acquire) == InitializationState::Initialized as usize
    }

    /// Initializes the wrapped value and makes it available as a smart pointer.
    ///
    /// Must be called before dereferencing the cell for the first time.
    pub fn init(&self, init: impl FnOnce(&mut T)) {
        // Safety: Atomically acquires the inner value for globally exclusive
        //         mutable access.
        #[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
        self.init_state
            .compare_exchange(
                InitializationState::Uninitialized as usize,
                InitializationState::Initializing as usize,
                Ordering::AcqRel,
                Ordering::Relaxed,
            )
            .unwrap();

        // Safety:
        // - UnsafeCell guarantees a non-null pointer.
        // - We acquired the inner value.
        let inner = unsafe { NonNull::new_unchecked(self.inner.get()).as_mut() };
        init(inner);

        // Safety: Atomically releases the inner value in a state that allows
        //         normal access by the program, both shared and mutable.
        #[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
        self.init_state
            .store(InitializationState::Initialized as usize, Ordering::Release);
    }
}

impl<T> Deref for InitCell<T> {
    type Target = T;

    #[inline]
    fn deref(&self) -> &T {
        // Safety: This will panic on program error only.
        #[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
        assert!(self.is_initialized());

        // Safety:
        // - UnsafeCell guarantees a non-null pointer.
        // - A shared reference to self proves shared read-only access to the cell.
        unsafe { NonNull::new_unchecked(self.inner.get()).as_ref() }
    }
}

impl<T> DerefMut for InitCell<T> {
    #[inline]
    fn deref_mut(&mut self) -> &mut T {
        // Safety: This will panic on program error only.
        #[cfg(any(not(feature = "unsafe-optimize"), debug_assertions))]
        assert!(self.is_initialized());

        self.inner.get_mut()
    }
}
