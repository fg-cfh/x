use core::{
    cell::UnsafeCell,
    sync::atomic::{Ordering, compiler_fence},
};

/// Like [`core::cell::Cell`] but with volatile read/write semantics.
#[repr(transparent)]
pub struct VolatileCell<T> {
    value: UnsafeCell<T>,
}

impl<T> VolatileCell<T> {
    pub const fn new(value: T) -> Self {
        VolatileCell {
            value: UnsafeCell::new(value),
        }
    }

    #[inline]
    pub fn get(&self) -> T
    where
        T: Copy,
    {
        // Safety: UnsafeCell guarantees a dereferenceable pointer.
        unsafe { self.value.get().read_volatile() }
    }

    #[inline]
    pub fn set(&self, value: T)
    where
        T: Copy,
    {
        // Safety: UnsafeCell guarantees a dereferenceable pointer.
        unsafe { self.value.get().write_volatile(value) }
    }
}

/// Behaves like an atomic type with release/acquire semantics on a single
/// ARMv7/8-M core. Currently not supported in other environments.
///
/// # Optimization
///
/// On a single core, atomic access provably introduces redundant memory
/// barriers and blocks certain opportunities for optimization. Volatile u32
/// access OTOH reliably reduces code size. This primitive therefore uses
/// compiler fences + volatile access to synchronize memory access.
///
/// # Safety
///
/// - On Cortex-M volatile aligned access to a u32 is guaranteed not to tear.
/// - To let the compiler synchronize preceding/following normal memory
///   accesses, we use "standalone" release/acquire compiler fences. This
///   "off-label" usage of compiler fences happens to work and is commonly
///   recommended practice in the embedded ecosystem (usually for DMA, but the
///   same principle applies here).
/// - This approach is future-proof: Pairing compiler fences with volatile
///   access will be officially supported once volatile read/write becomes
///   atomic, also see <https://github.com/rust-lang/unsafe-code-guidelines/issues/321#issuecomment-2894697770>.
#[repr(transparent)]
pub struct SyncVolatileU32 {
    value: VolatileCell<u32>,
}

impl SyncVolatileU32 {
    pub const fn new(value: u32) -> Self {
        Self {
            value: VolatileCell::new(value),
        }
    }

    #[inline]
    pub fn get(&self) -> u32 {
        let value = self.value.get();
        compiler_fence(Ordering::Acquire);
        value
    }

    #[inline]
    pub fn set(&self, value: u32) {
        compiler_fence(Ordering::Release);
        self.value.set(value);
    }
}

// Safety: See safety notes of SyncVolatileU32.
unsafe impl Sync for SyncVolatileU32 {}
