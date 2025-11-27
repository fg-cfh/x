use core::{
    arch::asm,
    marker::PhantomData,
    ops::{Add, Sub},
};

use cortex_m::peripheral::NVIC;
use paste::paste;

/// A trait representing the number of priority bits on a given ARMv7/8
/// platform.
///
/// Values between two and eight are allowed in principle, see
/// <https://developer.arm.com/documentation/107706/0100/Exceptions-and-interrupts-overview/Exception-priority-level-definitions>.
///
/// We currently support up to seven priority bits only.
pub trait PriorityBits: Copy {
    const PRIORITY_BITS: usize;
}

macro_rules! impl_execution_priorities {
    ($num_priority_bits:literal) => {
        paste! {
            #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
            pub struct [<PB$num_priority_bits>];
            impl PriorityBits for [<PB$num_priority_bits>] {
                const PRIORITY_BITS: usize = $num_priority_bits;
            }

            impl Add<u8> for ExecutionPriority<[<PB$num_priority_bits>]> {
                type Output = Option<Self>;

                fn add(self, rhs: u8) -> Self::Output {
                    self.const_add(rhs)
                }
            }

            impl Sub<u8> for ExecutionPriority<[<PB$num_priority_bits>]> {
                type Output = Option<Self>;

                fn sub(self, rhs: u8) -> Self::Output {
                    self.const_sub(rhs)
                }
            }
        }
    };
}

impl_execution_priorities!(2);
impl_execution_priorities!(3);
impl_execution_priorities!(4);
impl_execution_priorities!(5);
impl_execution_priorities!(6);
impl_execution_priorities!(7);

/// Represent logical ARMv7/8 execution priorities.
///
/// The lowest logical execution priority (priority: 0) is thread priority.
///
/// The lowest interrupt execution priority (priority: 1) is higher than thread
/// priority.
///
/// The highest interrupt execution priority depends on the number of available
/// priority bits (priority: 2^num_priority_bits). It is platform and product
/// specific.
///
/// Logical priorities of system exceptions could be modeled above interrupt
/// priorities. But we currently don't need them.
///
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ExecutionPriority<PB: PriorityBits>(u8, PhantomData<PB>);

const THREAD_MODE_SENTINEL: u8 = 0xff;

impl<PB: PriorityBits> ExecutionPriority<PB> {
    const _CHECK_PRIORITY_BITS: () = {
        assert!(PB::PRIORITY_BITS <= 8);
    };

    const HIGHEST_U8: u8 = 2u8.pow(PB::PRIORITY_BITS as u32);
    pub const HIGHEST: Self = Self(Self::HIGHEST_U8, PhantomData);

    const LOWEST_U8: u8 = 0;
    pub const LOWEST: Self = Self(Self::LOWEST_U8, PhantomData);

    pub const THREAD: Self = Self::LOWEST;
    pub const LOWEST_INTERRUPT: Self = Self(Self::LOWEST_U8 + 1, PhantomData);
    pub const HIGHEST_INTERRUPT: Self = Self::HIGHEST;

    /// Convert the logical execution priority to the corresponding left-aligned
    /// ARM NVIC execution priority representation. Thread priority (lowest) is
    /// represented by a special value (0xff).
    #[inline]
    pub const fn to_arm_nvic_repr(self) -> u8 {
        // Need to compare primitives for const compat.
        if self.0 == ExecutionPriority::<PB>::THREAD.0 {
            THREAD_MODE_SENTINEL
        } else {
            // NVIC priorities are left aligned.
            (Self::HIGHEST_U8 - self.0) << (8 - PB::PRIORITY_BITS)
        }
    }

    /// Convert the left-aligned ARM NVIC execution priority representation to
    /// the internal representation.
    #[inline]
    pub const fn from_arm_nvic_repr(priority: u8) -> Self {
        if priority == THREAD_MODE_SENTINEL {
            Self::THREAD
        } else {
            // NVIC priorities are left aligned. Less significant bits are ignored.
            Self(
                Self::HIGHEST_U8 - (priority >> (8 - PB::PRIORITY_BITS)),
                PhantomData,
            )
        }
    }

    pub fn current() -> Self {
        Self::from_arm_nvic_repr(current_arm_nvic_priority() as u8)
    }

    /// Const conversion from u8.
    pub const fn try_from_logical_priority(logical_priority: u8) -> Result<Self, ()> {
        if logical_priority > Self::HIGHEST_U8 {
            Err(())
        } else {
            Ok(Self(logical_priority, PhantomData))
        }
    }

    /// Const conversion to u8.
    pub const fn as_logical_priority(&self) -> u8 {
        self.0
    }

    pub const fn const_add(self, rhs: u8) -> Option<Self> {
        match Self::try_from_logical_priority(self.0 + rhs) {
            Ok(result) => Some(result),
            Err(_) => None,
        }
    }

    pub const fn const_sub(self, rhs: u8) -> Option<Self> {
        if rhs > self.0 {
            return None;
        }

        Some(Self(self.0 - rhs, PhantomData))
    }

    pub const fn plus_one(self) -> Self {
        match Self::try_from_logical_priority(self.0 + 1) {
            Ok(next_higher) => next_higher,
            Err(_) => panic!(),
        }
    }

    pub const fn minus_one(self) -> Self {
        assert!(self.0 > 0);
        Self(self.0 - 1, PhantomData)
    }
}

impl<PB: PriorityBits> TryFrom<u8> for ExecutionPriority<PB> {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::try_from_logical_priority(value)
    }
}

impl<PB: PriorityBits> From<ExecutionPriority<PB>> for u8 {
    fn from(value: ExecutionPriority<PB>) -> Self {
        value.as_logical_priority()
    }
}

/// Determine the priority of the current execution context. The returned
/// value conforms to ARM priority representation with one exception: 0xff
/// represents thread context priority which is considered to be lower than
/// any exception or interrupt priority.
///
/// Returning u32 rather then u8 is an optimization for
/// [`super::priority_cell::PriorityCell`] and should not be changed w/o
/// measuring performance impact.
#[inline]
pub fn current_arm_nvic_priority() -> u32 {
    // Note: We currently only support ARMv7/8.
    let exception_nr = ipsr() as usize;
    match exception_nr {
        // Match cases are evaluated in order: hottest path first.
        16.. => {
            // Safety: The index is bounded to the number of available
            //         execution contexts by system design.
            unsafe { (*NVIC::PTR).ipr.get_unchecked(exception_nr - 16).read() as u32 }
        }
        0 => THREAD_MODE_SENTINEL as u32,
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
