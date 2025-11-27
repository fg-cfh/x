use crate::util::sync::priority_cell::THREAD_MODE_PRIORITY;

const NUM_PRIO_BITS: u32 = 3;
const NVIC_PRIO_SHIFT: u32 = u8::BITS - NUM_PRIO_BITS;
const NVIC_PRIO_LOWEST: u8 = u8::MAX << NVIC_PRIO_SHIFT;

// Logical priorities
pub const THREAD_PRIO: u8 = 0;
pub const LOWEST_HANDLER_PRIO: u8 = 1;
pub const HIGHEST_HANDLER_PRIO: u8 = 2u8.pow(NUM_PRIO_BITS);

/// Converts the given logical priority to the corresponding ARM priority (as
/// represented in [`PriorityCell::priority`]).
#[inline]
pub const fn arm_system_priority(logical_priority: u8) -> u8 {
    assert!(logical_priority < HIGHEST_HANDLER_PRIO);

    if logical_priority == THREAD_PRIO {
        // Thread priority.
        THREAD_MODE_PRIORITY
    } else {
        let offset = logical_priority - 1;
        NVIC_PRIO_LOWEST - (offset << NVIC_PRIO_SHIFT)
    }
}
