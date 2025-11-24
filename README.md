//! Executor:
//! - multiple thread-level tasks
//! - one task per interrupt
//! - all in privileged mode
//!
//! Resources:
//! - task-local
//! - shared between named tasks
//! - shared by priority level
//! - single CPU:
//! - tasks that run at the same priority level can share resources w/o
//! locking
//! - tasks running at the highest priority can use resources w/o locking
//! - tree structure
//! - granularity needs to be managed at any level
//! - access should be mediated by zero-sized pointers
//! - access to state must not escape its domain (via aliasing)
//!
//! Messages:
//! - buffers,
//! - mac/drv requests/responses,
//! - timed signals, events
// Initialization:
// - everything
// - Thread: write
// - Interrupt: no
// - mediated by init state flag

// Init state:
// - level 0
// - Thread: write
// - Interrupt: read
// -> atomic

// Half period:
// - level 0
// - Thread: read
// - Interrupt: write
// -> atomic

// Alarms:
// - level 1
// - managed independently
// - mediated by alarm state flag
// - all: write (owned), no (otherwise)

// Timer epoch:
// - level 0
// - mediated by alarm state flag
// - all: write (owned), read (shared)

// Others: read only

// Waiting for async rtc timer:
// - state in atomic ownership flag and compare register
// - de-activates timer interrupt, clears state and changes atomic flag when
// dropped

// Waiting for async hp timer:
// - state in compare register
// - de-activates timer interrupt when dropped
