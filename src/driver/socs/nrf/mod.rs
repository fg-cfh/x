pub mod clock;
pub mod radio;
pub mod timer;

use crate::util::sync::priority::{ExecutionPriority, PB3};

pub type NrfPrioBits = PB3;
pub type NrfExcPrio = ExecutionPriority<NrfPrioBits>;
