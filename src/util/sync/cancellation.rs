use core::mem;

/// Objects with this marker trait promise to cancel an ongoing operation when
/// dropped.
#[must_use = "Cancels the operation if dropped."]
pub trait Cancelable: Sized {}
