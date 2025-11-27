pub enum Task<IntermediateOutput, FinalOutput> {
    Continue(IntermediateOutput),
    Completed(FinalOutput),
}

pub enum Interrupt {
    Continue,
    Completed,
}
