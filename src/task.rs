pub enum StepResult<Task, Result> {
    Continue(Task),
    Completed(Result),
}

pub trait Task<Event, Result>: Sized {
    fn step(self, event: Event) -> StepResult<Self, Result>;
}

pub trait OngoingTask<Event>: Sized {
    fn step(self, event: Event) -> Self;
}
