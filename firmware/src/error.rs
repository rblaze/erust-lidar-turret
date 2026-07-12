use core::convert::Infallible;

use thiserror::Error;

#[derive(Debug, PartialEq, Eq, Clone, Copy, Error)]
pub enum Error {
    #[error("peripherals already initialized")]
    AlreadyTaken,
    #[error(transparent)]
    Mailbox(#[from] async_scheduler::mailbox::Error),
    #[error("Device busy")]
    DeviceBusy,
    #[error("Buffer overrun")]
    BufferOverrun,
}

impl From<Infallible> for Error {
    fn from(_: Infallible) -> Self {
        unreachable!()
    }
}
