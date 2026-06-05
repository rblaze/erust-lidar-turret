use thiserror::Error;

#[derive(Debug, PartialEq, Eq, Clone, Copy, Error)]
pub enum Error {
    #[error("peripherals already initialized")]
    AlreadyTaken,
}
