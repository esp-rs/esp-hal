//! Useful wrappers

pub mod client;
pub mod server;

/// Errors returned by the adapters
#[derive(Debug)]
pub enum ConnectionError<E: embedded_io::Error> {
    /// Error from embedded-io
    Io(E),
    /// Error from Rustls
    Rustls(rustls::Error),
    /// Error from Rustls' `encode`function
    RustlsEncodeError(rustls::unbuffered::EncodeError),
}

impl<E> embedded_io::Error for ConnectionError<E>
where
    E: embedded_io::Error,
{
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            ConnectionError::Io(err) => err.kind(),
            _ => embedded_io::ErrorKind::Other,
        }
    }
}

impl<E> From<E> for ConnectionError<E>
where
    E: embedded_io::Error,
{
    fn from(value: E) -> Self {
        Self::Io(value)
    }
}
