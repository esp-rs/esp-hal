/// Represents SDIO configuration parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    /// Indicates support for high-speed mode.
    pub hs: bool,
}

impl Config {
    /// Creates a new [Config].
    pub const fn new() -> Self {
        Self { hs: false }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}
