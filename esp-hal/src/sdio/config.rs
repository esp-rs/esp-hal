use super::Timing;

/// Represents SDIO configuration parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    hs: bool,
    timing: Timing,
}

impl Config {
    /// Creates a new [Config].
    pub const fn new() -> Self {
        Self {
            hs: false,
            timing: Timing::new(),
        }
    }

    /// Gets the highspeed enable setting.
    pub const fn hs(&self) -> bool {
        self.hs
    }

    /// Sets the highspeed enable setting.
    pub fn set_hs(&mut self, hs: bool) {
        self.hs = hs;
    }

    /// Builder funciton that sets the highspeed enable setting.
    pub fn with_hs(self, hs: bool) -> Self {
        Self { hs, ..self }
    }

    /// Gets the timing setting.
    pub const fn timing(&self) -> Timing {
        self.timing
    }

    /// Sets the timing setting.
    pub fn set_timing(&mut self, timing: Timing) {
        self.timing = timing;
    }

    /// Builder funciton that sets the timing setting.
    pub fn with_timing(self, timing: Timing) -> Self {
        Self { timing, ..self }
    }
}

impl Default for Config {
    fn default() -> Self {
        Self::new()
    }
}
