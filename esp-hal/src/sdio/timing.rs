/// Represents SDIO device timing settings.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Timing {
    /// Send at posedge, and sample at posedge. Default for HS.
    PsendPsample = 0,
    /// Send at negedge, and sample at posedge. Default for DS.
    NsendPsample,
    /// Send at posedge, and sample at negedge.
    PsendNsample,
    /// Send at negedge, and sample at negedge.
    NsendNsample,
}

impl Timing {
    /// Creates a new [Timing].
    pub const fn new() -> Self {
        Self::PsendPsample
    }
}

impl Default for Timing {
    fn default() -> Self {
        Self::new()
    }
}
