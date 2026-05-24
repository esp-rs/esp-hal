//! MIPI DSI command-mode (DBI) interface.

use crate::mipi_dsi::MipiDsi;

/// Command-mode (DBI) handle.
///
/// Holds a mutable borrow of the [`MipiDsi`] bus so that command and video
/// modes cannot coexist without explicit sequencing.
pub struct DsiDbi<'bus, 'd> {
    _bus: &'bus mut MipiDsi<'d>,
    virtual_channel: u8,
}

impl<'bus, 'd> DsiDbi<'bus, 'd> {
    pub(crate) fn new(bus: &'bus mut MipiDsi<'d>, virtual_channel: u8) -> Self {
        Self { _bus: bus, virtual_channel }
    }

    /// Returns the virtual channel ID this interface was configured with.
    pub fn virtual_channel(&self) -> u8 {
        self.virtual_channel
    }
}
