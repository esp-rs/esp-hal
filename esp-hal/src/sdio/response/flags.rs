use bitfielder::bitfield;

use super::IoCurrentState;
use crate::sdio::Error;

bitfield! {
    /// Represents the SDIO R5 response flags.
    pub Flags(u8): u8,
    mask: 0xfb,
    default: 0,
    {
        /// Indicates a CRC error in the previous command.
        pub crc_error: 7;
        /// Indicates an illegal command.
        pub illegal_command: 6;
        raw_io_state: 5, 4;
        /// Indicates a general error.
        pub error: 3;
        /// Indicates an invalid function number.
        pub function_number: 1;
        /// Indicates the command's argument was out-of-range.
        pub out_of_range: 0;
    }
}

impl Flags {
    /// Gets the SDIO card current state.
    pub const fn io_current_state(&self) -> Result<IoCurrentState, Error> {
        IoCurrentState::try_from_inner(self.raw_io_state())
    }

    /// Sets the SDIO card current state.
    pub fn set_io_current_state(&mut self, val: IoCurrentState) {
        self.set_raw_io_state(val.into_inner());
    }

    /// Attempts to convert an inner value into a [Flags].
    pub const fn try_from_inner(val: u8) -> Result<Self, Error> {
        match Self(val) {
            f if f.io_current_state().is_err() => Err(Error::General),
            f => Ok(f),
        }
    }
}
