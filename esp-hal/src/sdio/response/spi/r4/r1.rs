use bitfielder::bitfield;

bitfield! {
    /// Represents the Modified R1 response type used in SPI mode R4 response.
    pub ModifiedR1(u8): u8,
    mask: 0x5d,
    default: 0x00,
    {
        /// Represents whether a command's argument was invalid.
        pub parameter_error: 6;
        /// Represents whether there was an error in command function number.
        pub function_number_error: 4;
        /// Represents whether the COM CRC check of the last command failed.
        pub crc_error: 3;
        /// Represents whether an illegal command was received.
        pub illegal_command: 2;
        /// Represents whether the card is in the idle state.
        pub idle: 0;
    }
}

impl ModifiedR1 {
    /// Gets whether an error condition is set.
    pub const fn is_err(&self) -> bool {
        self.parameter_error()
            || self.function_number_error()
            || self.crc_error()
            || self.illegal_command()
    }
}

impl core::fmt::Display for ModifiedR1 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "R1 {{")?;
        write!(f, " parameter_error: {},", self.parameter_error())?;
        write!(
            f,
            " function_number_error: {},",
            self.function_number_error()
        )?;
        write!(f, " crc_error: {},", self.crc_error())?;
        write!(f, " illegal_command: {},", self.illegal_command())?;
        write!(f, " idle: {}", self.idle())?;
        write!(f, " }}")
    }
}
