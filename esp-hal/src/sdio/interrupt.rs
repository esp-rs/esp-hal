use bitfield::bitfield;

bitfield! {
    /// Represents an interrupt to send to the host.
    ///
    /// # Note
    ///
    /// Values derived from [esp-idf](https://github.com/espressif/esp-idf/blob/v5.4.1/components/hal/include/hal/sdio_slave_types.h) SDIO driver.
    #[repr(C)]
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct HostInterrupt(u8);
    /// General purpose host interrupt: 0.
    pub general0, set_general0: 0;
    /// General purpose host interrupt: 1.
    pub general1, set_general1: 1;
    /// General purpose host interrupt: 2.
    pub general2, set_general2: 2;
    /// General purpose host interrupt: 3.
    pub general3, set_general3: 3;
    /// General purpose host interrupt: 4.
    pub general4, set_general4: 4;
    /// General purpose host interrupt: 5.
    pub general5, set_general5: 5;
    /// General purpose host interrupt: 6.
    pub general6, set_general6: 6;
    /// General purpose host interrupt: 7.
    pub general7, set_general7: 7;
}

impl HostInterrupt {
    /// Creates a new [HostInterrupt].
    pub const fn new() -> Self {
        Self(0)
    }

    /// Gets the raw bit value of the [HostInterrupt].
    pub const fn bits(&self) -> u8 {
        self.0
    }
}

impl From<HostInterrupt> for u8 {
    fn from(val: HostInterrupt) -> Self {
        val.bits()
    }
}

impl From<u8> for HostInterrupt {
    fn from(val: u8) -> Self {
        Self(val)
    }
}

impl Default for HostInterrupt {
    fn default() -> Self {
        Self::new()
    }
}

bitfield! {
    /// Represents an interrupt to sent from the host.
    ///
    /// # Note
    ///
    /// Values derived from [esp-idf](https://github.com/espressif/esp-idf/blob/v5.4.1/components/hal/esp32/include/hal/sdio_slave_ll.h) SDIO driver.
    #[repr(C)]
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct DeviceInterrupt(u8);
    /// General purpose host interrupt: 0.
    pub general0, set_general0: 0;
    /// General purpose host interrupt: 1.
    pub general1, set_general1: 1;
    /// General purpose host interrupt: 2.
    pub general2, set_general2: 2;
    /// General purpose host interrupt: 3.
    pub general3, set_general3: 3;
    /// General purpose host interrupt: 4.
    pub general4, set_general4: 4;
    /// General purpose host interrupt: 5.
    pub general5, set_general5: 5;
    /// General purpose host interrupt: 6.
    pub general6, set_general6: 6;
    /// General purpose host interrupt: 7.
    pub general7, set_general7: 7;
}

impl DeviceInterrupt {
    /// Creates a new [DeviceInterrupt].
    pub const fn new() -> Self {
        Self(0)
    }

    /// Gets the raw bit value of the [DeviceInterrupt].
    pub const fn bits(&self) -> u8 {
        self.0
    }
}

impl From<DeviceInterrupt> for u8 {
    fn from(val: DeviceInterrupt) -> Self {
        val.bits()
    }
}

impl From<u8> for DeviceInterrupt {
    fn from(val: u8) -> Self {
        Self(val)
    }
}

impl Default for DeviceInterrupt {
    fn default() -> Self {
        Self::new()
    }
}
