use esp_sync::CountingLock;

use crate::soc::regi2c::{regi2c_read, regi2c_write};

/// Serializes accesses to the analog I2C master, which esp-hal shares with the PHY library.
pub(crate) static LOCK: CountingLock = CountingLock::new();

/// Locks the analog I2C master, for callers outside esp-hal that access it.
///
/// Calls nest, and each one must be paired with a [`regi2c_exit_critical`] on the same core.
#[cfg(feature = "rt")]
#[crate::ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn regi2c_enter_critical() {
    unsafe { LOCK.acquire() };
}

/// Unlocks the analog I2C master locked by [`regi2c_enter_critical`].
#[cfg(feature = "rt")]
#[crate::ram]
#[unsafe(no_mangle)]
unsafe extern "C" fn regi2c_exit_critical() {
    unsafe { LOCK.release() };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct RegI2cMaster {
    pub master: u8,
    pub host_id: u8,
}

impl RegI2cMaster {
    pub const fn new(master: u8, host_id: u8) -> Self {
        Self { master, host_id }
    }

    #[allow(unused)]
    pub fn write(&self, reg: u8, data: u8) {
        LOCK.lock(|| self.write_unlocked(reg, data));
    }

    #[allow(unused)]
    pub fn read(&self, reg: u8) -> u8 {
        LOCK.lock(|| self.read_unlocked(reg))
    }

    // The ESP32-S31 light sleep sets the MPLL with the caches suspended, through the RAM `read` and
    // `write` in `soc::esp32s31::regi2c`. The `*_unlocked` accessors must then be inlined into the
    // RAM caller. On other chips, the accessors end in flash or ROM code anyway, and inlining only
    // copies them into each call site.

    /// Writes `reg` without taking the lock.
    ///
    /// The caller holds the lock, or runs where nothing else can access the analog I2C master.
    #[allow(unused)]
    #[cfg_attr(all(esp32s31, not(is_debug_build)), inline(always))]
    pub fn write_unlocked(&self, reg: u8, data: u8) {
        regi2c_write(self.master, self.host_id, reg, data);
    }

    /// Reads `reg` without taking the lock. See [`Self::write_unlocked`].
    #[allow(unused)]
    #[cfg_attr(all(esp32s31, not(is_debug_build)), inline(always))]
    pub fn read_unlocked(&self, reg: u8) -> u8 {
        regi2c_read(self.master, self.host_id, reg)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct RegI2cRegister {
    master: RegI2cMaster,
    reg_add: u8,
}

impl RegI2cRegister {
    pub const fn new(master: RegI2cMaster, reg_add: u8) -> Self {
        Self { master, reg_add }
    }

    #[allow(unused)]
    pub fn write_reg(&self, data: u8) {
        self.master.write(self.reg_add, data);
    }

    #[allow(unused)]
    pub fn read(&self) -> u8 {
        self.master.read(self.reg_add)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct RawRegI2cField {
    register: RegI2cRegister,
    msb: u8,
    lsb: u8,
}

impl RawRegI2cField {
    #[allow(unused)]
    pub const fn new(register: RegI2cRegister, msb: u8, lsb: u8) -> Self {
        ::core::assert!(lsb <= msb && msb < 8);
        Self { register, msb, lsb }
    }

    #[cfg_attr(all(esp32s31, not(is_debug_build)), inline(always))]
    const fn mask(&self) -> u8 {
        const fn lower_n_bits(bits: u8) -> u8 {
            !((u32::MAX << bits) as u8)
        }

        lower_n_bits(self.msb - self.lsb + 1)
    }

    #[allow(unused)]
    pub fn read(&self) -> u8 {
        (self.register.read() >> self.lsb) & self.mask()
    }

    #[allow(unused)]
    pub fn write_field(&self, data: u8) {
        LOCK.lock(|| self.write_field_unlocked(data));
    }

    /// Writes the field without taking the lock. See [`RegI2cMaster::write_unlocked`].
    #[allow(unused)]
    #[cfg_attr(all(esp32s31, not(is_debug_build)), inline(always))]
    pub fn write_field_unlocked(&self, data: u8) {
        let master = self.register.master;
        let reg = self.register.reg_add;
        let mask = self.mask() << self.lsb;
        let bits = master.read_unlocked(reg);
        master.write_unlocked(reg, (bits & !mask) | ((data << self.lsb) & mask));
    }
}

macro_rules! define_regi2c {
    ($(master: $master_name:ident($master:literal, $hostid:literal) {
        $(reg: $register_name:ident($register:literal) {
            $(field: $field_name:ident ( $msb:literal .. $lsb:literal )),*
        })*
    })+) => {
        $(
            #[allow(unused)]
            pub(crate) const $master_name: RegI2cMaster = RegI2cMaster::new($master, $hostid);

            $(
                #[allow(unused)]
                pub(crate) const $register_name: RegI2cRegister = RegI2cRegister::new($master_name, $register);

                $(
                    #[allow(unused)]
                    pub(crate) const $field_name: RawRegI2cField = RawRegI2cField::new($register_name, $msb, $lsb);
                )*
            )*
        )+
    };
}
pub(crate) use define_regi2c;
