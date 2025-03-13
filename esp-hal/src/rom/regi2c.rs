use crate::soc::regi2c::{regi2c_read, regi2c_write};

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
        regi2c_write(self.master, self.host_id, reg, data);
    }

    #[allow(unused)]
    pub fn read(&self, reg: u8) -> u8 {
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
        ::core::assert!(msb < 8 + lsb);
        Self { register, msb, lsb }
    }

    #[allow(unused)]
    pub fn read(&self) -> u8 {
        let reg_raw = self.register.read();
        (reg_raw >> self.lsb) & !(u8::MAX << (self.msb - self.lsb + 1))
    }

    #[allow(unused)]
    pub fn write_field(&self, data: u8) {
        let bits = self.register.read();

        let unwritten_bits = (!(u32::MAX << self.lsb) | (u32::MAX << (self.msb + 1))) as u8;
        let data_mask = !(u32::MAX << (self.msb - self.lsb + 1)) as u8;
        let data_bits = (data & data_mask) << self.lsb;

        self.register.write_reg((bits & unwritten_bits) | data_bits);
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
