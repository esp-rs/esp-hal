//! # Inter-Integrated Circuit (I2C)
//!
//! I2C is a serial, synchronous, multi-device, half-duplex communication
//! protocol that allows co-existence of multiple masters and slaves on the
//! same bus. I2C uses two bidirectional open-drain lines: serial data line
//! (SDA) and serial clock line (SCL), pulled up by resistors.
//!
//! For more information, see
#![doc = crate::trm_markdown_link!("i2c")]

pub mod master;

#[cfg(lp_i2c0)]
crate::unstable_module! {
    pub mod lp_i2c;
}

crate::any_peripheral! {
    /// Any I2C peripheral.
    pub peripheral AnyI2c<'d> {
        #[cfg(i2c0)]
        I2c0(crate::peripherals::I2C0<'d>),
        #[cfg(i2c1)]
        I2c1(crate::peripherals::I2C1<'d>),
    }
}
