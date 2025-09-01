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

#[cfg(soc_has_lp_i2c0)]
crate::unstable_module! {
    pub mod lp_i2c;
}

#[cfg(esp32s3)] // Only support ESP32-S3 for now.
#[cfg(soc_has_rtc_i2c)]
crate::unstable_module! {
    pub mod rtc;
}
