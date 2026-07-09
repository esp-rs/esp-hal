//! # Inter-Integrated Circuit (I2C)
//!
//! I2C is a serial, synchronous, multi-device, half-duplex communication
//! protocol that allows co-existence of multiple masters and slaves on the
//! same bus. I2C uses two bidirectional open-drain lines: serial data line
//! (SDA) and serial clock line (SCL), pulled up by resistors.
//!
//! For more information, see
#![doc = crate::trm_markdown_link!("i2c")]

#[cfg(i2c_master_driver_supported)]
pub mod master;

#[cfg(i2c_slave_driver_supported)]
crate::unstable_module! {
    pub mod slave;
}

#[cfg(soc_has_lp_i2c0)]
crate::unstable_module! {
    pub mod lp_i2c;
}

#[cfg(esp32s3)] // Only support ESP32-S3 for now.
#[cfg(soc_has_rtc_i2c)]
crate::unstable_module! {
    pub mod rtc;
}

#[cfg_attr(i2c_master_version = "1", path = "clocks/v1.rs")]
#[cfg_attr(i2c_master_version = "2", path = "clocks/v2.rs")]
#[cfg_attr(
    all(i2c_master_version = "3", not(any(esp32p4, soc_has_pcr))),
    path = "clocks/v3.rs"
)]
#[cfg_attr(esp32p4, path = "clocks/esp32p4.rs")]
#[cfg_attr(soc_has_pcr, path = "clocks/v3_pcr.rs")]
mod clocks;
