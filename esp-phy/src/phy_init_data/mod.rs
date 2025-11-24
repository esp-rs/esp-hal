#[cfg_attr(esp32c3, path = "phy_init_data_esp32c3.rs")]
#[cfg_attr(esp32c2, path = "phy_init_data_esp32c2.rs")]
#[cfg_attr(esp32c6, path = "phy_init_data_esp32c6.rs")]
#[cfg_attr(esp32h2, path = "phy_init_data_esp32h2.rs")]
#[cfg_attr(esp32, path = "phy_init_data_esp32.rs")]
#[cfg_attr(esp32s3, path = "phy_init_data_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "phy_init_data_esp32s2.rs")]
mod chip_phy_init_data;
pub(crate) use chip_phy_init_data::PHY_INIT_DATA_DEFAULT;
