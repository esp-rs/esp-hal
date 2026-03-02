#[cfg_attr(esp32, path = "phy_clocks_ll_esp32.rs")]
#[cfg_attr(esp32c2, path = "phy_clocks_ll_esp32c2.rs")]
#[cfg_attr(esp32c3, path = "phy_clocks_ll_esp32c3.rs")]
#[cfg_attr(esp32c5, path = "phy_clocks_ll_esp32c5.rs")]
#[cfg_attr(esp32c6, path = "phy_clocks_ll_esp32c6.rs")]
#[cfg_attr(esp32h2, path = "phy_clocks_ll_esp32h2.rs")]
#[cfg_attr(esp32s2, path = "phy_clocks_ll_esp32s2.rs")]
#[cfg_attr(esp32s3, path = "phy_clocks_ll_esp32s3.rs")]
mod phy_clocks_ll;

pub(crate) fn enable_phy(enable: bool) {
    phy_clocks_ll::enable_phy(enable);
}
