pub use self::soc::*;

#[cfg_attr(esp32, path = "esp32/mod.rs")]
#[cfg_attr(esp32c2, path = "esp32c2/mod.rs")]
#[cfg_attr(esp32c3, path = "esp32c3/mod.rs")]
#[cfg_attr(esp32c6, path = "esp32c6/mod.rs")]
#[cfg_attr(esp32h2, path = "esp32h2/mod.rs")]
#[cfg_attr(esp32s2, path = "esp32s2/mod.rs")]
#[cfg_attr(esp32s3, path = "esp32s3/mod.rs")]
mod soc;

mod efuse_field;

pub fn is_valid_ram_address(address: u32) -> bool {
    if (soc::constants::SOC_DRAM_LOW..=soc::constants::SOC_DRAM_HIGH).contains(&address) {
        true
    } else {
        false
    }
}
