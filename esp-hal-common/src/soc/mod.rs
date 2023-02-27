#[cfg(any(esp32, esp32s3))]
pub use self::soc::cpu_control;
pub use self::soc::{efuse, gpio, peripherals};

#[cfg_attr(esp32, path = "esp32/mod.rs")]
#[cfg_attr(esp32c2, path = "esp32c2/mod.rs")]
#[cfg_attr(esp32c3, path = "esp32c3/mod.rs")]
#[cfg_attr(esp32c6, path = "esp32c6/mod.rs")]
#[cfg_attr(esp32s2, path = "esp32s2/mod.rs")]
#[cfg_attr(esp32s3, path = "esp32s3/mod.rs")]
mod soc;
