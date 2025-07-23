#[cfg_attr(esp32, path = "radio_esp32.rs")]
#[cfg_attr(esp32c2, path = "radio_esp32c2.rs")]
#[cfg_attr(esp32c3, path = "radio_esp32c3.rs")]
#[cfg_attr(esp32c6, path = "radio_esp32c6.rs")]
#[cfg_attr(esp32h2, path = "radio_esp32h2.rs")]
#[cfg_attr(esp32s3, path = "radio_esp32s3.rs")]
#[cfg_attr(esp32s2, path = "radio_esp32s2.rs")]
mod chip_specific;

pub(crate) use chip_specific::*;
