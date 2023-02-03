#[cfg(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32s2",
    feature = "esp32s3",
))]
fn main() -> Result<(), String> {
    match std::env::var("OPT_LEVEL") {
        Ok(level) => {
            if level != "2" && level != "3" {
                let message = format!(
                    "esp-wifi should be built with optimization level 2 or 3 - yours is {}. 
                    See https://github.com/esp-rs/esp-wifi",
                    level
                )
                .to_string();
                println!("cargo:warning={}", message);
            }

            ()
        }
        Err(_err) => (),
    }

    let features: u8 = cfg!(feature = "wifi") as u8 + cfg!(feature = "ble") as u8;

    if features == 0 {
        return Err("You need to use feature `wifi` and/or `ble`".to_string());
    }

    if cfg!(feature = "esp32s2") && cfg!(feature = "ble") {
        return Err("BLE is not supported for ESP32-S2".into());
    }

    if features >= 2 {
        println!("cargo:rustc-cfg=coex");
    }

    Ok(())
}

#[cfg(not(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32s2",
    feature = "esp32s3",
)))]
fn main() {
    panic!("Select a chip via it's cargo feature");
}
