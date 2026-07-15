use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    esp_metadata_generated::Chip::from_cargo_feature()?.define_cfgs();
    setup_i2s_hil_cfgs();
    Ok(())
}

/// Cfgs for named `i2s_i2sN` HIL configurations (`HIL_I2S_INSTANCE` + per-instance PDM).
fn setup_i2s_hil_cfgs() {
    println!("cargo:rerun-if-env-changed=HIL_I2S_INSTANCE");
    println!("cargo:rustc-check-cfg=cfg(i2s_hil_instance, values(\"0\", \"1\", \"2\"))");
    println!("cargo:rustc-check-cfg=cfg(i2s_hil_current_instance_supports_pdm_tx)");
    println!("cargo:rustc-check-cfg=cfg(i2s_hil_current_instance_supports_pdm_rx)");

    let hil_instance = match std::env::var("HIL_I2S_INSTANCE").as_deref() {
        Ok("1") => 1,
        Ok("2") => 2,
        _ => 0,
    };
    println!("cargo:rustc-cfg=i2s_hil_instance=\"{hil_instance}\"");

    esp_metadata_generated::for_each_i2s! {
        (
            $instance:ident, $sys:ident, $mclk:ident, $bclk:ident, $ws:ident, $bclk_rx:ident,
            $ws_rx:ident, $dout:tt, $din:tt, $pdm_tx:tt, $pdm_rx:tt, $pcm2pdm:tt, $pdm2pcm:tt
        ) => {
            let instance_index = match stringify!($instance) {
                "I2S0" => 0,
                "I2S1" => 1,
                "I2S2" => 2,
                _ => unreachable!(),
            };
            if hil_instance == instance_index && $pdm_tx {
                println!("cargo:rustc-cfg=i2s_hil_current_instance_supports_pdm_tx");
            }
            if hil_instance == instance_index && $pdm_rx {
                println!("cargo:rustc-cfg=i2s_hil_current_instance_supports_pdm_rx");
            }
        };
    }
}
