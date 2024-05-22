use esp_build::assert_unique_used_features;

fn main() {
    // Ensure that only a single chip is specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32p4", "esp32s2", "esp32s3"
    );

    // Ensure that exactly a backend is selected:
    assert_unique_used_features!("defmt", "println");

    if is_nightly() {
        println!("cargo:rustc-cfg=nightly");
    }
}

fn is_nightly() -> bool {
    let version_output = std::process::Command::new(
        std::env::var_os("RUSTC").unwrap_or_else(|| std::ffi::OsString::from("rustc")),
    )
    .arg("-V")
    .output()
    .unwrap()
    .stdout;
    let version_string = String::from_utf8_lossy(&version_output);

    version_string.contains("nightly")
}
