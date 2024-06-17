use esp_build::assert_unique_used_features;

fn main() {
    // Ensure that only a single chip is specified:
    assert_unique_used_features!(
        "esp32", "esp32c2", "esp32c3", "esp32c6", "esp32h2", "esp32p4", "esp32s2", "esp32s3"
    );

    // Ensure that exactly a backend is selected:
    assert_unique_used_features!("defmt", "println");

    if cfg!(feature = "custom-halt") && cfg!(feature = "halt-cores") {
        panic!("Only one of `custom-halt` and `halt-cores` can be enabled");
    }

    check_nightly();
}

#[rustversion::all(not(stable),not(since(2024-06-12)))]
fn check_nightly() {
    println!("cargo:rustc-cfg=nightly_before_2024_06_12");
}

#[rustversion::since(2024-06-12)]
fn check_nightly() {
    println!("cargo:rustc-cfg=nightly_since_2024_06_12");
}

#[rustversion::stable]
fn check_nightly() {}
