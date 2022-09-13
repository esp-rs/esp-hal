fn main() {
    let esp32 = cfg!(feature = "esp32");
    let esp32c3 = cfg!(feature = "esp32c3");
    let esp32s2 = cfg!(feature = "esp32s2");
    let esp32s3 = cfg!(feature = "esp32s3");

    // Ensure that exactly one chip has been specified
    let chip_features = [esp32, esp32c3, esp32s2, esp32s3];
    match chip_features.iter().filter(|&&f| f).count() {
        1 => {}
        n => panic!("Exactly 1 chip must be enabled via its Cargo feature, {n} provided"),
    }

    // Inject a configuration symbol for the enabled chip
    if esp32 {
        println!("cargo:rustc-cfg=esp32");
    } else if esp32c3 {
        println!("cargo:rustc-cfg=esp32c3");
    } else if esp32s2 {
        println!("cargo:rustc-cfg=esp32s2");
    } else if esp32s3 {
        println!("cargo:rustc-cfg=esp32s3");
    }
}
