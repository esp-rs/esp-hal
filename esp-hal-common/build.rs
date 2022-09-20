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

    // Configuration symbol for the enabled chip
    if esp32 {
        println!("cargo:rustc-cfg=esp32");
    } else if esp32c3 {
        println!("cargo:rustc-cfg=esp32c3");
    } else if esp32s2 {
        println!("cargo:rustc-cfg=esp32s2");
    } else if esp32s3 {
        println!("cargo:rustc-cfg=esp32s3");
    }

    // Configuration symbol for the architecture of the enabled chip
    if esp32c3 {
        println!("cargo:rustc-cfg=riscv");
    } else {
        println!("cargo:rustc-cfg=xtensa");
    }

    // Configuration symbol for the core count of the enabled chip
    if esp32c3 || esp32s2 {
        println!("cargo:rustc-cfg=single_core");
    } else {
        println!("cargo:rustc-cfg=multi_core");
    }

    // Configuration symbol for the SYSTIMER peripheral
    if esp32c3 || esp32s2 || esp32s3 {
        println!("cargo:rustc-cfg=has_systimer");
    }

    // Configuration symbol for the USB_SERIAL_JTAG peripheral
    if esp32c3 || esp32s3 {
        println!("cargo:rustc-cfg=has_usb_serial_jtag");
    }
}
