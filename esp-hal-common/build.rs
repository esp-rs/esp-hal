fn main() {
    let esp32 = cfg!(feature = "esp32");
    let esp32c2 = cfg!(feature = "esp32c2");
    let esp32c3 = cfg!(feature = "esp32c3");
    let esp32c6 = cfg!(feature = "esp32c6");
    let esp32s2 = cfg!(feature = "esp32s2");
    let esp32s3 = cfg!(feature = "esp32s3");

    // Ensure that exactly one chip has been specified
    let chip_features = [esp32, esp32c2, esp32c3, esp32c6, esp32s2, esp32s3];
    match chip_features.iter().filter(|&&f| f).count() {
        1 => {}
        n => panic!("Exactly 1 chip must be enabled via its Cargo feature, {n} provided"),
    }

    if cfg!(feature = "esp32") && cfg!(feature = "esp32_40mhz") && cfg!(feature = "esp32_26mhz") {
        panic!("Only one xtal speed feature can be selected");
    }
    if cfg!(feature = "esp32c2")
        && cfg!(feature = "esp32c2_40mhz")
        && cfg!(feature = "esp32c2_26mhz")
    {
        panic!("Only one xtal speed feature can be selected");
    }

    // Define all required configuration symbols for the enabled chip.
    //
    // When adding a new device, at the bare minimum the following symbols MUST be
    // defined:
    //   - the name of the device
    //   - the architecture ('riscv' or 'xtensa')
    //   - the core count ('single_core' or 'multi_core')
    //
    // Additionally, the following symbols MAY be defined if present:
    //   - 'dac'
    //   - 'gdma'
    //   - 'i2c1'
    //   - 'i2s'
    //   - 'mcpwm'
    //   - 'pdma'
    //   - 'rmt'
    //   - 'spi3'
    //   - 'systimer'
    //   - 'timg0'
    //   - 'timg1'
    //   - 'uart2'
    //   - 'usb_otg'
    //   - 'usb_serial_jtag'
    //   - 'aes'
    //   - 'plic'
    //   - 'radio'
    //
    // New symbols can be added as needed, but please be sure to update both this
    // comment and the required vectors below.
    let symbols = if esp32 {
        vec![
            "esp32",
            "xtensa",
            "mcpwm",
            "multi_core",
            "dac",
            "i2c1",
            "i2s",
            "pdma",
            "rmt",
            "spi3",
            "timg0",
            "timg1",
            "uart2",
            "aes",
            "radio",
        ]
    } else if esp32c2 {
        vec![
            "esp32c2",
            "riscv",
            "single_core",
            "gdma",
            "systimer",
            "timg0",
            "radio",
        ]
    } else if esp32c3 {
        vec![
            "esp32c3",
            "riscv",
            "single_core",
            "gdma",
            "i2s",
            "rmt",
            "spi3",
            "systimer",
            "timg0",
            "timg1",
            "usb_serial_jtag",
            "aes",
            "radio",
        ]
    } else if esp32c6 {
        vec![
            "esp32c6",
            "riscv",
            "single_core",
            "gdma",
            "i2s",
            "mcpwm",
            "rmt",
            "systimer",
            "timg0",
            "timg1",
            "usb_serial_jtag",
            "plic",
            "aes",
            "radio",
        ]
    } else if esp32s2 {
        vec![
            "esp32s2",
            "xtensa",
            "single_core",
            "dac",
            "i2c1",
            "i2s",
            "pdma",
            "rmt",
            "spi3",
            "systimer",
            "timg0",
            "timg1",
            "usb_otg",
            "aes",
            "radio",
        ]
    } else if esp32s3 {
        vec![
            "esp32s3",
            "xtensa",
            "multi_core",
            "gdma",
            "i2c1",
            "i2s",
            "mcpwm",
            "rmt",
            "spi3",
            "systimer",
            "timg0",
            "timg1",
            "uart2",
            "usb_otg",
            "usb_serial_jtag",
            "aes",
            "radio",
        ]
    } else {
        unreachable!(); // We've already confirmed exactly one chip was selected
    };

    for symbol in symbols {
        println!("cargo:rustc-cfg={symbol}");
    }
}
