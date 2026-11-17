//! Demo test suite using embedded-test
//!
//! You can run this using `cargo test` as usual.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    //%if option("defmt")
    use defmt::assert_eq;
    //%endif
    use esp_hal as _;

    #[init]
    fn init() {
        let _ = esp_hal::init(esp_hal::Config::default());

        //%if option("defmt")
        rtt_target::rtt_init_defmt!();
        //%endif
    }

    #[test]
    fn hello_test() {
        //%if option("defmt")
        defmt::info!("Running test!");
        //%endif

        assert_eq!(1 + 1, 2);
    }
}
