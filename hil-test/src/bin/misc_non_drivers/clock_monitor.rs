#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::clock::Clocks;

    #[test]
    fn test_estimated_clock() {
        let _p = esp_hal::init(esp_hal::Config::default());

        let target_frequency = if cfg!(esp32c2) {
            26
        } else if cfg!(esp32h2) {
            32
        } else if cfg!(esp32c5) {
            48
        } else {
            40
        };

        let xtal_frequency = Clocks::get().xtal_clock.as_mhz();
        hil_test::assert!(
            xtal_frequency == target_frequency,
            "Measured frequency: {}",
            xtal_frequency
        );
    }
}
