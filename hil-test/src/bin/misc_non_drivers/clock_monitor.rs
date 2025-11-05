#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::rtc_cntl::Rtc;

    struct Context<'a> {
        rtc: Rtc<'a>,
    }
    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let rtc = Rtc::new(peripherals.LPWR);

        Context { rtc }
    }

    #[test]
    fn test_estimated_clock(mut ctx: Context<'static>) {
        let target_frequency = if cfg!(esp32c2) {
            26
        } else if cfg!(esp32h2) {
            32
        } else {
            40
        };

        // The internal RC oscillators are not very accurate at all. Leave a 20% acceptance range
        // around the expected value.
        let twenty_percent = 20 * target_frequency / 100;
        let expected_range =
            (target_frequency - twenty_percent)..=(target_frequency + twenty_percent);

        let measured_frequency = ctx.rtc.estimate_xtal_frequency();
        hil_test::assert!(
            expected_range.contains(&measured_frequency),
            "Measured frequency: {}",
            measured_frequency
        );
    }
}
