//! Demo test suite using embedded-test
//!
//! You can run this using `cargo test` as usual.

#![no_std]
#![no_main]

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(test)]
#[embedded_test::tests(executor = esp_rtos::embassy::Executor::new())]
mod tests {
    //%if option("defmt")
    use defmt::assert_eq;
    //%endif

    #[init]
    fn init() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0);

        //%if option("defmt")
        rtt_target::rtt_init_defmt!();
        //%endif
    }

    #[test]
    async fn hello_test() {
        //%if option("defmt")
        defmt::info!("Running test!");
        //%endif

        embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        assert_eq!(1 + 1, 2);
    }
}
