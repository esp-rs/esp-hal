//! Checks that one-shot ADC conversions are unaffected by a running radio.
//!
//! Radio initialization hands SAR power to the PHY's power detector, which powers the SAR down
//! between its own measurements. Without the ADC driver overriding that, a grounded pad reads close
//! to full scale.

#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{
        analog::adc::{Adc, AdcConfig, Attenuation},
        clock::CpuClock,
        gpio::{Level, Output, OutputConfig},
        peripherals::Peripherals,
        timer::timg::TimerGroup,
    };

    // The grounded pad reads a few codes; the failure reads close to full scale.
    const TOLERANCE: u16 = 0x100;

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    async fn adc_reads_unaffected_by_radio(p: Peripherals) {
        let timg0: TimerGroup<'_, _> = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        // The ADC pads of the `adc` test, driven by the other pad of their pair.
        cfg_select! {
            any(esp32c5, esp32c6) => {
                let (adc_pin, driver) = hil_test::lp_test_pins!(p);
            }
            esp32c61 => {
                let (driver, adc_pin) = hil_test::lp_test_pins!(p);
            }
            esp32h2 => {
                let (adc_pin, driver) = hil_test::hp_test_pins!(p);
            }
        }

        let _driver = Output::new(driver, Level::Low, OutputConfig::default());

        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(adc_pin, Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config);
        let mut read = || nb::block!(adc.read_oneshot(&mut pin)).unwrap();

        let before = read();
        hil_test::assert!(before < TOLERANCE, "grounded pad read {}", before);

        {
            let _radio = cfg_select! {
                soc_has_wifi => {
                    esp_radio::wifi::WifiController::new(p.WIFI, Default::default())
                }
                _ => {
                    esp_radio::ble::controller::BleConnector::new(p.BT, Default::default())
                }
            }
            .unwrap();

            let with_radio = read();
            hil_test::assert!(
                with_radio.abs_diff(before) < TOLERANCE,
                "read {} with the radio running, {} before",
                with_radio,
                before
            );
        }

        let after = read();
        hil_test::assert!(
            after.abs_diff(before) < TOLERANCE,
            "read {} after the radio stopped, {} before",
            after,
            before
        );
    }
}
