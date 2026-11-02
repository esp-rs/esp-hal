//! Checks that one-shot ADC conversions are unaffected by a running radio.
//!
//! Radio initialization hands SAR power to the PHY's power detector, which powers the SAR down
//! between its own measurements. Without the ADC driver overriding that, a pad driven to 0 V reads
//! close to full scale.

#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{
        analog::adc::{Adc, AdcConfig, Attenuation},
        clock::CpuClock,
        delay::Delay,
        i2c::master::{Config as I2cConfig, I2c},
        peripherals::Peripherals,
        time::Rate,
        timer::timg::TimerGroup,
    };

    const MCP4725_ADDR: u8 = 0x60;
    // The 0 V DAC output reads a few codes; the failure reads close to full scale.
    const TOLERANCE: u16 = 0x100;

    fn set_dac_code(i2c: &mut I2c<'_, esp_hal::Blocking>, code: u16, delay: &Delay) {
        let code = code & 0x0fff;
        let fast_buf = [(code >> 8) as u8 & 0x0f, code as u8];
        i2c.write(MCP4725_ADDR, &fast_buf).unwrap();
        delay.delay_millis(5);

        let mut data = [0u8; 3];
        i2c.read(MCP4725_ADDR, &mut data).unwrap();
        let readback = ((data[1] as u16) << 4) | (data[2] as u16 >> 4);
        if readback != code {
            let reg_buf = [0x40, (code >> 4) as u8, ((code & 0x0f) << 4) as u8];
            i2c.write(MCP4725_ADDR, &reg_buf).unwrap();
            delay.delay_millis(5);
        }

        delay.delay_millis(10);
    }

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

        let delay = Delay::new();
        let (sda, scl) = hil_test::i2c_pins!(p);
        let mut i2c = I2c::new(
            p.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

        set_dac_code(&mut i2c, 0, &delay);

        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(hil_test::adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config).unwrap();
        let mut read = || nb::block!(adc.read_oneshot(&mut pin)).unwrap();

        let before = read();
        hil_test::assert!(before < TOLERANCE, "0 V pad read {}", before);

        {
            let _radio = cfg_select! {
                soc_has_wifi => esp_radio::wifi::WifiController::new(p.WIFI, Default::default()),
                _ => esp_radio::ble::controller::BleConnector::new(p.BT, Default::default()),
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
