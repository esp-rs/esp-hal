//! ADC Tests
//!
//! Nothing drives the ADC pins to a known voltage yet, so these tests only check that conversions
//! complete and stay within the ADC's output range, not that the readings are accurate.
//!
//! The `self_cal` configuration ignores the eFuse init code, forcing `Adc::new` to measure it at
//! runtime the way it has to on chips without calibration eFuses.

//% CHIP_FILTER: adc_driver_supported
//% FEATURES: unstable
//% FEATURES(efuse):
//% CHIP_FILTER(self_cal): adc_driver_supported && !esp32 && !esp32c5 && !esp32s31
//% CARGO-CONFIG(self_cal): target.'cfg(target_os="none")'.rustflags=["--cfg=__test_adc_self_cal"]

#![no_std]
#![no_main]

use hil_test as _;

// A pad of a connected test pair where one exists, so the other pad can drive it once the tests
// check the readings. Neither LP pad is on ADC1 on the ESP32, ESP32-H2, ESP32-P4 and ESP32-S31, so
// those use their HP pair where it has an ADC1 pad, or an unconnected ADC1 pad.
macro_rules! adc_pin {
    ($peripherals:expr) => {{
        cfg_select! {
            esp32 => $peripherals.GPIO36,
            esp32h2 => hil_test::hp_test_pins!($peripherals).0,
            esp32p4 => $peripherals.GPIO20,
            esp32s31 => hil_test::hp_test_pins!($peripherals).1,
            esp32c61 => hil_test::lp_test_pins!($peripherals).1,
            // esp32c2, esp32c3, esp32c5, esp32c6, esp32s2, esp32s3
            _ => hil_test::lp_test_pins!($peripherals).0,
        }
    }};
}

const READS: usize = 16;

// The ESP32-S2 SAR is 13 bits wide. The ESP32-S31 SAR is differential, see the `analog::adc`
// module documentation.
const MAX_RAW: u16 = if cfg!(esp32s2) {
    0x1fff
} else if cfg!(esp32s31) {
    4393
} else {
    0xfff
};

macro_rules! assert_reads_complete {
    ($adc:expr, $pin:expr) => {{
        for _ in 0..READS {
            let value = nb::block!($adc.read_oneshot(&mut $pin)).unwrap();
            hil_test::assert!(value <= MAX_RAW, "reading {} exceeds {}", value, MAX_RAW);
        }
    }};
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{
        analog::adc::{Adc, AdcConfig, Attenuation},
        peripherals::Peripherals,
    };

    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn blocking_reads_complete(p: Peripherals) {
        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config);

        assert_reads_complete!(adc, pin);
    }

    // Every attenuation has its own init code, so each one goes through calibration separately.
    #[test]
    fn every_attenuation_reads_complete(mut p: Peripherals) {
        let mut gpio = adc_pin!(p);

        for attenuation in [
            Attenuation::_0dB,
            Attenuation::_2p5dB,
            Attenuation::_6dB,
            Attenuation::_11dB,
        ] {
            let mut config = AdcConfig::new();
            let mut pin = config.enable_pin(gpio.reborrow(), attenuation);
            let mut adc = Adc::new(p.ADC1.reborrow(), config);

            assert_reads_complete!(adc, pin);
        }
    }

    #[test]
    #[cfg(not(any(esp32, esp32s31)))]
    fn basic_calibration_reads_complete(p: Peripherals) {
        use esp_hal::{analog::adc::AdcCalBasic, peripherals::ADC1};

        let mut config = AdcConfig::new();
        let mut pin =
            config.enable_pin_with_cal::<_, AdcCalBasic<ADC1>>(adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config);

        assert_reads_complete!(adc, pin);
    }

    // Line fitting needs a factory reference point from eFuse, which the ESP32-H2 dev kits on the
    // HIL runners do not have burnt - `AdcCalLine` (and `AdcCalCurve`, built on it) panics there.
    #[test]
    #[cfg(not(any(esp32h2, esp32s31)))]
    fn line_calibration_reads_complete(p: Peripherals) {
        use esp_hal::{analog::adc::AdcCalLine, peripherals::ADC1};

        let mut config = AdcConfig::new();
        let mut pin =
            config.enable_pin_with_cal::<_, AdcCalLine<ADC1>>(adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config);

        for _ in 0..READS {
            nb::block!(adc.read_oneshot(&mut pin)).unwrap();
        }
    }

    // Not on the ESP32-H2 HIL dev kits, see `line_calibration_reads_complete`.
    #[test]
    #[cfg(any(esp32c3, esp32c5, esp32c6, esp32c61, esp32p4, esp32s3))]
    fn curve_calibration_reads_complete(p: Peripherals) {
        use esp_hal::{analog::adc::AdcCalCurve, peripherals::ADC1};

        let mut config = AdcConfig::new();
        let mut pin =
            config.enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config);

        for _ in 0..READS {
            nb::block!(adc.read_oneshot(&mut pin)).unwrap();
        }
    }

    #[test]
    #[cfg(not(esp32))]
    async fn async_reads_complete(p: Peripherals) {
        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config).into_async();

        for _ in 0..READS {
            let value = adc.read_oneshot(&mut pin).await;
            hil_test::assert!(value <= MAX_RAW, "reading {} exceeds {}", value, MAX_RAW);
        }
    }
}
