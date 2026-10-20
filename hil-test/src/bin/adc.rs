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
//% CHIP_FILTER(self_cal): adc_calibration_v1_scheme_supported && !esp32c5
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
        let mut adc = Adc::new(p.ADC1, config).unwrap();

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
            let mut adc = Adc::new(p.ADC1.reborrow(), config).unwrap();

            assert_reads_complete!(adc, pin);
        }
    }

    #[test]
    #[cfg(adc_basic_calibration)]
    fn basic_calibration_reads_complete(p: Peripherals) {
        use esp_hal::{analog::adc::AdcCalBasic, peripherals::ADC1};

        let mut config = AdcConfig::new();
        let mut pin = config
            .enable_pin_with_cal::<_, AdcCalBasic<ADC1>>(adc_pin!(p), Attenuation::_11dB)
            .unwrap();
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        assert_reads_complete!(adc, pin);
    }

    // Line fitting needs a factory reference point from eFuse, which the ESP32-H2 dev kits on the
    // HIL runners do not have burnt - `AdcCalLine` (and `AdcCalCurve`, built on it) returns
    // `Err(Error::MissingCalibrationData)` there.
    #[test]
    #[cfg(all(adc_line_calibration, not(esp32h2)))]
    fn line_calibration_reads_complete(p: Peripherals) {
        use esp_hal::{analog::adc::AdcCalLine, peripherals::ADC1};

        let mut config = AdcConfig::new();
        let mut pin = config
            .enable_pin_with_cal::<_, AdcCalLine<ADC1>>(adc_pin!(p), Attenuation::_11dB)
            .unwrap();
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        for _ in 0..READS {
            nb::block!(adc.read_oneshot(&mut pin)).unwrap();
        }
    }

    // The ESP32-H2 dev kits on the HIL runners have no line-fitting eFuse data, so calibration
    // reports `MissingCalibrationData` there instead of reading.
    #[test]
    #[cfg(esp32h2)]
    fn line_calibration_missing_data_reports_error(p: Peripherals) {
        use esp_hal::{
            analog::adc::{AdcCalLine, Error},
            peripherals::ADC1,
        };

        let (failed_pin, good_pin) = hil_test::hp_test_pins!(p);

        let mut config = AdcConfig::new();
        let result =
            config.enable_pin_with_cal::<_, AdcCalLine<ADC1>>(failed_pin, Attenuation::_11dB);
        hil_test::assert!(
            matches!(result, Err(Error::MissingCalibrationData)),
            "line calibration without eFuse data must report MissingCalibrationData"
        );

        // The failed call must not have configured anything: enabling another channel still works.
        let mut pin = config.enable_pin(good_pin, Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        assert_reads_complete!(adc, pin);
    }

    // The ESP32-C2 only characterizes line fitting at 0 dB and 11 dB; the other attenuations have
    // no reference point.
    #[test]
    #[cfg(esp32c2)]
    fn line_calibration_unsupported_attenuation_reports_error(p: Peripherals) {
        use esp_hal::{
            analog::adc::{AdcCalLine, Error},
            peripherals::ADC1,
        };

        let (failed_pin, good_pin) = hil_test::lp_test_pins!(p);

        let mut config = AdcConfig::new();
        let result =
            config.enable_pin_with_cal::<_, AdcCalLine<ADC1>>(failed_pin, Attenuation::_6dB);
        hil_test::assert!(
            matches!(
                result,
                Err(Error::UnsupportedAttenuation(Attenuation::_6dB))
            ),
            "line calibration at 6 dB must report UnsupportedAttenuation"
        );

        // The failed call must not have configured anything: enabling another channel still works.
        let mut pin = config.enable_pin(good_pin, Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        assert_reads_complete!(adc, pin);
    }

    // Not on the ESP32-H2 HIL dev kits, see `line_calibration_reads_complete`.
    #[test]
    #[cfg(all(adc_curve_calibration, not(esp32h2)))]
    fn curve_calibration_reads_complete(p: Peripherals) {
        use esp_hal::{analog::adc::AdcCalCurve, peripherals::ADC1};

        let mut config = AdcConfig::new();
        let mut pin = config
            .enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(adc_pin!(p), Attenuation::_11dB)
            .unwrap();
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        for _ in 0..READS {
            nb::block!(adc.read_oneshot(&mut pin)).unwrap();
        }
    }

    #[test]
    #[cfg(not(esp32))]
    async fn async_reads_complete(p: Peripherals) {
        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config).unwrap().into_async();

        for _ in 0..READS {
            let value = adc.read_oneshot(&mut pin).await.unwrap();
            hil_test::assert!(value <= MAX_RAW, "reading {} exceeds {}", value, MAX_RAW);
        }
    }
    #[test]
    fn unconfigured_channel_returns_error(p: Peripherals) {
        use esp_hal::analog::adc::{AdcChannel, Error};

        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(adc_pin!(p), Attenuation::_11dB);

        let empty = AdcConfig::new();
        let mut adc = Adc::new(p.ADC1, empty).unwrap();

        let channel = pin.pin.adc_channel();
        let result = nb::block!(adc.read_oneshot(&mut pin));
        hil_test::assert!(
            matches!(
                result,
                Err(Error::ChannelNotConfigured(ch)) if ch == channel
            ),
            "reading an unconfigured channel must report ChannelNotConfigured"
        );
    }

    #[test]
    #[cfg(not(esp32))]
    async fn async_unconfigured_channel_returns_error(p: Peripherals) {
        use esp_hal::analog::adc::{AdcChannel, Error};

        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(adc_pin!(p), Attenuation::_11dB);

        let empty = AdcConfig::new();
        let mut adc = Adc::new(p.ADC1, empty).unwrap().into_async();

        let channel = pin.pin.adc_channel();
        let result = adc.read_oneshot(&mut pin).await;
        hil_test::assert!(
            matches!(
                result,
                Err(Error::ChannelNotConfigured(ch)) if ch == channel
            ),
            "reading an unconfigured channel must report ChannelNotConfigured"
        );
    }

    // ADC2 ownership is tracked by a global claim flag, driven here the same
    // way the radio drives it: dropping a never-claimed handle or failing to
    // claim must not release a live owner, while dropping a live driver must.
    #[test]
    #[cfg(esp32)]
    fn adc2_contention_keeps_claim_and_allows_retry(mut p: Peripherals) {
        use esp_hal::{
            Internal,
            analog::adc::{Adc, AdcConfig, Error, release_adc2, try_claim_adc2},
        };

        // Simulate the radio holding ADC2.
        try_claim_adc2(unsafe { Internal::conjure() }).unwrap();

        // Dropping a never-claimed handle must not release the live owner.
        // (`Drop for ADC2` used to wipe the radio's claim here.)
        drop(p.ADC2.reborrow());
        hil_test::assert!(
            try_claim_adc2(unsafe { Internal::conjure() }).is_err(),
            "dropping an unused ADC2 handle must not release the live owner"
        );

        hil_test::assert!(
            matches!(
                Adc::new(p.ADC2.reborrow(), AdcConfig::new()),
                Err(Error::Adc2InUse)
            ),
            "ADC2 claim must fail while the radio holds it"
        );
        // The failed claim above must not have released the live owner.
        hil_test::assert!(
            matches!(
                Adc::new(p.ADC2.reborrow(), AdcConfig::new()),
                Err(Error::Adc2InUse)
            ),
            "failed claim must not release the live ADC2 owner"
        );

        // Simulate radio deinit, then retry with the owned token.
        release_adc2(unsafe { Internal::conjure() });
        let adc = Adc::new(p.ADC2, AdcConfig::new()).unwrap();

        // Dropping the live driver releases the claim again.
        drop(adc);
        hil_test::assert!(
            try_claim_adc2(unsafe { Internal::conjure() }).is_ok(),
            "dropping a live ADC2 driver must release the claim"
        );
        // Leave the flag unclaimed for later tests.
        release_adc2(unsafe { Internal::conjure() });
    }
}
