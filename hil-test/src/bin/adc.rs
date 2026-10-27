//! ADC Tests
//!
//! An MCP4725 12-bit DAC on the I2C bus drives the ADC1 pin (`hil_test::adc_pin!`) to known
//! voltages so the tests can verify raw sweeps, attenuation scaling, and calibrated millivolt
//! readings.
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

use esp_hal::{
    Blocking,
    analog::adc::{Adc, AdcCalScheme, AdcChannel, AdcConfig, AdcPin, Attenuation},
    delay::Delay,
    i2c::master::{Config as I2cConfig, I2c},
    peripherals::ADC1,
    time::Rate,
};
use hil_test as _;

const MCP4725_ADDR: u8 = 0x60;
const VREF_MV: u32 = 3300;
const SWEEP_STEPS: usize = 11;
const SETTLE_MS: u32 = 10;
const ADC_SAMPLES: usize = 8;
const LINK_TEST_MIN_DELTA: u16 = 800;

// The ESP32-S2 SAR is 13 bits wide. The ESP32-S31 SAR is differential, see the `analog::adc`
// module documentation.
const MAX_RAW: u16 = if cfg!(esp32s2) {
    0x1fff
} else if cfg!(esp32s31) {
    4393
} else {
    0xfff
};

// Maximum expected reading when the DAC drives 0 V.
const MAX_ZERO_RAW: u16 = if cfg!(esp32s31) {
    // Differential SAR: 0 V sits near `ZERO_DIFF_CODE` (2198).
    2400
} else if cfg!(esp32s2) {
    400
} else {
    200
};

fn code_to_target_mv(code: u16) -> u16 {
    ((code as u32 * VREF_MV) / 4096) as u16
}

fn dac_code_for_step(step: usize, total_steps: usize) -> u16 {
    if total_steps <= 1 {
        return 0;
    }
    if step >= total_steps - 1 {
        return 4095;
    }
    ((step as u32 * 4095) / (total_steps as u32 - 1)) as u16
}

fn read_dac_code(i2c: &mut I2c<'_, Blocking>) -> u16 {
    let mut data = [0u8; 3];
    i2c.read(MCP4725_ADDR, &mut data).unwrap();
    ((data[1] as u16) << 4) | (data[2] as u16 >> 4)
}

fn set_dac_code(i2c: &mut I2c<'_, Blocking>, code: u16, delay: &Delay) {
    let code = code & 0x0fff;
    let fast_buf = [(code >> 8) as u8 & 0x0f, code as u8];
    i2c.write(MCP4725_ADDR, &fast_buf).unwrap();
    delay.delay_millis(5);

    if read_dac_code(i2c) != code {
        let reg_buf = [0x40, (code >> 4) as u8, ((code & 0x0f) << 4) as u8];
        i2c.write(MCP4725_ADDR, &reg_buf).unwrap();
        delay.delay_millis(5);
        hil_test::assert_eq!(read_dac_code(i2c), code, "MCP4725 readback mismatch");
    }

    delay.delay_millis(SETTLE_MS);
}

fn read_averaged<'d, PIN, CS>(
    adc: &mut Adc<'d, ADC1<'d>, Blocking>,
    pin: &mut AdcPin<PIN, ADC1<'d>, CS>,
) -> u16
where
    PIN: AdcChannel,
    CS: AdcCalScheme<ADC1<'d>>,
{
    let mut sum = 0u32;
    for _ in 0..ADC_SAMPLES {
        let val = nb::block!(adc.read_oneshot(pin)).unwrap();
        sum += val as u32;
    }
    (sum / ADC_SAMPLES as u32) as u16
}

#[cfg(not(esp32))]
async fn read_averaged_async<'d, PIN, CS>(
    adc: &mut Adc<'d, ADC1<'d>, esp_hal::Async>,
    pin: &mut AdcPin<PIN, ADC1<'d>, CS>,
) -> u16
where
    PIN: AdcChannel,
    CS: AdcCalScheme<ADC1<'d>>,
{
    let mut sum = 0u32;
    for _ in 0..ADC_SAMPLES {
        let val = adc.read_oneshot(pin).await.unwrap();
        sum += val as u32;
    }
    (sum / ADC_SAMPLES as u32) as u16
}

#[embedded_test::tests(default_timeout = 5, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::peripherals::Peripherals;

    use super::*;

    #[init]
    fn init() -> Peripherals {
        esp_hal::init(esp_hal::Config::default())
    }

    #[test]
    fn blocking_raw_sweep(p: Peripherals) {
        let delay = Delay::new();
        let (sda, scl) = hil_test::i2c_pins!(p);
        let mut i2c = I2c::new(
            p.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(hil_test::adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        let mut raw_min = 0;
        let mut raw_max = 0;
        let mut prev_raw = 0;

        for step in 0..SWEEP_STEPS {
            let dac_code = dac_code_for_step(step, SWEEP_STEPS);
            set_dac_code(&mut i2c, dac_code, &delay);
            let raw = read_averaged(&mut adc, &mut pin);

            hil_test::assert!(
                raw <= MAX_RAW,
                "step {}: reading {} exceeds {}",
                step,
                raw,
                MAX_RAW
            );

            if step == 0 {
                raw_min = raw;
                hil_test::assert!(
                    raw <= MAX_ZERO_RAW,
                    "raw@dac=0 ({}) exceeds {}",
                    raw,
                    MAX_ZERO_RAW
                );
            } else if dac_code <= 2900 {
                // Below the 11 dB saturation region (~2350 mV), readings must increase
                // monotonically with each ~330 mV DAC step.
                hil_test::assert!(
                    raw > prev_raw + 50,
                    "step {} (dac={}): raw {} did not increase over previous {}",
                    step,
                    dac_code,
                    raw,
                    prev_raw
                );
            }

            if step == SWEEP_STEPS - 1 {
                raw_max = raw;
            }
            prev_raw = raw;
        }

        let delta = raw_max.saturating_sub(raw_min);
        hil_test::assert!(
            delta >= LINK_TEST_MIN_DELTA,
            "ADC raw delta too small: raw@0={}, raw@4095={}, delta={}",
            raw_min,
            raw_max,
            delta
        );
    }

    // Every attenuation has its own init code and input range: verify that each one responds to a
    // 0 -> ~450 mV step and that higher attenuation produces a smaller raw code for the same input
    // voltage.
    #[test]
    fn every_attenuation_scales_readings(mut p: Peripherals) {
        let delay = Delay::new();
        let (sda, scl) = hil_test::i2c_pins!(p);
        let mut i2c = I2c::new(
            p.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

        let mut gpio = hil_test::adc_pin!(p);
        let mut prev_mid = u16::MAX;

        // ~450 mV (dac = 560) stays well inside the linear range of all four attenuations.
        const MID_DAC_CODE: u16 = 560;

        for attenuation in [
            Attenuation::_0dB,
            Attenuation::_2p5dB,
            Attenuation::_6dB,
            Attenuation::_11dB,
        ] {
            let mut config = AdcConfig::new();
            let mut pin = config.enable_pin(gpio.reborrow(), attenuation);
            let mut adc = Adc::new(p.ADC1.reborrow(), config).unwrap();

            set_dac_code(&mut i2c, 0, &delay);
            let raw_zero = read_averaged(&mut adc, &mut pin);
            hil_test::assert!(raw_zero <= MAX_ZERO_RAW);

            set_dac_code(&mut i2c, MID_DAC_CODE, &delay);
            let raw_mid = read_averaged(&mut adc, &mut pin);
            hil_test::assert!(
                raw_mid <= MAX_RAW,
                "reading {} exceeds {}",
                raw_mid,
                MAX_RAW
            );
            hil_test::assert!(
                raw_mid > raw_zero + 150,
                "attenuation {:?}: raw_mid ({}) did not rise sufficiently above raw_zero ({})",
                attenuation,
                raw_mid,
                raw_zero
            );

            // ESP32-S31 has a single hardware attenuation setting.
            #[cfg(not(esp32s31))]
            {
                hil_test::assert!(
                    raw_mid < prev_mid,
                    "attenuation {:?}: raw_mid ({}) was not smaller than previous attenuation ({})",
                    attenuation,
                    raw_mid,
                    prev_mid
                );
                prev_mid = raw_mid;
            }
            #[cfg(esp32s31)]
            let _ = &mut prev_mid;
        }
    }

    #[test]
    #[cfg(adc_basic_calibration)]
    fn basic_calibration_reads(p: Peripherals) {
        use esp_hal::analog::adc::AdcCalBasic;

        let delay = Delay::new();
        let (sda, scl) = hil_test::i2c_pins!(p);
        let mut i2c = I2c::new(
            p.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

        let mut config = AdcConfig::new();
        let mut pin = config
            .enable_pin_with_cal::<_, AdcCalBasic<ADC1>>(hil_test::adc_pin!(p), Attenuation::_11dB)
            .unwrap();
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        set_dac_code(&mut i2c, 0, &delay);
        let low = read_averaged(&mut adc, &mut pin);
        hil_test::assert!(low <= MAX_ZERO_RAW);

        set_dac_code(&mut i2c, 1500, &delay);
        let mid = read_averaged(&mut adc, &mut pin);

        set_dac_code(&mut i2c, 2800, &delay);
        let high = read_averaged(&mut adc, &mut pin);

        hil_test::assert!(
            low + 300 < mid && mid + 300 < high && high <= MAX_RAW,
            "basic calibration readings not monotonic: low={}, mid={}, high={}",
            low,
            mid,
            high
        );
    }

    // Line fitting needs a factory reference point from eFuse, which the ESP32-H2 dev kits on the
    // HIL runners do not have burnt - `AdcCalLine` (and `AdcCalCurve`, built on it) returns
    // `Err(Error::MissingCalibrationData)` there.
    #[test]
    #[cfg(all(adc_line_calibration, not(esp32h2)))]
    fn line_calibration_accuracy(p: Peripherals) {
        use esp_hal::analog::adc::AdcCalLine;

        let delay = Delay::new();
        let (sda, scl) = hil_test::i2c_pins!(p);
        let mut i2c = I2c::new(
            p.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

        let mut config = AdcConfig::new();
        let mut pin = config
            .enable_pin_with_cal::<_, AdcCalLine<ADC1>>(hil_test::adc_pin!(p), Attenuation::_11dB)
            .unwrap();
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        let mut prev_mv = 0;
        // Test points in the linear region of 11 dB (~330 mV to ~1980 mV).
        for step in 1..=6 {
            let dac_code = dac_code_for_step(step, SWEEP_STEPS);
            let target_mv = code_to_target_mv(dac_code);
            set_dac_code(&mut i2c, dac_code, &delay);

            let cali_mv = read_averaged(&mut adc, &mut pin);
            let err_mv = cali_mv.abs_diff(target_mv);

            hil_test::assert!(
                err_mv <= 250,
                "step {} (dac={}): cali_mv={} too far from target_mv={} (err={} mV)",
                step,
                dac_code,
                cali_mv,
                target_mv,
                err_mv
            );
            hil_test::assert!(
                cali_mv > prev_mv,
                "step {}: cali_mv ({}) did not increase over previous ({})",
                step,
                cali_mv,
                prev_mv
            );
            prev_mv = cali_mv;
        }
    }

    // The ESP32-H2 dev kits on the HIL runners have no line-fitting eFuse data, so calibration
    // reports `MissingCalibrationData` there instead of reading.
    #[test]
    #[cfg(esp32h2)]
    fn line_calibration_missing_data_reports_error(p: Peripherals) {
        use esp_hal::analog::adc::{AdcCalLine, Error};

        let (failed_pin, _) = hil_test::hp_test_pins!(p);
        let good_pin = hil_test::adc_pin!(p);

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

        let value = nb::block!(adc.read_oneshot(&mut pin)).unwrap();
        hil_test::assert!(value <= MAX_RAW, "reading {} exceeds {}", value, MAX_RAW);
    }

    // The ESP32-C2 only characterizes line fitting at 0 dB and 11 dB; the other attenuations have
    // no reference point.
    #[test]
    #[cfg(esp32c2)]
    fn line_calibration_unsupported_attenuation_reports_error(p: Peripherals) {
        use esp_hal::analog::adc::{AdcCalLine, Error};

        let (failed_pin, _) = hil_test::lp_test_pins!(p);
        let good_pin = hil_test::adc_pin!(p);

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

        let value = nb::block!(adc.read_oneshot(&mut pin)).unwrap();
        hil_test::assert!(value <= MAX_RAW, "reading {} exceeds {}", value, MAX_RAW);
    }

    // Not on the ESP32-H2 HIL dev kits, see `line_calibration_accuracy`.
    #[test]
    #[cfg(all(adc_curve_calibration, not(esp32h2)))]
    fn curve_calibration_accuracy(p: Peripherals) {
        use esp_hal::analog::adc::AdcCalCurve;

        let delay = Delay::new();
        let (sda, scl) = hil_test::i2c_pins!(p);
        let mut i2c = I2c::new(
            p.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

        let mut config = AdcConfig::new();
        let mut pin = config
            .enable_pin_with_cal::<_, AdcCalCurve<ADC1>>(hil_test::adc_pin!(p), Attenuation::_11dB)
            .unwrap();
        let mut adc = Adc::new(p.ADC1, config).unwrap();

        let mut prev_mv = 0;
        // Test points up to ~2310 mV (step 7 of 11).
        for step in 1..=7 {
            let dac_code = dac_code_for_step(step, SWEEP_STEPS);
            let target_mv = code_to_target_mv(dac_code);
            set_dac_code(&mut i2c, dac_code, &delay);

            let cali_mv = read_averaged(&mut adc, &mut pin);
            let err_mv = cali_mv.abs_diff(target_mv);

            hil_test::assert!(
                err_mv <= 150,
                "step {} (dac={}): cali_mv={} too far from target_mv={} (err={} mV)",
                step,
                dac_code,
                cali_mv,
                target_mv,
                err_mv
            );
            hil_test::assert!(
                cali_mv > prev_mv,
                "step {}: cali_mv ({}) did not increase over previous ({})",
                step,
                cali_mv,
                prev_mv
            );
            prev_mv = cali_mv;
        }
    }

    #[test]
    #[cfg(not(esp32))]
    async fn async_raw_sweep(p: Peripherals) {
        let delay = Delay::new();
        let (sda, scl) = hil_test::i2c_pins!(p);
        let mut i2c = I2c::new(
            p.I2C0,
            I2cConfig::default().with_frequency(Rate::from_khz(100)),
        )
        .unwrap()
        .with_sda(sda)
        .with_scl(scl);

        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(hil_test::adc_pin!(p), Attenuation::_11dB);
        let mut adc = Adc::new(p.ADC1, config).unwrap().into_async();

        set_dac_code(&mut i2c, 0, &delay);
        let raw_min = read_averaged_async(&mut adc, &mut pin).await;
        hil_test::assert!(raw_min <= MAX_ZERO_RAW);

        set_dac_code(&mut i2c, 2048, &delay);
        let raw_mid = read_averaged_async(&mut adc, &mut pin).await;

        set_dac_code(&mut i2c, 4095, &delay);
        let raw_max = read_averaged_async(&mut adc, &mut pin).await;
        hil_test::assert!(raw_max <= MAX_RAW);

        hil_test::assert!(
            raw_min < raw_mid && raw_mid < raw_max,
            "async readings not monotonic: min={}, mid={}, max={}",
            raw_min,
            raw_mid,
            raw_max
        );
        hil_test::assert!(
            raw_max - raw_min >= LINK_TEST_MIN_DELTA,
            "async delta too small: min={}, max={}",
            raw_min,
            raw_max
        );
    }

    #[test]
    fn unconfigured_channel_returns_error(p: Peripherals) {
        use esp_hal::analog::adc::{AdcChannel, Error};

        let mut config = AdcConfig::new();
        let mut pin = config.enable_pin(hil_test::adc_pin!(p), Attenuation::_11dB);

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
        let mut pin = config.enable_pin(hil_test::adc_pin!(p), Attenuation::_11dB);

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
