//! ADC + DAC loopback test
//!
//! ESP32 and ESP32-S2 are the only chips with both an ADC and a DAC, and
//! conveniently each DAC channel is wired to the same physical pad as one of
//! the ADC2 channels:
//!
//!   ESP32:    DAC1 / GPIO25 / ADC2_CH8     DAC2 / GPIO26 / ADC2_CH9
//!   ESP32-S2: DAC1 / GPIO17 / ADC2_CH6     DAC2 / GPIO18 / ADC2_CH7
//!
//! That gives us an internal loopback with no wiring: the DAC drives a known
//! voltage onto the pad, and the ADC samples the same pad. This test relies on
//! that to validate that the ADC driver returns *roughly* the right value for
//! a programmed DAC level.
//!
//! Tolerances are deliberately wide:
//!
//!   - the 8-bit DAC has its own non-linearity (~±50 mV near the rails)
//!   - on ESP32-S2 there's no efuse-based ADC calibration in esp-hal yet, so
//!     raw codes can be offset by several hundred LSB
//!   - the DAC drives 0..VDD (~3.3 V) but at 11 dB attenuation the ADC's
//!     usable input range tops out near 3.1 V, so high DAC codes saturate

//% CHIPS: esp32 esp32s2
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

/// DAC1 lives on a different GPIO on each chip; this picks the right pad.
#[cfg(esp32)]
macro_rules! dac1_pad { ($p:expr) => { $p.GPIO25 } }
#[cfg(esp32)]
macro_rules! dac2_pad { ($p:expr) => { $p.GPIO26 } }
#[cfg(esp32)]
type Dac1Gpio = esp_hal::peripherals::GPIO25<'static>;
#[cfg(esp32)]
type Dac2Gpio = esp_hal::peripherals::GPIO26<'static>;

#[cfg(esp32s2)]
macro_rules! dac1_pad { ($p:expr) => { $p.GPIO17 } }
#[cfg(esp32s2)]
macro_rules! dac2_pad { ($p:expr) => { $p.GPIO18 } }
#[cfg(esp32s2)]
type Dac1Gpio = esp_hal::peripherals::GPIO17<'static>;
#[cfg(esp32s2)]
type Dac2Gpio = esp_hal::peripherals::GPIO18<'static>;

#[embedded_test::tests(default_timeout = 5)]
mod tests {
    use esp_hal::{
        Blocking,
        analog::{
            adc::{Adc, AdcChannel, AdcConfig, AdcPin, Attenuation},
            dac::Dac,
        },
        delay::Delay,
        peripherals::{ADC2, DAC1, DAC2},
    };

    use super::{Dac1Gpio, Dac2Gpio};

    /// Full-scale code at this chip's native ADC resolution.
    ///
    /// ESP32's ADC is configurable 9/10/11/12-bit and defaults to 12 (so
    /// `0..=4095`). The S2's ADC is fixed at 13-bit (`0..=8191`), and the
    /// driver returns the masked raw value. Threshold checks below are all
    /// expressed as fractions of `ADC_MAX` so they scale correctly on both.
    #[cfg(esp32)]
    const ADC_MAX: u16 = 4095;
    #[cfg(esp32s2)]
    const ADC_MAX: u16 = 8191;

    /// Delay between writing the DAC and sampling the ADC, so the analog
    /// output has time to settle on the pad.
    const SETTLE_US: u32 = 200;

    /// How many oneshot samples to average per measurement. Without calibration
    /// on the S2 (and on ESP32 ADC2 in general), single-sample jitter can be
    /// tens of LSB; averaging knocks that down so the thresholds below stay
    /// tight enough to be useful.
    const AVG_SAMPLES: u32 = 16;

    /// Maximum allowed backward step between consecutive sweep readings. The
    /// ADC's full-scale code differs by 2x between ESP32 (12-bit) and S2
    /// (13-bit), so express the tolerance as a fraction of full scale —
    /// roughly 1.2% — rather than as a raw LSB count.
    const NOISE_TOLERANCE: u16 = ADC_MAX / 80;

    struct Context {
        delay: Delay,
        dac1: Dac<'static, DAC1<'static>>,
        dac2: Dac<'static, DAC2<'static>>,
        adc: Adc<'static, ADC2<'static>, Blocking>,
        adc_pin1: AdcPin<Dac1Gpio, ADC2<'static>>,
        adc_pin2: AdcPin<Dac2Gpio, ADC2<'static>>,
    }

    impl Context {
        fn read_pin1(&mut self) -> u16 {
            read_avg(&mut self.adc, &mut self.adc_pin1)
        }

        fn read_pin2(&mut self) -> u16 {
            read_avg(&mut self.adc, &mut self.adc_pin2)
        }
    }

    fn read_avg<PIN: AdcChannel>(
        adc: &mut Adc<'static, ADC2<'static>, Blocking>,
        pin: &mut AdcPin<PIN, ADC2<'static>>,
    ) -> u16 {
        let mut sum: u32 = 0;
        for _ in 0..AVG_SAMPLES {
            sum += nb::block!(adc.read_oneshot(pin)).unwrap() as u32;
        }
        (sum / AVG_SAMPLES) as u16
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        // The DAC consumes the GPIO and puts it in analog mode. The ADC needs
        // a handle to the same pad — `set_analog` is idempotent, and the pad's
        // analog mux physically connects both the DAC output driver and the
        // ADC input sampler, so the two `set_analog` calls don't conflict.
        let dac1 = Dac::new(peripherals.DAC1, dac1_pad!(peripherals));
        let dac2 = Dac::new(peripherals.DAC2, dac2_pad!(peripherals));

        // SAFETY: we just moved the original handles into the DAC; the pad is
        // already in analog mode and not used for anything else.
        let adc_pad1 = unsafe { super::steal_dac1_pad() };
        let adc_pad2 = unsafe { super::steal_dac2_pad() };

        let mut config = AdcConfig::<ADC2<'static>>::new();
        let adc_pin1 = config.enable_pin(adc_pad1, Attenuation::_11dB);
        let adc_pin2 = config.enable_pin(adc_pad2, Attenuation::_11dB);
        let adc = Adc::new(peripherals.ADC2, config);

        Context {
            delay: Delay::new(),
            dac1,
            dac2,
            adc,
            adc_pin1,
            adc_pin2,
        }
    }

    // -- DAC1 ----------------------------------------------------------------

    /// DAC at zero must read low. Catches gross channel-mux errors (e.g. the
    /// pre-fix C3 bug where ADC2 sampled an ADC1 pad).
    #[test]
    fn dac1_zero_reads_low(mut ctx: Context) {
        ctx.dac1.write(0);
        ctx.delay.delay_micros(SETTLE_US);

        let raw = ctx.read_pin1();
        defmt::info!("DAC1=0 -> ADC = {}", raw);
        hil_test::assert!(
            raw < ADC_MAX / 10,
            "expected DAC=0 to read < {}, got {}",
            ADC_MAX / 10,
            raw,
        );
    }

    /// DAC at full scale must read near full scale. At 11 dB attenuation the
    /// ADC saturates somewhere around DAC code 240 (depending on VDD), so we
    /// only require >= 80% FS.
    #[test]
    fn dac1_max_reads_high(mut ctx: Context) {
        ctx.dac1.write(255);
        ctx.delay.delay_micros(SETTLE_US);

        let raw = ctx.read_pin1();
        defmt::info!("DAC1=255 -> ADC = {}", raw);
        let lower = ADC_MAX * 8 / 10;
        hil_test::assert!(raw >= lower, "expected >= {}, got {}", lower, raw);
    }

    /// Midpoint check — DAC at 0x80 must land in the middle half of the ADC
    /// range. Bounds gross gain/offset errors.
    #[test]
    fn dac1_midpoint_in_middle_half(mut ctx: Context) {
        ctx.dac1.write(128);
        ctx.delay.delay_micros(SETTLE_US);

        let raw = ctx.read_pin1();
        defmt::info!("DAC1=128 -> ADC = {}", raw);
        let lower = ADC_MAX / 4;
        let upper = ADC_MAX * 3 / 4;
        hil_test::assert!(
            raw >= lower && raw <= upper,
            "expected midpoint in [{}, {}], got {}",
            lower,
            upper,
            raw,
        );
    }

    /// Strongest single signal: ADC reading must increase monotonically with
    /// DAC code, and the full sweep must span most of the ADC range. This
    /// catches sign flips, wrap-around offsets, wrong channels, etc.
    #[test]
    fn dac1_sweep_monotonic(mut ctx: Context) {
        let steps: [u8; 17] = [
            0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
        ];
        let mut readings = [0u16; 17];
        for (i, &v) in steps.iter().enumerate() {
            ctx.dac1.write(v);
            ctx.delay.delay_micros(SETTLE_US);
            readings[i] = ctx.read_pin1();
            defmt::info!("DAC1={} -> ADC = {}", v, readings[i]);
        }

        // Allow a small backward step to absorb residual noise after averaging.
        for w in readings.windows(2) {
            hil_test::assert!(
                w[1] + NOISE_TOLERANCE >= w[0],
                "non-monotonic: {} -> {}",
                w[0],
                w[1],
            );
        }

        let span = readings[steps.len() - 1] - readings[0];
        let min_span = ADC_MAX * 7 / 10;
        hil_test::assert!(
            span > min_span,
            "DAC sweep produced only {} LSB span (expected > {})",
            span,
            min_span,
        );
    }

    // -- DAC2 ----------------------------------------------------------------
    // Mirrors DAC1; covers the second channel mapping.

    #[test]
    fn dac2_zero_reads_low(mut ctx: Context) {
        ctx.dac2.write(0);
        ctx.delay.delay_micros(SETTLE_US);
        let raw = ctx.read_pin2();
        defmt::info!("DAC2=0 -> ADC = {}", raw);
        hil_test::assert!(raw < ADC_MAX / 10, "DAC2=0 raw = {}", raw);
    }

    #[test]
    fn dac2_max_reads_high(mut ctx: Context) {
        ctx.dac2.write(255);
        ctx.delay.delay_micros(SETTLE_US);
        let raw = ctx.read_pin2();
        defmt::info!("DAC2=255 -> ADC = {}", raw);
        hil_test::assert!(raw >= ADC_MAX * 8 / 10, "DAC2=255 raw = {}", raw);
    }

    #[test]
    fn dac2_sweep_monotonic(mut ctx: Context) {
        let steps: [u8; 9] = [0, 32, 64, 96, 128, 160, 192, 224, 255];
        let mut readings = [0u16; 9];
        for (i, &v) in steps.iter().enumerate() {
            ctx.dac2.write(v);
            ctx.delay.delay_micros(SETTLE_US);
            readings[i] = ctx.read_pin2();
            defmt::info!("DAC2={} -> ADC = {}", v, readings[i]);
        }
        for w in readings.windows(2) {
            hil_test::assert!(
                w[1] + NOISE_TOLERANCE >= w[0],
                "DAC2 non-monotonic: {} -> {}",
                w[0],
                w[1],
            );
        }
        let span = readings[steps.len() - 1] - readings[0];
        hil_test::assert!(
            span > ADC_MAX * 7 / 10,
            "DAC2 sweep span = {} (expected > {})",
            span,
            ADC_MAX * 7 / 10,
        );
    }

    /// Cross-channel independence: writing DAC1 must not move the reading on
    /// the DAC2 pad. Catches channel-multiplexing bugs (e.g. en_pad encoded
    /// wrong) — without the fix in this PR, the ADC2 driver on RISC-V used to
    /// sample the wrong pad entirely.
    #[test]
    fn channels_are_independent(mut ctx: Context) {
        ctx.dac1.write(255);
        ctx.dac2.write(0);
        ctx.delay.delay_micros(SETTLE_US);
        let pin1_hi = ctx.read_pin1();
        let pin2_lo = ctx.read_pin2();

        ctx.dac1.write(0);
        ctx.dac2.write(255);
        ctx.delay.delay_micros(SETTLE_US);
        let pin1_lo = ctx.read_pin1();
        let pin2_hi = ctx.read_pin2();

        defmt::info!(
            "pin1_hi={} pin1_lo={} pin2_hi={} pin2_lo={}",
            pin1_hi,
            pin1_lo,
            pin2_hi,
            pin2_lo,
        );

        // Each pad must show a clear separation between its DAC-driven-high
        // and DAC-driven-low state, even when the OTHER pad is driven the
        // opposite way.
        hil_test::assert!(pin1_hi > pin1_lo + ADC_MAX / 2, "pin1 didn't separate");
        hil_test::assert!(pin2_hi > pin2_lo + ADC_MAX / 2, "pin2 didn't separate");
    }
}

unsafe fn steal_dac1_pad() -> Dac1Gpio {
    #[cfg(esp32)]
    return unsafe { esp_hal::peripherals::GPIO25::steal() };
    #[cfg(esp32s2)]
    return unsafe { esp_hal::peripherals::GPIO17::steal() };
}

unsafe fn steal_dac2_pad() -> Dac2Gpio {
    #[cfg(esp32)]
    return unsafe { esp_hal::peripherals::GPIO26::steal() };
    #[cfg(esp32s2)]
    return unsafe { esp_hal::peripherals::GPIO18::steal() };
}
