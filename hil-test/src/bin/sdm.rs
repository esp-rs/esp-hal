//! SDM Test
//!
//! Uses the common test pins documented in `hil-test/README.md`:
//! - SDM output => first common test pin
//! - RMT RX     => second common test pin

//% CHIPS: esp32 esp32c3 esp32c5 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{AnyPin, Level, Output, OutputConfig, interconnect::PeripheralInput},
    peripherals::{GPIO_SD, RMT},
    rmt::{CHANNEL_RAM_SIZE, Channel, PulseCode, Rmt, Rx, RxChannelConfig, RxChannelCreator},
    sdm::{Sdm, SdmConfig},
    time::Rate,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(esp32h2)] {
        const RMT_FREQUENCY: Rate = Rate::from_mhz(32);
        // default clock source for h2 is 32Mhz
        const RMT_DIVIDER: u8 = 32;
    } else {
        const RMT_FREQUENCY: Rate = Rate::from_mhz(80);
        // default clock source for other models is 80Mhz
        const RMT_DIVIDER: u8 = 80;
    }
    // make sure that all models uses 1 Mhz clock
}

const SDM_FREQUENCY: Rate = Rate::from_khz(500);
const SDM_WARM_UP_US: u32 = 1_000;
const SAMPLE_TIME_US: u32 = 200;
const RMT_IDLE_THRESHOLD: u16 = 100;
const RX_LEN: usize = CHANNEL_RAM_SIZE;
const DEBUG_WAVE_MAX_CHARS: usize = 200;
const DUTY_MIN: u8 = 10;
const DUTY_MAX: u8 = 245;
// recommended density is [-90, 90]
// ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/sdm.html
const DUTY_STEP: u8 = 20;
const DUTY_RATIO_TOLERANCE_PER_MILLE: u32 = 20;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s3))] {
        macro_rules! rx_channel_creator {
            ($rmt:expr) => {
                $rmt.channel4
            };
        }
    } else {
        macro_rules! rx_channel_creator {
            ($rmt:expr) => {
                $rmt.channel2
            };
        }
    }
}

struct Context {
    gpio_sd: GPIO_SD<'static>,
    rmt: RMT<'static>,
    sdm_pin: AnyPin<'static>,
    rmt_pin: AnyPin<'static>,
}

struct PulseDebug(PulseCode);

impl core::fmt::Debug for PulseDebug {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let code = self.0;
        let level1 = if code.level1() == Level::High {
            'H'
        } else {
            'L'
        };
        let level2 = if code.level2() == Level::High {
            'H'
        } else {
            'L'
        };

        write!(f, "{level1}{}/{level2}{}", code.length1(), code.length2())
    }
}

struct PulseCodesDebug<'a>(&'a [PulseCode]);

impl core::fmt::Debug for PulseCodesDebug<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut list = f.debug_list();

        for code in self.0 {
            list.entry(&PulseDebug(*code));
        }

        list.finish()
    }
}

struct PulseWaveDebug<'a>(&'a [PulseCode]);

impl core::fmt::Display for PulseWaveDebug<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut written = 0;

        for code in self.0 {
            for (level, length) in [
                (code.level1(), code.length1()),
                (code.level2(), code.length2()),
            ] {
                if length == 0 {
                    continue;
                }

                let symbol = if level == Level::High { '^' } else { '_' };
                let width = usize::from(length);

                for _ in 0..width {
                    if written >= DEBUG_WAVE_MAX_CHARS {
                        return f.write_str("...");
                    }

                    written += 1;
                    write!(f, "{symbol}")?;
                }
            }
        }

        Ok(())
    }
}

struct Measurement {
    ratio: u32,
    count: usize,
    pulses: [PulseCode; RX_LEN],
}

impl Measurement {
    fn new(data: &[PulseCode; RX_LEN], count: usize) -> Self {
        Self {
            ratio: high_ratio_per_mille(data, count),
            count,
            pulses: *data,
        }
    }

    fn valid_pulses(&self) -> &[PulseCode] {
        &self.pulses[..self.count.min(RX_LEN)]
    }
}

fn configure_rx_channel<'a>(
    rmt: Rmt<'a, Blocking>,
    pin: impl PeripheralInput<'a>,
) -> Channel<'a, Blocking, Rx> {
    rx_channel_creator!(rmt)
        .configure_rx(
            &RxChannelConfig::default()
                .with_clk_divider(RMT_DIVIDER)
                .with_idle_threshold(RMT_IDLE_THRESHOLD),
        )
        .unwrap()
        .with_pin(pin)
}

fn high_ratio_per_mille(data: &[PulseCode], count: usize) -> u32 {
    let mut high = 0_u32;
    let mut low = 0_u32;
    let mut remaining = SAMPLE_TIME_US;

    for code in data.iter().take(count) {
        if code.length1() != 0 {
            let length = u32::from(code.length1()).min(remaining);

            if code.level1() == Level::High {
                high += length;
            } else {
                low += length;
            }

            remaining -= length;
            if remaining == 0 {
                break;
            }
        }

        if code.length2() != 0 {
            let length = u32::from(code.length2()).min(remaining);

            if code.level2() == Level::High {
                high += length;
            } else {
                low += length;
            }

            remaining -= length;
            if remaining == 0 {
                break;
            }
        }
    }

    assert!(high + low > 0, "RMT did not capture any SDM pulse duration");
    high * 1_000 / (high + low)
}

fn expected_ratio_per_mille(duty: u8) -> u32 {
    (u32::from(duty) * 1_000 + 128) / 256
}

fn measure_high_ratio(ctx: &mut Context, duty: u8) -> Measurement {
    let sdm = Sdm::new(ctx.gpio_sd.reborrow(), SdmConfig::default());
    let config = sdm
        .channel_config()
        .with_frequency(SDM_FREQUENCY)
        .unwrap()
        .with_duty(duty);
    let channel = sdm
        .channel0
        .connect(ctx.sdm_pin.reborrow(), config)
        .unwrap();

    Delay::new().delay_micros(SDM_WARM_UP_US);

    let rmt = Rmt::new(ctx.rmt.reborrow(), RMT_FREQUENCY).unwrap();
    let rx_channel = configure_rx_channel(rmt, ctx.rmt_pin.reborrow());
    let mut rx_data = [PulseCode::end_marker(); RX_LEN];
    let rx_transaction = rx_channel.receive(&mut rx_data).unwrap();

    Delay::new().delay_micros(SAMPLE_TIME_US);

    let _creator = channel.disconnect();
    let mut output = Output::new(ctx.sdm_pin.reborrow(), Level::Low, OutputConfig::default());
    output.set_low();
    // send idle signal so that RMT will stop recording pulses

    let (count, _rx_channel) = rx_transaction.wait().unwrap();

    Measurement::new(&rx_data, count)
}

#[embedded_test::tests(default_timeout = 10)]
mod tests {
    use hil_test::assert;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let (sdm_pin, rmt_pin) = hil_test::common_test_pins!(peripherals);

        Context {
            gpio_sd: peripherals.GPIO_SD,
            rmt: peripherals.RMT,
            sdm_pin: AnyPin::from(sdm_pin),
            rmt_pin: AnyPin::from(rmt_pin),
        }
    }

    #[test]
    fn duties_have_expected_high_ratios(mut ctx: Context) {
        let mut duty = DUTY_MIN;

        loop {
            let measurement = measure_high_ratio(&mut ctx, duty);
            let expected = expected_ratio_per_mille(duty);
            let error = measurement.ratio.abs_diff(expected);
            let pulses = measurement.valid_pulses();

            assert!(
                error <= DUTY_RATIO_TOLERANCE_PER_MILLE,
                "expected duty {} high ratio to be {} +/- {} per mille, got {}; count={}; pulses={:?}; wave={}",
                duty,
                expected,
                DUTY_RATIO_TOLERANCE_PER_MILLE,
                measurement.ratio,
                measurement.count,
                PulseCodesDebug(pulses),
                PulseWaveDebug(pulses),
            );

            if duty == DUTY_MAX {
                break;
            }

            duty = duty.saturating_add(DUTY_STEP).min(DUTY_MAX);
        }
    }

    #[test]
    fn higher_duties_have_higher_high_ratios(mut ctx: Context) {
        let mut previous_duty = DUTY_MIN;
        let mut previous = measure_high_ratio(&mut ctx, previous_duty);
        let mut duty = previous_duty.saturating_add(DUTY_STEP).min(DUTY_MAX);

        while duty != previous_duty {
            let current = measure_high_ratio(&mut ctx, duty);
            let previous_pulses = previous.valid_pulses();
            let current_pulses = current.valid_pulses();

            assert!(
                current.ratio > previous.ratio,
                "expected duty {} high ratio ({}) to be higher than duty {} ({}); current_count={}; current_pulses={:?}; current_wave={}; previous_count={}; previous_pulses={:?}; previous_wave={}",
                duty,
                current.ratio,
                previous_duty,
                previous.ratio,
                current.count,
                PulseCodesDebug(current_pulses),
                PulseWaveDebug(current_pulses),
                previous.count,
                PulseCodesDebug(previous_pulses),
                PulseWaveDebug(previous_pulses),
            );

            if duty == DUTY_MAX {
                break;
            }

            previous_duty = duty;
            previous = current;
            duty = duty.saturating_add(DUTY_STEP).min(DUTY_MAX);
        }
    }
}
