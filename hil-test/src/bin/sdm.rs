//! SDM Test
//!
//! Uses the common test pins documented in `hil-test/README.md`:
//! - SDM output => first common test pin
//! - RMT RX     => second common test pin

//% CHIP_FILTER: sdm_driver_supported && rmt_driver_supported
//% FEATURES: embassy unstable

#![no_std]
#![no_main]

use embassy_time::Timer;
use esp_hal::{
    delay::Delay,
    gpio::{AnyPin, Level},
    peripherals::{RMT, SDM_CH0},
    rmt::{AnyRmtRxChannel, CHANNEL_RAM_SIZE, PulseCode, Rmt, RmtConfig, RxConfig, RxError},
    sdm::{Channel as SdmChannel, ChannelConfig},
    time::Rate,
    timer::timg::TimerGroup,
};
use hil_test as _;

// A 1 MHz RMT tick on every supported chip.
const RMT_TICK_RATE: Rate = Rate::from_mhz(1);

const SDM_FREQUENCY: Rate = Rate::from_khz(500);
const SDM_WARM_UP_US: u32 = 1_000;
const SAMPLE_TIME_US: u64 = 200;
const RMT_IDLE_NS: u32 = 100_000;
const RX_LEN: usize = CHANNEL_RAM_SIZE;
const DEBUG_WAVE_MAX_CHARS: usize = 200;
const DUTY_MIN: u8 = 10;
const DUTY_MAX: u8 = 245;
// recommended density is [-90, 90]
// ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/sdm.html
const DUTY_STEP: u8 = 20;
const DUTY_RATIO_TOLERANCE_PER_MILLE: u32 = 20;

cfg_select! {
    any(esp32, esp32s3, esp32s31, esp32p4) => {
        macro_rules! rx_channel {
            ($peripherals:expr) => {
                $peripherals.RMT_CH4
            };
        }
    }
    _ => {
        macro_rules! rx_channel {
            ($peripherals:expr) => {
                $peripherals.RMT_CH2
            };
        }
    }
}

struct Context {
    sdm_ch0: SDM_CH0<'static>,
    rmt: RMT<'static>,
    rmt_rx: AnyRmtRxChannel<'static>,
    sdm_pin: AnyPin<'static>,
    rmt_pin: AnyPin<'static>,
}

struct PulseDebug(PulseCode);

impl defmt::Format for PulseDebug {
    fn format(&self, f: defmt::Formatter<'_>) {
        let code = self.0;
        let level = |level| if level == Level::High { 'H' } else { 'L' };

        defmt::write!(
            f,
            "{=char}{=u16}/{=char}{=u16}",
            level(code.level1()),
            code.length1(),
            level(code.level2()),
            code.length2()
        );
    }
}

struct PulseCodesDebug<'a>(&'a [PulseCode]);

impl defmt::Format for PulseCodesDebug<'_> {
    fn format(&self, f: defmt::Formatter<'_>) {
        defmt::write!(f, "[");

        for (i, code) in self.0.iter().enumerate() {
            if i > 0 {
                defmt::write!(f, ", ");
            }
            defmt::write!(f, "{}", PulseDebug(*code));
        }

        defmt::write!(f, "]");
    }
}

struct PulseWaveDebug<'a>(&'a [PulseCode]);

impl defmt::Format for PulseWaveDebug<'_> {
    fn format(&self, f: defmt::Formatter<'_>) {
        let mut wave = [0; DEBUG_WAVE_MAX_CHARS];
        let mut written = 0;
        let mut truncated = false;

        'codes: for code in self.0 {
            for (level, length) in [
                (code.level1(), code.length1()),
                (code.level2(), code.length2()),
            ] {
                let symbol = if level == Level::High { b'^' } else { b'_' };

                for _ in 0..length {
                    if written == wave.len() {
                        truncated = true;
                        break 'codes;
                    }

                    wave[written] = symbol;
                    written += 1;
                }
            }
        }

        let wave = core::str::from_utf8(&wave[..written]).unwrap();
        let ellipsis = if truncated { "..." } else { "" };
        defmt::write!(f, "{=str}{=str}", wave, ellipsis);
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

fn high_ratio_per_mille(data: &[PulseCode], count: usize) -> u32 {
    let mut levels = [(Level::Low, 0_u16); 2 * RX_LEN];
    let mut len = 0;

    // The RMT end marker has a zero length. The other half of the end marker is not valid.
    for (level, length) in data.iter().take(count).flat_map(|code| {
        [
            (code.level1(), code.length1()),
            (code.level2(), code.length2()),
        ]
    }) {
        if length == 0 {
            break;
        }
        levels[len] = (level, length);
        len += 1;
    }

    // The first level can be partial, and `set_duty(0)` cuts the last level. The duty change
    // can also change the last whole SDM period. Do not measure that period.
    let mut periods = levels.get(1..len.saturating_sub(1)).unwrap_or(&[]);
    if let [(first, _), ..] = periods {
        let start_level = *first;
        while let [.., (last, _)] = periods
            && *last == start_level
        {
            periods = &periods[..periods.len() - 1];
        }
    }
    periods = &periods[..periods.len().saturating_sub(2)];

    let (high, total) = periods
        .iter()
        .fold((0_u32, 0_u32), |(high, total), (level, length)| {
            let length = u32::from(*length);
            let high = if *level == Level::High {
                high + length
            } else {
                high
            };
            (high, total + length)
        });

    assert!(total > 0, "RMT did not capture a whole SDM period");
    high * 1_000 / total
}

fn expected_ratio_per_mille(duty: u8) -> u32 {
    (u32::from(duty) * 1_000 + 128) / 256
}

async fn measure_high_ratio(ctx: &mut Context, duty: u8) -> Measurement {
    let config = ChannelConfig::new()
        .with_frequency(SDM_FREQUENCY)
        .unwrap()
        .with_duty(duty);
    let mut channel = SdmChannel::new(ctx.sdm_ch0.reborrow(), ctx.sdm_pin.reborrow(), config);

    Delay::new().delay_micros(SDM_WARM_UP_US);

    let rmt = Rmt::new(ctx.rmt.reborrow(), RmtConfig::default())
        .unwrap()
        .into_async();
    let mut rx_channel = rmt
        .async_rx_channel(
            ctx.rmt_rx.reborrow(),
            RxConfig::default()
                .with_tick_rate(RMT_TICK_RATE)
                .with_idle_ns(RMT_IDLE_NS),
        )
        .unwrap()
        .with_pin(ctx.rmt_pin.reborrow());
    let mut rx_data = [PulseCode::end_marker(); RX_LEN];

    let (rx_result, ()) =
        embassy_futures::join::join(rx_channel.receive_async(&mut rx_data), async {
            Timer::after_micros(SAMPLE_TIME_US).await;

            // Send an idle signal, so that the RMT stops recording pulses.
            channel.set_duty(0);
        })
        .await;

    // A longer capture fills the buffer, which is enough for the measurement.
    let count = match rx_result {
        Ok(count) => count,
        Err(RxError::BufferTooSmall) => RX_LEN,
        Err(e) => panic!("RMT receive error {:?}", e),
    };

    Measurement::new(&rx_data, count)
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use hil_test::assert;

    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        esp_rtos::start(TimerGroup::new(peripherals.TIMG0).timer0);

        let (sdm_pin, rmt_pin) = hil_test::common_test_pins!(peripherals);

        Context {
            sdm_ch0: peripherals.SDM_CH0,
            rmt: peripherals.RMT,
            rmt_rx: rx_channel!(peripherals).into(),
            sdm_pin: AnyPin::from(sdm_pin),
            rmt_pin: AnyPin::from(rmt_pin),
        }
    }

    #[test]
    async fn duties_have_expected_high_ratios(mut ctx: Context) {
        let mut duty = DUTY_MIN;

        loop {
            let measurement = measure_high_ratio(&mut ctx, duty).await;
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
    async fn higher_duties_have_higher_high_ratios(mut ctx: Context) {
        let mut previous_duty = DUTY_MIN;
        let mut previous = measure_high_ratio(&mut ctx, previous_duty).await;
        let mut duty = previous_duty.saturating_add(DUTY_STEP).min(DUTY_MAX);

        while duty != previous_duty {
            let current = measure_high_ratio(&mut ctx, duty).await;
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
