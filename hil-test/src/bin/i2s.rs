//! I2S Loopback Test
//!
//! This test uses I2S TX to transmit known data to I2S RX (forced to slave mode
//! with loopback mode enabled).

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable
// FIXME: re-enable on ESP32 when it no longer fails spuriously

#![no_std]
#![no_main]

use esp_hal::{
    Async,
    delay::Delay,
    dma::DmaTxStreamBuf,
    gpio::{AnyPin, NoPin, Pin},
    i2s::master::{Channels, Config, DataFormat, I2s, I2sTx},
    peripherals::I2S0,
    time::Rate,
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2))] {
        type DmaChannel0<'d> = esp_hal::peripherals::DMA_I2S0<'d>;
    } else {
        type DmaChannel0<'d> = esp_hal::peripherals::DMA_CH0<'d>;
    }
}

const BUFFER_SIZE: usize = 2000;

#[derive(Clone)]
struct SampleSource {
    i: u8,
}

impl SampleSource {
    // choose values which DON'T restart on every descriptor buffer's start
    const ADD: u8 = 5;
    const CUT_OFF: u8 = 113;

    fn new() -> Self {
        Self { i: 0 }
    }
}

impl Iterator for SampleSource {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        let i = self.i;
        self.i = (i + Self::ADD) % Self::CUT_OFF;
        Some(i)
    }
}

#[embassy_executor::task]
async fn writer(mut tx_buffer: DmaTxStreamBuf, i2s_tx: I2sTx<'static, Async>) {
    let mut samples = SampleSource::new();
    tx_buffer.push_with(|buf| {
        for b in buf.iter_mut() {
            *b = samples.next().unwrap();
        }
        buf.len()
    });

    let mut tx_transfer = i2s_tx.write(tx_buffer).unwrap();

    loop {
        while tx_transfer.available_bytes() == 0 {
            tx_transfer.wait_for_available().await.unwrap();
        }
        tx_transfer.push_with(|buffer| {
            for b in buffer.iter_mut() {
                *b = samples.next().unwrap();
            }
            buffer.len()
        });
    }
}

fn enable_loopback() {
    let i2s = esp_hal::peripherals::I2S0::regs();
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2))] {
            i2s.conf().modify(|_, w| w.sig_loopback().set_bit());
            i2s.conf().modify(|_, w| w.rx_slave_mod().set_bit());
        } else {
            i2s.tx_conf().modify(|_, w| w.sig_loopback().set_bit());
            i2s.rx_conf().modify(|_, w| w.rx_slave_mod().set_bit());

            i2s.tx_conf().modify(|_, w| w.tx_update().set_bit());
            i2s.rx_conf().modify(|_, w| w.rx_update().set_bit());
        }
    }
}

#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use esp_hal::{dma_rx_stream_buffer, dma_tx_stream_buffer};

    use super::*;

    struct Context {
        dout: AnyPin<'static>,
        dma_channel: DmaChannel0<'static>,
        i2s: I2S0<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(
            esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()),
        );

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_I2S0;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let (_, dout) = hil_test::common_test_pins!(peripherals);

        Context {
            dout: dout.degrade(),
            dma_channel,
            i2s: peripherals.I2S0,
        }
    }

    #[test]
    async fn test_i2s_loopback_async(ctx: Context) {
        let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };

        // We need more than 3 descriptors for continuous transfer to work
        let rx_buffer = dma_rx_stream_buffer!(BUFFER_SIZE, BUFFER_SIZE / 3);
        let tx_buffer = dma_tx_stream_buffer!(BUFFER_SIZE, BUFFER_SIZE / 3);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap()
        .into_async();

        let (din, dout) = unsafe { ctx.dout.split() };

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        let i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(din)
            .build();

        enable_loopback();

        let mut rx_transfer = i2s_rx.read(rx_buffer, BUFFER_SIZE).unwrap();
        spawner.must_spawn(writer(tx_buffer, i2s_tx));

        let mut sample_idx = 0;
        let mut samples = SampleSource::new();
        for _ in 0..30 {
            while rx_transfer.available_bytes() == 0 {
                rx_transfer.wait_for_available().await.unwrap();
            }
            let data = rx_transfer.peek();
            let len = data.len();
            for &b in data {
                let expected = samples.next().unwrap();
                assert_eq!(
                    b, expected,
                    "Sample #{} does not match ({} != {})",
                    sample_idx, b, expected
                );
                sample_idx += 1;
            }
            rx_transfer.consume(len);
        }
    }

    #[test]
    fn test_i2s_loopback(ctx: Context) {
        // NOTE: 32000 bits of buffer maybe too large, but for some reason it fails with buffer
        // size 16000 as it seems DMA can be quick enough to run out of descriptors in that
        // case.
        let rx_buffer = dma_rx_stream_buffer!(32000, 4000);
        let mut tx_buffer = dma_tx_stream_buffer!(32000, 4000);

        let i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        let (din, dout) = unsafe { ctx.dout.split() };

        let i2s_tx = i2s
            .i2s_tx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_dout(dout)
            .build();

        let i2s_rx = i2s
            .i2s_rx
            .with_bclk(NoPin)
            .with_ws(NoPin)
            .with_din(din)
            .build();

        enable_loopback();

        let mut samples = SampleSource::new();
        tx_buffer.push_with(|buf| {
            for b in buf.iter_mut() {
                *b = samples.next().unwrap();
            }
            buf.len()
        });

        let mut rcv = [0u8; 11000];
        let mut filler = [0x1u8; 12000];

        let mut rx_transfer = i2s_rx.read(rx_buffer, 4000).unwrap();
        // trying to peek data before calling `available` should just do nothing
        assert_eq!(0, rx_transfer.peek().len());

        // no data available yet
        assert_eq!(0, rx_transfer.available_bytes());

        let mut tx_transfer = i2s_tx.write(tx_buffer).unwrap();

        let mut iteration = 0;
        let mut sample_idx = 0;
        let mut check_samples = SampleSource::new();
        loop {
            let tx_avail = tx_transfer.available_bytes();

            // make sure there are more than one descriptor buffers ready to push
            if tx_avail > 5000 {
                for b in &mut filler[0..tx_avail].iter_mut() {
                    *b = samples.next().unwrap();
                }
                tx_transfer.push(&filler[0..tx_avail]);
            }

            // test calling available multiple times doesn't break anything
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            rx_transfer.available_bytes();
            let rx_avail = rx_transfer.available_bytes();

            // make sure there are more than one descriptor buffers ready to pop
            if rx_avail > 0 {
                rcv.fill(0xff);
                let data = rx_transfer.peek();
                let len = data.len();
                assert!(len > 0);

                for &b in data {
                    let expected = check_samples.next().unwrap();
                    assert_eq!(
                        b, expected,
                        "Sample #{} does not match ({} != {})",
                        sample_idx, b, expected
                    );
                    sample_idx += 1;
                }

                rx_transfer.consume(len);

                iteration += 1;

                if iteration == 1 {
                    // delay to make it likely `available` will need to handle more than one
                    // descriptor next time
                    Delay::new().delay_millis(160);
                }
            }

            if iteration > 30 {
                break;
            }
        }
    }

    #[test]
    #[cfg(not(esp32s2))]
    fn test_i2s_rx_half_sample_bits_regression(ctx: Context) {
        // Regression test for rx_half_sample_bits configuration bug.
        // Validates that TX and RX half_sample_bits registers are configured identically.
        let _i2s = I2s::new(
            ctx.i2s,
            ctx.dma_channel,
            Config::new_tdm_philips()
                .with_sample_rate(Rate::from_hz(16000))
                .with_data_format(DataFormat::Data16Channel16)
                .with_channels(Channels::STEREO),
        )
        .unwrap();

        // Access registers directly to read back the configured values
        let regs = esp_hal::peripherals::I2S0::regs();
        let tx_conf1 = regs.tx_conf1().read();
        let rx_conf1 = regs.rx_conf1().read();

        let tx_half_sample_bits = tx_conf1.tx_half_sample_bits().bits();
        let rx_half_sample_bits = rx_conf1.rx_half_sample_bits().bits();

        // These MUST be identical - if they differ, the rx_half_sample_bits bug is present
        assert_eq!(
            tx_half_sample_bits, rx_half_sample_bits,
            "16-bit Stereo: half_sample_bits mismatch TX={}, RX={} (should be identical for proper timing)",
            tx_half_sample_bits, rx_half_sample_bits
        );

        // For Data16Channel16 + STEREO: (16 * 2) / 2 - 1 = 15
        let expected_value = 15u8;
        assert_eq!(
            tx_half_sample_bits, expected_value,
            "16-bit Stereo: TX half_sample_bits={}, expected {}",
            tx_half_sample_bits, expected_value
        );
        assert_eq!(
            rx_half_sample_bits, expected_value,
            "16-bit Stereo: RX half_sample_bits={}, expected {}",
            rx_half_sample_bits, expected_value
        );
    }
}
