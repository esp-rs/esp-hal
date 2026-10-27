//! Recording from the two microphones over I2S.
//!
//! I2S runs all the time, because the ES8389 needs BCLK to stay out of standby. TX drives BCLK and
//! WS and only sends silence. RX receives frames of two 16-bit channels at 48 kHz, one per
//! microphone. When nothing is recorded, the frames are dropped.

use allocator_api2::vec::Vec;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
    signal::Signal,
};
use embassy_time::Duration;
use esp_alloc::ExternalMemory;
use esp_hal::{
    dma_rx_stream_buffer,
    dma_tx_stream_buffer,
    gpio::AnyPin,
    i2s::master::{Channels, DataFormat, I2s, TdmConfig},
    peripherals::{DMA_CH0, I2S0},
    time::Rate,
};

/// The frame rate on the I2S bus.
pub const I2S_SAMPLE_RATE: u32 = 48_000;
/// The rate of recorded speech, 1/3 of the I2S rate.
pub const RECORDING_SAMPLE_RATE: u32 = 16_000;
/// Recordings stop growing after this time.
pub const MAX_RECORDING: Duration = Duration::from_secs(30);

/// Mono 16-bit samples in PSRAM.
pub type Samples = Vec<i16, ExternalMemory>;

/// One left and one right 16-bit sample.
const FRAME_BYTES: usize = 4;

enum Command {
    Record,
    StopRecording,
}

static COMMANDS: Channel<CriticalSectionRawMutex, Command, 1> = Channel::new();
static RECORDING: Signal<CriticalSectionRawMutex, Samples> = Signal::new();

/// Starts recording mono speech at [`RECORDING_SAMPLE_RATE`].
pub async fn start_recording() {
    COMMANDS.send(Command::Record).await;
}

/// Stops the recording and returns its samples.
pub async fn stop_recording() -> Samples {
    COMMANDS.send(Command::StopRecording).await;
    RECORDING.wait().await
}

enum State {
    Idle,
    Recording(Decimator),
}

#[embassy_executor::task]
pub async fn run(
    i2s: I2S0<'static>,
    dma: DMA_CH0<'static>,
    bclk: AnyPin<'static>,
    ws: AnyPin<'static>,
    dout: AnyPin<'static>,
    din: AnyPin<'static>,
) {
    // RX shares BCLK and WS with TX. The 16-bit samples travel in 32-bit slots, because the codec
    // derives its clocks from BCLK and expects BCLK = 64 x the sample rate.
    let config = TdmConfig::new_tdm_philips()
        .with_signal_loopback(true)
        .with_sample_rate(Rate::from_hz(I2S_SAMPLE_RATE))
        .with_data_format(DataFormat::Data32Channel16)
        .with_channels(Channels::STEREO);
    let i2s = I2s::new(i2s, dma, config).unwrap().into_async();
    let tx = i2s
        .i2s_tx
        .with_bclk(bclk)
        .with_ws(ws)
        .with_dout(dout)
        .build();
    let microphones = i2s.i2s_rx.with_din(din).build();
    // About 170 ms each. Starting Wi-Fi blocks interrupts for about 80 ms (the PHY calibration, see
    // https://github.com/esp-rs/esp-hal/issues/6605), and the buffers must bridge that.
    let Ok(mut tx) = tx.write(dma_tx_stream_buffer!(32 * 1024, 2048)) else {
        panic!("Failed to start the I2S TX DMA");
    };
    let Ok(mut microphones) = microphones.read(dma_rx_stream_buffer!(32 * 1024, 2048)) else {
        panic!("Failed to start the microphone DMA");
    };

    let mut state = State::Idle;
    let mut frames = [0u8; 2048];
    loop {
        if let Err(error) = tx.wait_for_available_async().await {
            panic!("The I2S DMA stopped: {error:?}");
        }

        if let Ok(command) = COMMANDS.try_receive() {
            state = match (command, state) {
                (Command::Record, _) => State::Recording(Decimator::new()),
                (Command::StopRecording, State::Recording(decimator)) => {
                    RECORDING.signal(decimator.samples);
                    State::Idle
                }
                (Command::StopRecording, _) => {
                    RECORDING.signal(Vec::new_in(ExternalMemory));
                    State::Idle
                }
            };
        }

        loop {
            let len = microphones.available_bytes().min(frames.len());
            let len = len - len % FRAME_BYTES;
            if len == 0 {
                break;
            }
            let len = microphones.pop(&mut frames[..len]);
            if let State::Recording(decimator) = &mut state {
                decimator.push(&frames[..len]);
            }
        }

        // TX only sends silence.
        while tx.push_with(|out| {
            out.fill(0);
            out.len()
        }) > 0
        {}
    }
}

/// Mixes both microphones down to mono and averages every 3 frames (48 kHz -> 16 kHz).
struct Decimator {
    samples: Samples,
    sum: i32,
    count: usize,
}

impl Decimator {
    fn new() -> Self {
        let capacity = (MAX_RECORDING.as_secs() as u32 * RECORDING_SAMPLE_RATE) as usize;
        Self {
            samples: Vec::with_capacity_in(capacity, ExternalMemory),
            sum: 0,
            count: 0,
        }
    }

    fn push(&mut self, frames: &[u8]) {
        for &[left_low, left_high, right_low, right_high] in frames.as_chunks::<FRAME_BYTES>().0 {
            let left = i16::from_le_bytes([left_low, left_high]);
            let right = i16::from_le_bytes([right_low, right_high]);
            self.sum += i32::from(left) + i32::from(right);
            self.count += 1;
            if self.count == 3 {
                if self.samples.len() < self.samples.capacity() {
                    self.samples.push((self.sum / 6) as i16);
                }
                self.sum = 0;
                self.count = 0;
            }
        }
    }
}
