//! Full-duplex audio over I2S.
//!
//! I2S runs all the time, because the ES8389 needs BCLK to stay out of standby. Each frame holds
//! two 16-bit channels at 48 kHz: the two microphones on the way in, the two speakers on the way
//! out. When nothing is recorded, the microphone frames are dropped. When nothing is played, the
//! speakers get silence.

use allocator_api2::vec::Vec;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
    signal::Signal,
};
use embassy_time::Duration;
use esp_alloc::ExternalMemory;
use esp_hal::{
    Async,
    dma::{DmaRxStreamBuf, DmaTxStreamBuf},
    i2s::master::{I2sRxDmaTransfer, I2sTxDmaTransfer},
};

/// The frame rate on the I2S bus.
pub const I2S_SAMPLE_RATE: u32 = 48_000;
/// The rate of recorded speech, 1/3 of the I2S rate.
pub const RECORDING_SAMPLE_RATE: u32 = 16_000;
/// The rate of played speech, 1/2 of the I2S rate.
pub const PLAYBACK_SAMPLE_RATE: u32 = 24_000;
/// Recordings stop growing after this time.
pub const MAX_RECORDING: Duration = Duration::from_secs(30);

/// Mono 16-bit samples in PSRAM.
pub type Samples = Vec<i16, ExternalMemory>;

/// One left and one right 16-bit sample.
const FRAME_BYTES: usize = 4;

enum Command {
    Record,
    StopRecording,
    Play(Samples),
}

static COMMANDS: Channel<CriticalSectionRawMutex, Command, 1> = Channel::new();
static RECORDING: Signal<CriticalSectionRawMutex, Samples> = Signal::new();
static PLAYED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Starts recording mono speech at [`RECORDING_SAMPLE_RATE`].
pub async fn start_recording() {
    COMMANDS.send(Command::Record).await;
}

/// Stops the recording and returns its samples.
pub async fn stop_recording() -> Samples {
    COMMANDS.send(Command::StopRecording).await;
    RECORDING.wait().await
}

/// Plays mono speech at [`PLAYBACK_SAMPLE_RATE`] on both speakers and waits until it is done.
pub async fn play(samples: Samples) {
    COMMANDS.send(Command::Play(samples)).await;
    PLAYED.wait().await;
}

enum State {
    Idle,
    Recording(Decimator),
    Playing(Interpolator),
}

#[embassy_executor::task]
pub async fn run(
    mut speakers: I2sTxDmaTransfer<'static, Async, DmaTxStreamBuf>,
    mut microphones: I2sRxDmaTransfer<'static, Async, DmaRxStreamBuf>,
) {
    let mut state = State::Idle;
    let mut frames = [0u8; 2048];

    loop {
        speakers.wait_for_available_async().await.unwrap();

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
                (Command::Play(samples), _) => State::Playing(Interpolator::new(samples)),
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

        loop {
            let pushed = speakers.push_with(|out| {
                let (frames, _) = out.as_chunks_mut::<FRAME_BYTES>();
                for frame in frames.iter_mut() {
                    let sample = match &mut state {
                        State::Playing(interpolator) => interpolator.next().unwrap_or(0),
                        _ => 0,
                    };
                    let [low, high] = sample.to_le_bytes();
                    *frame = [low, high, low, high];
                }
                frames.len() * FRAME_BYTES
            });
            if pushed == 0 {
                break;
            }
        }

        if let State::Playing(interpolator) = &state
            && interpolator.is_done()
        {
            PLAYED.signal(());
            state = State::Idle;
        }
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

/// Inserts the midpoint between consecutive samples (24 kHz -> 48 kHz).
struct Interpolator {
    samples: Samples,
    position: usize,
    previous: i16,
    midpoint_sent: bool,
}

impl Interpolator {
    fn new(samples: Samples) -> Self {
        Self {
            samples,
            position: 0,
            previous: 0,
            midpoint_sent: false,
        }
    }

    fn is_done(&self) -> bool {
        self.position == self.samples.len()
    }
}

impl Iterator for Interpolator {
    type Item = i16;

    fn next(&mut self) -> Option<i16> {
        let sample = *self.samples.get(self.position)?;
        if !self.midpoint_sent {
            self.midpoint_sent = true;
            return Some(((i32::from(self.previous) + i32::from(sample)) / 2) as i16);
        }
        self.midpoint_sent = false;
        self.previous = sample;
        self.position += 1;
        Some(sample)
    }
}
