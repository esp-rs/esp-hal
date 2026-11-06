//! Speech recognition on the second core with MultiNet7 from Espressif's ESP-SR, through the
//! `esp-sr` crate.
//!
//! MultiNet does not transcribe arbitrary speech: it recognizes a fixed list of English phrases.
//! It uses the vector unit and the hardware loops of its core, which esp-rtos does not save when
//! it switches threads, and recognizing takes about a second. So it runs alone on the second core.

use alloc::vec::Vec;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::{delay::Delay, time::Instant};
use esp_sr::{MultiNet, Phrase, Recognition};

use crate::audio::{RECORDING_SAMPLE_RATE, Samples};

const _: () = assert!(RECORDING_SAMPLE_RATE == esp_sr::SAMPLE_RATE);

/// The peak of a recording after [`gain`], about -4 dBFS. MultiNet recognizes quiet speech
/// poorly, and the microphones pick up speech at about -25 dBFS from half a meter.
const TARGET_PEAK: i32 = 20_000;
/// The highest gain, +18 dB, so that a recording without speech does not become loud noise.
const MAX_GAIN: i32 = 8;

static REQUEST: Signal<CriticalSectionRawMutex, Samples> = Signal::new();
static RESULT: Signal<CriticalSectionRawMutex, Option<usize>> = Signal::new();
static READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Waits until the recognizer has loaded its model and phrases.
pub async fn wait_until_ready() {
    READY.wait().await
}

/// Returns the index of the phrase in 16 kHz mono speech, if any.
pub async fn recognize(speech: Samples) -> Option<usize> {
    REQUEST.signal(speech);
    RESULT.wait().await
}

/// Runs the recognizer of `phrases`. This is the entry point of the second core.
pub fn run(phrases: Vec<Phrase<'static>>) -> ! {
    let mut multinet = match MultiNet::new(phrases) {
        Ok(multinet) => multinet,
        Err(error) => panic!("Failed to start MultiNet: {error:?}"),
    };
    log::info!("Speech recognizer ready");
    READY.signal(());
    let delay = Delay::new();
    loop {
        let Some(mut speech) = REQUEST.try_take() else {
            delay.delay_millis(10);
            continue;
        };
        let gain = gain(&speech);
        for sample in speech.iter_mut() {
            *sample = (i32::from(*sample) * gain).clamp(i16::MIN.into(), i16::MAX.into()) as i16;
        }

        let start = Instant::now();
        let index = match multinet.recognize(&speech) {
            Recognition::Phrase {
                index,
                probability,
                heard,
            } => {
                log::info!(
                    "MultiNet: {heard:?} ({probability}) at gain {gain} in {}",
                    start.elapsed()
                );
                Some(index)
            }
            Recognition::Nothing { heard } => {
                // The phonemes help to find extra pronunciations of a phrase.
                log::info!(
                    "MultiNet: nothing recognized at gain {gain} in {}, heard {heard:?}",
                    start.elapsed()
                );
                None
            }
        };
        RESULT.signal(index);
    }
}

/// The gain that brings the peak of the speech to [`TARGET_PEAK`], at most [`MAX_GAIN`].
fn gain(speech: &[i16]) -> i32 {
    let peak = speech
        .iter()
        .map(|sample| i32::from(*sample).abs())
        .max()
        .unwrap_or(0);
    (TARGET_PEAK / peak.max(1)).clamp(1, MAX_GAIN)
}
