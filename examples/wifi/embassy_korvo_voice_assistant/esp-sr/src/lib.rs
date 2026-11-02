//! Offline speech command recognition with MultiNet7 from Espressif's ESP-SR, for esp-hal.
//!
//! MultiNet does not transcribe arbitrary speech: it recognizes which one of a list of English
//! phrases, such as "turn the light red", was said. It runs entirely on the chip.
//!
//! The recognizer itself is Espressif's prebuilt code, linked from the libraries in `lib/` with the
//! model in `model/`, both from ESP-SR 2.6.0 under the license in `LICENSE-ESP-SR`, which allows
//! use on Espressif chips. `tools/vendor.py` prepares them, see its documentation. This crate
//! provides everything else they need in Rust: the C library functions they import, ESP-SR's phrase
//! list, the cJSON and FFT parts of its dependencies, and the model, embedded in the firmware.
//! Building needs neither a C compiler nor GNU ld.
//!
//! # Requirements
//!
//! - An ESP32-S31 with PSRAM, which esp-alloc manages: MultiNet allocates about 3 MiB there.
//! - About 4 MiB of flash for the libraries and the model.
//! - MultiNet uses the vector unit (PIE) and the hardware loops of the core it runs on, which
//!   esp-rtos does not save when it switches threads. Run it where no other code uses them, for
//!   example alone on the second core. Recognizing a few seconds of speech takes about a second.
//! - One [`MultiNet`] at a time, used from one core.

#![no_std]

extern crate alloc;

use alloc::string::String;
use core::ffi::CStr;

mod cjson;
#[cfg(target_arch = "riscv32")]
mod commands;
mod dl;
mod libc;
mod memory;
#[cfg(target_arch = "riscv32")]
mod models;
#[cfg(target_arch = "riscv32")]
mod multinet;
mod printf;
#[cfg(target_arch = "riscv32")]
mod sys;

#[cfg(target_arch = "riscv32")]
pub use multinet::MultiNet;

/// The sample rate of the speech, 16-bit mono.
pub const SAMPLE_RATE: u32 = 16_000;

/// A phrase to recognize.
#[derive(Clone, Copy)]
pub struct Phrase<'a> {
    /// The phrase in plain English words, at most 63 characters, such as "turn the light red".
    /// ESP-SR converts it to phonemes.
    pub text: &'a CStr,
    /// Extra pronunciations in ESP-SR's phonemes, for when MultiNet hears the phrase differently
    /// than ESP-SR derives from the text. For example, the t of "light" often runs into the next
    /// word: "turn the light red" is `TkN jc LiT RfD`, but MultiNet can hear `TkN jc Li Rf`.
    /// [`Recognition`] reports what MultiNet heard.
    pub pronunciations: &'a [&'a CStr],
}

impl<'a> Phrase<'a> {
    /// A phrase without extra pronunciations.
    pub const fn new(text: &'a CStr) -> Self {
        Self {
            text,
            pronunciations: &[],
        }
    }
}

/// What MultiNet made of speech.
#[derive(Debug)]
pub enum Recognition {
    /// It recognized the phrase at `index` in the list [`MultiNet::new`] got.
    Phrase {
        index: usize,
        /// MultiNet's confidence. Recognized phrases usually have 0.1 to 0.4.
        probability: f32,
        /// The phonemes it heard.
        heard: String,
    },
    /// It recognized none of the phrases.
    Nothing {
        /// The phonemes it heard. A phrase that fails often can get these as an extra
        /// pronunciation.
        heard: String,
    },
}

#[derive(Debug)]
pub enum Error {
    /// MultiNet could not load its model, usually for lack of PSRAM.
    Model,
    /// MultiNet rejected the phrase or pronunciation, for example for words it cannot spell.
    Phrase(String),
    /// MultiNet rejected the phrase list.
    Phrases,
    /// Only one [`MultiNet`] can exist at a time.
    InUse,
}
