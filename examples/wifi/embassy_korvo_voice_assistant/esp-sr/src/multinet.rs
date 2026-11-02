//! The recognizer, on the chip only.

use alloc::{string::String, vec, vec::Vec};
use core::{
    ffi::{CStr, c_int},
    sync::atomic::{AtomicBool, Ordering},
};

use crate::{Error, Phrase, Recognition, SAMPLE_RATE, commands, models, sys};

/// The MultiNet command ID of the phrase at `index`. MultiNet7 keeps per-command values in arrays
/// at `command_id - 1`, so IDs start at 1, as in ESP-SR's default commands. ID 0 would write before
/// the arrays.
fn command_id(index: usize) -> c_int {
    index as c_int + 1
}

/// MultiNet gives up when it has not recognized a phrase this long after it started listening.
const TIMEOUT_MS: c_int = 6000;
/// Silence after the speech, so that MultiNet can finish a phrase at the very end.
const TRAILING_SILENCE_MS: u32 = 1000;

/// Whether a [`MultiNet`] exists. ESP-SR keeps its phrase list in globals.
static IN_USE: AtomicBool = AtomicBool::new(false);

/// The MultiNet7 recognizer with its phrases.
pub struct MultiNet {
    iface: &'static sys::EspMnIface,
    model: *mut sys::Model,
    chunk: Vec<i16>,
}

impl MultiNet {
    /// Loads the model and the phrases to recognize, which takes a few seconds.
    pub fn new<'a>(phrases: impl IntoIterator<Item = Phrase<'a>>) -> Result<Self, Error> {
        if IN_USE.swap(true, Ordering::Acquire) {
            return Err(Error::InUse);
        }
        enable_coprocessors();
        models::install();
        // SAFETY: The model is installed. On errors, `Drop` frees what was created.
        unsafe {
            let iface = &*sys::esp_mn_handle_from_name(models::MULTINET.as_ptr());
            let model = (iface.create)(models::MULTINET.as_ptr(), TIMEOUT_MS);
            if model.is_null() {
                IN_USE.store(false, Ordering::Release);
                return Err(Error::Model);
            }
            let chunk_len = (iface.get_samp_chunksize)(model) as usize;
            let multinet = Self {
                iface,
                model,
                chunk: vec![0; chunk_len],
            };
            assert_eq!((iface.get_samp_rate)(model) as u32, SAMPLE_RATE);

            if commands::esp_mn_commands_alloc(iface, model) != sys::ESP_OK {
                return Err(Error::Phrases);
            }
            for (index, phrase) in phrases.into_iter().enumerate() {
                let id = command_id(index);
                let rejected = |text: &CStr| Error::Phrase(String::from(text.to_string_lossy()));
                if commands::esp_mn_commands_add(id, phrase.text.as_ptr()) != sys::ESP_OK {
                    return Err(rejected(phrase.text));
                }
                for phonemes in phrase.pronunciations {
                    // ESP-SR ignores a phrase whose text it already has, so the phonemes stand in
                    // for the text.
                    let ptr = phonemes.as_ptr();
                    if commands::esp_mn_commands_phoneme_add(id, ptr, ptr) != sys::ESP_OK {
                        return Err(rejected(phonemes));
                    }
                }
            }
            if !commands::esp_mn_commands_update().is_null() {
                return Err(Error::Phrases);
            }
            Ok(multinet)
        }
    }

    /// Recognizes a phrase in 16 kHz mono speech.
    pub fn recognize(&mut self, speech: &[i16]) -> Recognition {
        enable_coprocessors();
        let silence = (SAMPLE_RATE * TRAILING_SILENCE_MS / 1000) as usize;
        let silent_chunks = silence.div_ceil(self.chunk.len());
        let chunks = speech
            .chunks(self.chunk.len())
            .chain(core::iter::repeat_n(&[][..], silent_chunks));

        // SAFETY: The model is valid, and every chunk has the size MultiNet asked for.
        unsafe {
            (self.iface.clean)(self.model);
            for samples in chunks {
                self.chunk[..samples.len()].copy_from_slice(samples);
                self.chunk[samples.len()..].fill(0);
                match (self.iface.detect)(self.model, self.chunk.as_mut_ptr()) {
                    sys::ESP_MN_STATE_DETECTED => break,
                    sys::ESP_MN_STATE_TIMEOUT => break,
                    _ => {}
                }
            }
            let results = &*(self.iface.get_results)(self.model);
            // `c_char` is `u8` on RISC-V.
            let text = |text: &[u8]| {
                let bytes = CStr::from_bytes_until_nul(text).unwrap_or_default();
                String::from(bytes.to_string_lossy().trim())
            };
            if results.state == sys::ESP_MN_STATE_DETECTED && results.num > 0 {
                Recognition::Phrase {
                    index: results.command_id[0] as usize - 1,
                    probability: results.prob[0],
                    heard: text(&results.string),
                }
            } else {
                Recognition::Nothing {
                    heard: text(&results.raw_string),
                }
            }
        }
    }
}

impl Drop for MultiNet {
    fn drop(&mut self) {
        // SAFETY: The model is valid and not used again. MultiNet's `destroy` also frees the phrase
        // list, which it still reads until then.
        unsafe { (self.iface.destroy)(self.model) };
        IN_USE.store(false, Ordering::Release);
    }
}

/// Turns on the hardware loops (CSR 0x7F1) and the vector unit (PIE, CSR 0x7F2) of this core,
/// which MultiNet uses. ESP-IDF's `rv_utils_enable_pie` and its hardware loop setup do the same.
fn enable_coprocessors() {
    // SAFETY: Writing 1 puts both units into their initial state, which is harmless between calls
    // into MultiNet. Nothing else on the core uses them.
    unsafe { core::arch::asm!("csrwi 0x7F1, 1", "csrwi 0x7F2, 1") };
}
