//! Speech recognition on the second core with MultiNet7 from Espressif's ESP-SR.
//!
//! MultiNet does not transcribe arbitrary speech: it recognizes the fixed English phrases in
//! [`QUESTIONS`]. It runs on the second core, which nothing else uses, because the prebuilt ESP-SR
//! libraries use picolibc, and picolibc keeps `errno` and similar state in thread-local storage
//! that the `tp` register points to. On the first core, esp-rtos keeps the current task in `tp`.

use alloc::{boxed::Box, ffi::CString, vec, vec::Vec};
use core::{
    ffi::{CStr, c_char, c_int, c_void},
    ptr,
    sync::atomic::{AtomicPtr, Ordering},
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::delay::Delay;

use crate::audio::{RECORDING_SAMPLE_RATE, Samples};

/// The phrases MultiNet recognizes. ESP-SR converts them to phonemes, so they must be plain
/// English words. A phrase has at most 63 characters.
pub const QUESTIONS: &[&CStr] = &[
    c"tell me a joke",
    c"tell me a fun fact",
    c"tell me a short story",
    c"say something nice",
    c"who are you",
    c"what can you do",
    c"why is the sky blue",
    c"how do airplanes fly",
    c"how far away is the moon",
    c"how hot is the sun",
    c"what is the speed of light",
    c"what is the tallest mountain",
    c"what is the capital of france",
    c"who invented the telephone",
    c"how many people live on earth",
    c"recommend a good book",
    c"give me a recipe for pancakes",
    c"what is a microcontroller",
];

/// The model name, which is also the directory of the model files in ESP-SR.
const MODEL: &CStr = c"mn7_en";
/// MultiNet gives up when it has not recognized a phrase this long after it started listening.
const TIMEOUT_MS: c_int = 6000;
/// Silence after the recording, so that MultiNet can finish a phrase at the very end.
const TRAILING_SILENCE_MS: u32 = 1000;

static REQUEST: Signal<CriticalSectionRawMutex, Samples> = Signal::new();
static RESULT: Signal<CriticalSectionRawMutex, Option<&'static str>> = Signal::new();

/// Returns the phrase from [`QUESTIONS`] in 16 kHz mono speech, if any.
pub async fn recognize(speech: Samples) -> Option<&'static str> {
    REQUEST.signal(speech);
    RESULT.wait().await
}

/// Runs the recognizer. This is the entry point of the second core.
pub fn run() {
    // esp-rtos's interrupt entry code reads `tp`. Nothing on this core needs interrupts, except
    // esp-hal's flash driver to park it during flash writes, which this example does not do.
    // SAFETY: Clears MIE in `mstatus`.
    unsafe { core::arch::asm!("csrci mstatus, 8") };
    enable_pie();
    let tls = Tls::new();

    let mut recognizer = tls.run(Recognizer::new);
    log::info!("Speech recognizer ready");
    let delay = Delay::new();
    loop {
        match REQUEST.try_take() {
            Some(speech) => {
                let phrase = tls.run(|| recognizer.recognize(&speech));
                RESULT.signal(phrase.map(|index| QUESTIONS[index].to_str().unwrap()));
            }
            None => delay.delay_millis(10),
        }
    }
}

/// Turns on the hardware loops (CSR 0x7F1) and the SIMD instructions (CSR 0x7F2) of this core,
/// which the ESP-SR libraries use. ESP-IDF's `rv_utils_enable_pie` does the same.
fn enable_pie() {
    // SAFETY: Writing 1 moves both units from off to the initial state.
    unsafe { core::arch::asm!("csrwi 0x7F1, 1", "csrwi 0x7F2, 1") };
}

/// A copy of the template of picolibc's thread-local variables, see `picolibc_tls.x`. The linker
/// addresses each variable relative to the start of the template.
struct Tls {
    block: Box<[u128]>,
}

impl Tls {
    fn new() -> Self {
        unsafe extern "C" {
            static _tdata_start: u8;
            static _tdata_end: u8;
            static _tbss_end: u8;
        }
        let start = &raw const _tdata_start;
        let data_len = (&raw const _tdata_end).addr() - start.addr();
        let len = (&raw const _tbss_end).addr() - start.addr();

        // `u128` aligns the block to 16 bytes, more than any variable needs.
        let mut block = vec![0u128; len.div_ceil(16)].into_boxed_slice();
        // SAFETY: The block has `len` bytes, and `.tdata` is the first `data_len` bytes of the
        // template.
        unsafe { ptr::copy_nonoverlapping(start, block.as_mut_ptr().cast(), data_len) };
        Self { block }
    }

    /// Runs `f` with `tp` pointing to the thread-local storage.
    fn run<R>(&self, f: impl FnOnce() -> R) -> R {
        let previous: *const u8;
        // SAFETY: Only C code reads `tp` while it points to the block. Interrupts are off.
        unsafe {
            core::arch::asm!("mv {0}, tp", "mv tp, {1}", out(reg) previous, in(reg) self.block.as_ptr())
        };
        let result = f();
        unsafe { core::arch::asm!("mv tp, {0}", in(reg) previous) };
        result
    }
}

struct Recognizer {
    multinet: &'static MultiNet,
    model: *mut c_void,
    chunk: Vec<i16>,
}

impl Recognizer {
    fn new() -> Self {
        install_models();
        // SAFETY: The model is installed, and `QUESTIONS` are NUL-terminated.
        unsafe {
            let multinet = &*esp_mn_handle_from_name(MODEL.as_ptr());
            let model = (multinet.create)(MODEL.as_ptr(), TIMEOUT_MS);
            assert!(!model.is_null(), "Failed to create MultiNet");
            let rate = (multinet.get_samp_rate)(model);
            assert_eq!(rate as u32, RECORDING_SAMPLE_RATE, "MultiNet sample rate");

            assert_eq!(esp_mn_commands_alloc(multinet, model), 0);
            for (id, question) in QUESTIONS.iter().enumerate() {
                let result = esp_mn_commands_add(id as c_int, question.as_ptr());
                assert_eq!(result, 0, "MultiNet rejected {question:?}");
            }
            assert!(
                esp_mn_commands_update().is_null(),
                "Failed to set the phrases"
            );

            let chunk_len = (multinet.get_samp_chunksize)(model) as usize;
            Self {
                multinet,
                model,
                chunk: vec![0; chunk_len],
            }
        }
    }

    /// Returns the index in [`QUESTIONS`] of the phrase in the speech.
    fn recognize(&mut self, speech: &[i16]) -> Option<usize> {
        let silence = (RECORDING_SAMPLE_RATE * TRAILING_SILENCE_MS / 1000) as usize;
        let silent_chunks = silence.div_ceil(self.chunk.len());
        let chunks = speech
            .chunks(self.chunk.len())
            .chain(core::iter::repeat_n(&[][..], silent_chunks));

        // SAFETY: The model is valid, and every chunk has the size MultiNet asked for.
        unsafe { (self.multinet.clean)(self.model) };
        for samples in chunks {
            self.chunk[..samples.len()].copy_from_slice(samples);
            self.chunk[samples.len()..].fill(0);
            match unsafe { (self.multinet.detect)(self.model, self.chunk.as_mut_ptr()) } {
                DETECTED => {
                    let results = unsafe { &*(self.multinet.get_results)(self.model) };
                    log::info!("MultiNet: {:?} ({})", results.phrase(), results.prob[0]);
                    return (results.num > 0).then_some(results.command_id[0] as usize);
                }
                TIMEOUT => return None,
                _ => {}
            }
        }
        None
    }
}

/// The model files, which MultiNet looks up by model and file name.
struct Model {
    name: &'static CStr,
    files: &'static [(&'static CStr, &'static [u8])],
}

macro_rules! model_file {
    ($path:literal) => {
        include_bytes!(concat!(
            env!("ESP_SR_COMPONENTS"),
            "/espressif__esp-sr/model/multinet_model/",
            $path
        ))
    };
}

const MODELS: [Model; 2] = [
    Model {
        name: MODEL,
        files: &[
            (c"_MODEL_INFO_", model_file!("mn7_en/_MODEL_INFO_")),
            (c"mn7_data", model_file!("mn7_en/mn7_data")),
            (c"mn7_index", model_file!("mn7_en/mn7_index")),
            (c"vocab", model_file!("mn7_en/vocab")),
        ],
    },
    // The default phrases, which MultiNet loads before `QUESTIONS` replace them.
    Model {
        name: c"fst",
        files: &[(c"commands_en.txt", model_file!("fst/commands_en.txt"))],
    },
];

/// `srmodel_list_t` from ESP-SR's `model_path.h`, built with `ESP_PLATFORM`.
#[repr(C)]
struct ModelList {
    model_name: *const *const c_char,
    model_info: *const *const c_char,
    partition: *const c_void,
    mmap_handle: *const c_void,
    num: c_int,
    model_data: *const *const ModelFiles,
}

/// `srmodel_data_t` from ESP-SR's `model_path.h`.
#[repr(C)]
struct ModelFiles {
    num: c_int,
    files: *const *const c_char,
    data: *const *const u8,
    sizes: *const c_int,
}

static MODEL_LIST: AtomicPtr<ModelList> = AtomicPtr::new(ptr::null_mut());

/// Builds the model list that MultiNet gets from [`get_static_srmodels`]. ESP-SR's `srmodel_load`
/// builds the same list from a flash partition.
fn install_models() {
    fn leak<T>(items: impl IntoIterator<Item = T>) -> *const T {
        Vec::leak(items.into_iter().collect()).as_ptr()
    }

    let files = MODELS.map(|model| {
        let files = ModelFiles {
            num: model.files.len() as c_int,
            files: leak(model.files.iter().map(|(name, _)| name.as_ptr())),
            data: leak(model.files.iter().map(|(_, data)| data.as_ptr())),
            sizes: leak(model.files.iter().map(|(_, data)| data.len() as c_int)),
        };
        Box::into_raw(Box::new(files)).cast_const()
    });
    let list = ModelList {
        model_name: leak(MODELS.iter().map(|model| model.name.as_ptr())),
        model_info: leak(MODELS.iter().map(|model| {
            model_info(model).map_or(ptr::null(), |info| info.into_raw().cast_const())
        })),
        partition: ptr::null(),
        mmap_handle: ptr::null(),
        num: MODELS.len() as c_int,
        model_data: leak(files),
    };
    MODEL_LIST.store(Box::into_raw(Box::new(list)), Ordering::Release);
}

/// The line of the `_MODEL_INFO_` file that is not a comment, such as `MN7_v2_english_8_0.9_0.90`.
/// ESP-SR's `get_model_info` reads it the same way.
fn model_info(model: &Model) -> Option<CString> {
    let (_, info) = model
        .files
        .iter()
        .find(|(name, _)| *name == c"_MODEL_INFO_")?;
    let info = info
        .split(|&byte| byte == b'\n')
        .find(|line| !line.is_empty() && !line.starts_with(b"#"))?;
    Some(CString::new(info).unwrap())
}

#[unsafe(no_mangle)]
extern "C" fn get_static_srmodels() -> *mut ModelList {
    MODEL_LIST.load(Ordering::Acquire)
}

/// Without a base path, MultiNet reads the models from [`get_static_srmodels`].
#[unsafe(no_mangle)]
extern "C" fn get_model_base_path() -> *const c_char {
    ptr::null()
}

const DETECTED: c_int = 1;
const TIMEOUT: c_int = 2;

/// `esp_mn_iface_t` from ESP-SR's `esp_mn_iface.h`.
#[repr(C)]
struct MultiNet {
    create: unsafe extern "C" fn(*const c_char, c_int) -> *mut c_void,
    get_samp_rate: unsafe extern "C" fn(*mut c_void) -> c_int,
    get_samp_chunksize: unsafe extern "C" fn(*mut c_void) -> c_int,
    get_samp_chunknum: *const c_void,
    set_det_threshold: *const c_void,
    get_language: *const c_void,
    detect: unsafe extern "C" fn(*mut c_void, *mut i16) -> c_int,
    destroy: *const c_void,
    get_results: unsafe extern "C" fn(*mut c_void) -> *const Results,
    open_log: *const c_void,
    clean: unsafe extern "C" fn(*mut c_void),
    set_speech_commands: *const c_void,
    switch_loader_mode: *const c_void,
    print_active_speech_commands: *const c_void,
    check_speech_command: *const c_void,
}

/// `esp_mn_results_t` from ESP-SR's `esp_mn_iface.h`.
#[repr(C)]
struct Results {
    state: c_int,
    num: c_int,
    command_id: [c_int; 5],
    phrase_id: [c_int; 5],
    prob: [f32; 5],
    string: [c_char; 256],
    raw_string: [c_char; 256],
}

impl Results {
    fn phrase(&self) -> &CStr {
        CStr::from_bytes_until_nul(&self.string).unwrap_or_default()
    }
}

unsafe extern "C" {
    fn esp_mn_handle_from_name(name: *const c_char) -> *const MultiNet;
    fn esp_mn_commands_alloc(multinet: *const MultiNet, model: *mut c_void) -> c_int;
    fn esp_mn_commands_add(id: c_int, phrase: *const c_char) -> c_int;
    fn esp_mn_commands_update() -> *const c_void;
}
