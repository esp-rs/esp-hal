//! The C types and functions of ESP-SR 2.6.0 that this crate uses, from `esp_mn_iface.h`,
//! `esp_mn_models.h`, `model_path.h`, `flite_g2p.h` and `esp_mn_speech_commands.h`.

use core::ffi::{c_char, c_int, c_void};

/// `esp_err_t` values.
pub const ESP_OK: c_int = 0;
pub const ESP_ERR_INVALID_STATE: c_int = 0x103;

/// `esp_mn_state_t`.
pub const ESP_MN_STATE_DETECTED: c_int = 1;
pub const ESP_MN_STATE_TIMEOUT: c_int = 2;

/// `ESP_MN_MAX_PHRASE_NUM`.
pub const ESP_MN_MAX_PHRASE_NUM: usize = 400;

/// The model instance, `model_iface_data_t`, which only MultiNet looks into.
pub type Model = c_void;

/// `esp_mn_iface_t`, the functions of a MultiNet model.
#[repr(C)]
pub struct EspMnIface {
    pub create: unsafe extern "C" fn(name: *const c_char, duration: c_int) -> *mut Model,
    pub get_samp_rate: unsafe extern "C" fn(model: *mut Model) -> c_int,
    pub get_samp_chunksize: unsafe extern "C" fn(model: *mut Model) -> c_int,
    pub get_samp_chunknum: unsafe extern "C" fn(model: *mut Model) -> c_int,
    pub set_det_threshold: unsafe extern "C" fn(model: *mut Model, threshold: f32) -> c_int,
    pub get_language: unsafe extern "C" fn(model: *mut Model) -> *mut c_char,
    pub detect: unsafe extern "C" fn(model: *mut Model, samples: *mut i16) -> c_int,
    pub destroy: unsafe extern "C" fn(model: *mut Model),
    pub get_results: unsafe extern "C" fn(model: *mut Model) -> *mut EspMnResults,
    pub open_log: unsafe extern "C" fn(model: *mut Model),
    pub clean: unsafe extern "C" fn(model: *mut Model),
    pub set_speech_commands:
        unsafe extern "C" fn(model: *mut Model, root: *mut EspMnNode) -> *mut EspMnError,
    pub switch_loader_mode:
        Option<unsafe extern "C" fn(model: *mut Model, mode: c_int) -> *mut Model>,
    pub print_active_speech_commands: unsafe extern "C" fn(model: *mut Model),
    pub check_speech_command: unsafe extern "C" fn(model: *mut Model, text: *const c_char) -> c_int,
}

/// `esp_mn_results_t`.
#[repr(C)]
pub struct EspMnResults {
    pub state: c_int,
    pub num: c_int,
    pub command_id: [c_int; 5],
    pub phrase_id: [c_int; 5],
    pub prob: [f32; 5],
    pub string: [c_char; 256],
    pub raw_string: [c_char; 256],
}

/// `esp_mn_phrase_t`, a phrase and the command it belongs to.
#[repr(C)]
pub struct EspMnPhrase {
    pub string: *mut c_char,
    pub phonemes: *mut c_char,
    pub command_id: i16,
    pub threshold: f32,
    pub wave: *mut i16,
}

/// `esp_mn_node_t`, an element of the phrase list. The first node has no phrase.
#[repr(C)]
pub struct EspMnNode {
    pub phrase: *mut EspMnPhrase,
    pub next: *mut EspMnNode,
}

/// `esp_mn_error_t`, the phrases MultiNet did not accept.
#[repr(C)]
pub struct EspMnError {
    pub num: i16,
    pub phrases: *mut *mut EspMnPhrase,
}

/// `srmodel_list_t`, as ESP-SR builds it with `ESP_PLATFORM`.
#[repr(C)]
pub struct SrModelList {
    pub model_name: *const *const c_char,
    pub model_info: *const *const c_char,
    pub partition: *const c_void,
    pub mmap_handle: *const c_void,
    pub num: c_int,
    pub model_data: *const *const SrModelData,
}

/// `srmodel_data_t`, the files of a model.
#[repr(C)]
pub struct SrModelData {
    pub num: c_int,
    pub files: *const *const c_char,
    pub data: *const *const u8,
    pub sizes: *const c_int,
}

unsafe extern "C" {
    pub fn esp_mn_handle_from_name(name: *const c_char) -> *const EspMnIface;
    /// Converts English text to MultiNet7's phonemes, in memory from `esp_sr_malloc`.
    pub fn flite_g2p(graphemes: *const c_char, map_phonemes: c_int) -> *mut c_char;
}
