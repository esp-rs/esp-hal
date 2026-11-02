//! The phrase list of MultiNet, a port of ESP-SR's `esp_mn_speech_commands.c` as built for
//! MultiNet7 (`CONFIG_SR_MN_EN_MULTINET7_QUANT`): phrases are checked as phonemes.
//!
//! MultiNet calls `esp_mn_commands_alloc`, `esp_mn_commands_add`, `esp_mn_commands_phoneme_add`,
//! `esp_mn_commands_get_from_string` and `esp_mn_commands_free` while it loads its default phrases.

use core::{
    ffi::{CStr, c_char, c_int},
    ptr,
    sync::atomic::{AtomicPtr, Ordering},
};

use crate::{
    memory::{boxed, deallocate, duplicate},
    sys::{
        ESP_ERR_INVALID_STATE,
        ESP_MN_MAX_PHRASE_NUM,
        ESP_OK,
        EspMnError,
        EspMnIface,
        EspMnNode,
        EspMnPhrase,
        Model,
        flite_g2p,
    },
};

/// The first node of the list, without a phrase, and the model the list belongs to.
static ROOT: AtomicPtr<EspMnNode> = AtomicPtr::new(ptr::null_mut());
static IFACE: AtomicPtr<EspMnIface> = AtomicPtr::new(ptr::null_mut());
static MODEL: AtomicPtr<Model> = AtomicPtr::new(ptr::null_mut());

/// The list, if a model created it.
fn list() -> Option<(&'static EspMnIface, *mut Model, *mut EspMnNode)> {
    let (root, iface, model) = (
        ROOT.load(Ordering::Acquire),
        IFACE.load(Ordering::Acquire),
        MODEL.load(Ordering::Acquire),
    );
    if root.is_null() || iface.is_null() || model.is_null() {
        log::error!("Create the MultiNet model first");
        return None;
    }
    // SAFETY: `esp_mn_commands_alloc` stored a MultiNet interface, which is static.
    Some((unsafe { &*iface }, model, root))
}

/// The phrases of the list after the root.
///
/// # Safety
///
/// The list must not change while the iterator lives.
unsafe fn nodes(root: *mut EspMnNode) -> impl Iterator<Item = *mut EspMnNode> {
    // SAFETY: The nodes of the list are valid until they are freed.
    let first = unsafe { (*root).next };
    core::iter::successors((!first.is_null()).then_some(first), |&node| {
        let next = unsafe { (*node).next };
        (!next.is_null()).then_some(next)
    })
}

/// # Safety
///
/// `text` must be NUL-terminated.
unsafe fn search(root: *mut EspMnNode, text: *const c_char) -> *mut EspMnNode {
    // SAFETY: The phrases and `text` are NUL-terminated.
    unsafe {
        let text = CStr::from_ptr(text);
        nodes(root)
            .find(|&node| CStr::from_ptr((*(*node).phrase).string) == text)
            .unwrap_or(ptr::null_mut())
    }
}

fn phrase_alloc(command_id: c_int, text: &CStr) -> *mut EspMnPhrase {
    if text.is_empty() {
        log::error!("The phrase is empty");
        return ptr::null_mut();
    }
    let string = duplicate(text.to_bytes());
    if string.is_null() {
        return ptr::null_mut();
    }
    let phrase = boxed(EspMnPhrase {
        string,
        phonemes: ptr::null_mut(),
        command_id: command_id as i16,
        threshold: 0.0,
        wave: ptr::null_mut(),
    });
    if phrase.is_null() {
        // SAFETY: `string` is a new allocation.
        unsafe { deallocate(string.cast()) };
    }
    phrase
}

/// # Safety
///
/// `phrase` must be null or come from `phrase_alloc`.
unsafe fn phrase_free(phrase: *mut EspMnPhrase) {
    if phrase.is_null() {
        return;
    }
    // SAFETY: The fields are null or allocations of this module or of the ESP-SR libraries, which
    // use the same memory.
    unsafe {
        deallocate((*phrase).wave.cast());
        deallocate((*phrase).string.cast());
        deallocate((*phrase).phonemes.cast());
        deallocate(phrase.cast());
    }
}

/// # Safety
///
/// `node` must be null or a node of the list.
unsafe fn node_free(node: *mut EspMnNode) {
    if !node.is_null() {
        unsafe {
            phrase_free((*node).phrase);
            deallocate(node.cast());
        }
    }
}

/// Appends `phrase` with `phonemes`, a copy of which the phrase keeps.
///
/// # Safety
///
/// `root` must be the root of the list, `phonemes` NUL-terminated or null.
unsafe fn append(root: *mut EspMnNode, phrase: *mut EspMnPhrase, phonemes: *const c_char) -> c_int {
    unsafe {
        if !phonemes.is_null() {
            (*phrase).phonemes = duplicate(CStr::from_ptr(phonemes).to_bytes());
        }
        let node = boxed(EspMnNode {
            phrase,
            next: ptr::null_mut(),
        });
        if node.is_null() {
            phrase_free(phrase);
            return ESP_ERR_INVALID_STATE;
        }
        let mut last = root;
        while !(*last).next.is_null() {
            last = (*last).next;
        }
        (*last).next = node;
    }
    ESP_OK
}

/// Updates the command of an existing phrase. Returns whether the phrase exists.
///
/// # Safety
///
/// `root` must be the root of the list, `text` NUL-terminated.
unsafe fn update_existing(root: *mut EspMnNode, command_id: c_int, text: *const c_char) -> bool {
    unsafe {
        let node = search(root, text);
        if node.is_null() {
            return false;
        }
        let phrase = &mut *(*node).phrase;
        if i32::from(phrase.command_id) == command_id {
            log::info!(
                "command {command_id}: ({:?}) already exists.",
                CStr::from_ptr(text)
            );
        } else {
            phrase.command_id = command_id as i16;
        }
        true
    }
}

/// Starts an empty list for `model`.
#[unsafe(no_mangle)]
pub(crate) unsafe extern "C" fn esp_mn_commands_alloc(
    iface: *const EspMnIface,
    model: *mut Model,
) -> c_int {
    if !ROOT.load(Ordering::Acquire).is_null() {
        unsafe { esp_mn_commands_free() };
    }
    let root = boxed(EspMnNode {
        phrase: ptr::null_mut(),
        next: ptr::null_mut(),
    });
    if root.is_null() {
        return ESP_ERR_INVALID_STATE;
    }
    IFACE.store(iface.cast_mut(), Ordering::Release);
    MODEL.store(model, Ordering::Release);
    ROOT.store(root, Ordering::Release);
    ESP_OK
}

#[unsafe(no_mangle)]
pub(crate) unsafe extern "C" fn esp_mn_commands_free() -> c_int {
    unsafe { esp_mn_commands_clear() };
    let root = ROOT.swap(ptr::null_mut(), Ordering::AcqRel);
    // SAFETY: The root has no phrase.
    unsafe { node_free(root) };
    IFACE.store(ptr::null_mut(), Ordering::Release);
    MODEL.store(ptr::null_mut(), Ordering::Release);
    ESP_OK
}

/// Removes all phrases.
#[unsafe(no_mangle)]
unsafe extern "C" fn esp_mn_commands_clear() -> c_int {
    let root = ROOT.load(Ordering::Acquire);
    if root.is_null() {
        log::error!("The MultiNet phrases are not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    // SAFETY: The nodes belong to the list, which is not used during this.
    unsafe {
        let mut node = (*root).next;
        while !node.is_null() {
            let next = (*node).next;
            node_free(node);
            node = next;
        }
        (*root).next = ptr::null_mut();
    }
    ESP_OK
}

/// Adds a phrase in English text, which MultiNet7 checks as phonemes.
#[unsafe(no_mangle)]
pub(crate) unsafe extern "C" fn esp_mn_commands_add(
    command_id: c_int,
    text: *const c_char,
) -> c_int {
    let Some((iface, model, root)) = list() else {
        return ESP_ERR_INVALID_STATE;
    };
    // SAFETY: C callers pass a NUL-terminated phrase, and the list is not used during this.
    unsafe {
        if nodes(root).count() > ESP_MN_MAX_PHRASE_NUM {
            log::error!("The number of phrases exceeds {ESP_MN_MAX_PHRASE_NUM}");
            return ESP_ERR_INVALID_STATE;
        }
        let phonemes = flite_g2p(text, 1);
        if phonemes.is_null() {
            return ESP_ERR_INVALID_STATE;
        }
        let result = if (iface.check_speech_command)(model, phonemes) == 0 {
            log::error!(
                "invalid command, please check format, {:?} ({:?}).",
                CStr::from_ptr(text),
                CStr::from_ptr(phonemes)
            );
            ESP_ERR_INVALID_STATE
        } else if update_existing(root, command_id, text) {
            ESP_OK
        } else {
            let phrase = phrase_alloc(command_id, CStr::from_ptr(text));
            if phrase.is_null() {
                ESP_ERR_INVALID_STATE
            } else {
                append(root, phrase, phonemes)
            }
        };
        deallocate(phonemes.cast());
        result
    }
}

/// Adds a phrase with its phonemes, which MultiNet7 checks.
#[unsafe(no_mangle)]
pub(crate) unsafe extern "C" fn esp_mn_commands_phoneme_add(
    command_id: c_int,
    text: *const c_char,
    phonemes: *const c_char,
) -> c_int {
    let Some((iface, model, root)) = list() else {
        return ESP_ERR_INVALID_STATE;
    };
    // SAFETY: C callers pass a NUL-terminated phrase and phonemes, and the list is not used
    // during this.
    unsafe {
        if nodes(root).count() > ESP_MN_MAX_PHRASE_NUM {
            log::error!("The number of phrases exceeds {ESP_MN_MAX_PHRASE_NUM}");
            return ESP_ERR_INVALID_STATE;
        }
        if (iface.check_speech_command)(model, phonemes) == 0 {
            log::error!(
                "invalid command, please check format, {:?} ({:?}).",
                CStr::from_ptr(text),
                CStr::from_ptr(phonemes)
            );
            return ESP_ERR_INVALID_STATE;
        }
        if update_existing(root, command_id, text) {
            return ESP_OK;
        }
        let phrase = phrase_alloc(command_id, CStr::from_ptr(text));
        if phrase.is_null() {
            return ESP_ERR_INVALID_STATE;
        }
        append(root, phrase, phonemes)
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn esp_mn_commands_get_from_string(text: *const c_char) -> *mut EspMnPhrase {
    let root = ROOT.load(Ordering::Acquire);
    if root.is_null() {
        log::warn!("The MultiNet phrases are not initialized");
        return ptr::null_mut();
    }
    // SAFETY: C callers pass a NUL-terminated phrase, and the list is not used during this.
    unsafe {
        let node = search(root, text);
        if node.is_null() {
            ptr::null_mut()
        } else {
            (*node).phrase
        }
    }
}

/// Hands the phrases to MultiNet. Returns the phrases it rejected, or null.
#[unsafe(no_mangle)]
pub(crate) unsafe extern "C" fn esp_mn_commands_update() -> *mut EspMnError {
    let Some((iface, model, root)) = list() else {
        return ptr::null_mut();
    };
    // SAFETY: The model and the list are valid.
    unsafe {
        let error = (iface.set_speech_commands)(model, root);
        if error.is_null() || (*error).num == 0 {
            ptr::null_mut()
        } else {
            error
        }
    }
}
