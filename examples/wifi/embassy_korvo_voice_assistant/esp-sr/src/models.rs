//! The English MultiNet7 model, embedded in the firmware, and the model list that MultiNet reads
//! through `get_static_srmodels`. ESP-SR's `srmodel_load` builds the same list from a flash
//! partition.

use alloc::{boxed::Box, ffi::CString, vec::Vec};
use core::{
    ffi::{CStr, c_char, c_int},
    ptr,
    sync::atomic::{AtomicPtr, Ordering},
};

use crate::sys::{SrModelData, SrModelList};

/// The model name, which is also the directory of its files.
pub const MULTINET: &CStr = c"mn7_en";

/// The files of a model, which MultiNet looks up by model and file name.
struct Model {
    name: &'static CStr,
    files: &'static [(&'static CStr, &'static [u8])],
}

macro_rules! model_file {
    ($path:literal) => {
        include_bytes!(concat!(env!("CARGO_MANIFEST_DIR"), "/model/", $path))
    };
}

const MODELS: [Model; 2] = [
    Model {
        name: MULTINET,
        files: &[
            (c"_MODEL_INFO_", model_file!("mn7_en/_MODEL_INFO_")),
            (c"mn7_data", model_file!("mn7_en/mn7_data")),
            (c"mn7_index", model_file!("mn7_en/mn7_index")),
            (c"vocab", model_file!("mn7_en/vocab")),
        ],
    },
    // The default phrases, which MultiNet loads before the application's replace them.
    Model {
        name: c"fst",
        files: &[(c"commands_en.txt", model_file!("fst/commands_en.txt"))],
    },
];

static MODEL_LIST: AtomicPtr<SrModelList> = AtomicPtr::new(ptr::null_mut());

/// Builds the model list once. It lives as long as the program.
pub fn install() {
    if !MODEL_LIST.load(Ordering::Acquire).is_null() {
        return;
    }
    fn leak<T>(items: impl IntoIterator<Item = T>) -> *const T {
        Vec::leak(items.into_iter().collect()).as_ptr()
    }

    let files = MODELS.map(|model| {
        let files = SrModelData {
            num: model.files.len() as c_int,
            files: leak(model.files.iter().map(|(name, _)| name.as_ptr())),
            data: leak(model.files.iter().map(|(_, data)| data.as_ptr())),
            sizes: leak(model.files.iter().map(|(_, data)| data.len() as c_int)),
        };
        Box::into_raw(Box::new(files)).cast_const()
    });
    let list = SrModelList {
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
extern "C" fn get_static_srmodels() -> *mut SrModelList {
    MODEL_LIST.load(Ordering::Acquire)
}

/// Without a base path, MultiNet reads the models from [`get_static_srmodels`].
#[unsafe(no_mangle)]
extern "C" fn get_model_base_path() -> *const c_char {
    ptr::null()
}
