#![doc = include_str!("../README.md")]
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(rustdoc::bare_urls)]
#![no_std]

#[doc(hidden)]
/// Helper macro for checking doctest code snippets
#[macro_export]
macro_rules! before_snippet {
    () => {
        r#"
# #![no_std]
# use procmacros::handler;
# use esp_hal::{interrupt::{self, InterruptConfigurable}, time::{Duration, Instant, Rate}};
# macro_rules! println {
#     ($($tt:tt)*) => { };
# }
# macro_rules! print {
#     ($($tt:tt)*) => { };
# }
# #[panic_handler]
# fn panic(_ : &core::panic::PanicInfo) -> ! {
#     loop {}
# }
# fn main() {
#   let _ = example();
# }
# struct ExampleError {}
# impl <T> From<T> for ExampleError where T: core::fmt::Debug {
#   fn from(_value: T) -> Self {
#       Self{}
#   }
# }
# async fn example() -> Result<(), ExampleError> {
#   let mut peripherals = esp_hal::init(esp_hal::Config::default());
"#
    };
}

pub mod rom;

/// This is needed by `libesp_rom.a` (if used)
/// Other crates (i.e. esp-radio) also rely on this being defined somewhere
#[unsafe(no_mangle)]
unsafe extern "C" fn __assert_func(
    file: *const core::ffi::c_char,
    line: u32,
    func: *const core::ffi::c_char,
    expr: *const core::ffi::c_char,
) {
    unsafe {
        panic!(
            "__assert_func in {}:{} ({}): {}",
            core::ffi::CStr::from_ptr(file).to_str().unwrap(),
            line,
            core::ffi::CStr::from_ptr(func).to_str().unwrap(),
            core::ffi::CStr::from_ptr(expr).to_str().unwrap(),
        );
    }
}
