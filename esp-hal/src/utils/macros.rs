#[doc(hidden)]
/// Helper macro for checking doctest code snippets
#[macro_export]
macro_rules! before_snippet {
    () => {
        r#"
# #![no_std]
# use esp_hal::prelude::*;
# use procmacros::handler;
# use esp_hal::interrupt;
# #[panic_handler]
# fn panic(_ : &core::panic::PanicInfo) -> ! {
#     loop {}
# }
# fn main() {
#     let mut peripherals = esp_hal::init(esp_hal::Config::default());
"#
    };
}
