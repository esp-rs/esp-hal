//! Helper Utils

// Only provide adapter when feature is enabled!
#[cfg(feature = "smartled")]
pub mod smart_leds_adapter;
#[cfg(feature = "smartled")]
pub use smart_leds_adapter::SmartLedsAdapter;

// Re-export the macro that due to the macro_export configuration was already exported
// in the root module (i.e., `esp-hal-common`)
#[cfg(feature = "smartled")]
pub use crate::smartLedAdapter;
