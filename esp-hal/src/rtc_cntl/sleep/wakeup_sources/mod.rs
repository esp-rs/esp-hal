#[cfg(not(any(esp32c2, esp32c3)))]
mod ext1;
#[cfg(not(any(esp32c2, esp32c3)))]
pub use ext1::Ext1WakeupSource;

mod timer;
pub use timer::TimerWakeupSource;
