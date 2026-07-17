#[cfg(any(esp32, esp32s2, esp32s3))]
mod ext0;
#[cfg(any(esp32, esp32s2, esp32s3))]
#[instability::unstable]
pub use ext0::*;

#[cfg(not(any(esp32c2, esp32c3)))]
mod ext1;
#[cfg(not(any(esp32c2, esp32c3)))]
#[instability::unstable]
pub use ext1::*;

mod timer;

#[instability::unstable]
pub use timer::*;

mod uart;
#[instability::unstable]
pub use uart::*;
