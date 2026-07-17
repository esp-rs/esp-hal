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

mod gpio;
#[instability::unstable]
pub use gpio::*;

#[cfg(esp32c6)]
mod lp_core;
#[cfg(esp32c6)]
#[instability::unstable]
pub use lp_core::*;

#[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
mod rtcio;
#[cfg(any(esp32c2, esp32c3, esp32s2, esp32s3))]
#[instability::unstable]
pub use rtcio::*;

mod timer;

#[instability::unstable]
pub use timer::*;

#[cfg(any(esp32s2, esp32s3))]
mod ulp;
#[cfg(any(esp32s2, esp32s3))]
#[instability::unstable]
pub use ulp::*;

mod uart;
#[instability::unstable]
pub use uart::*;
