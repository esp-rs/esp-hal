#[cfg(any(feature = "time-systimer-16mhz", feature = "time-systimer-80mhz"))]
pub use self::systimer::*;
#[cfg(feature = "time-timg0")]
pub use self::timg::*;

#[cfg(any(feature = "time-systimer-16mhz", feature = "time-systimer-80mhz"))]
mod systimer;
#[cfg(feature = "time-timg0")]
mod timg;
