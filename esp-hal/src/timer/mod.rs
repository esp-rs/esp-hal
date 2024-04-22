//! General-purpose timers.

#[cfg(systimer)]
pub mod systimer;
#[cfg(any(timg0, timg1))]
pub mod timg;
