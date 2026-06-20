//! # Secure Digital / MultiMedia Card host (SDMMC / SDIO)
//!
//! ## Overview
//!
//! Driver for the SDMMC/SDIO host controller (`SDHOST`). The controller exposes
//! up to two independent card slots that share a single transfer engine.
//!
//! This module is under active development; only scaffolding is present so far.

use core::marker::PhantomData;

#[cfg(feature = "__sdmmc")]
use sdio as _;

/// SDMMC / SDIO host controller driver.
pub struct SdHostController<'d> {
    _lifetime: PhantomData<&'d ()>,
}

/// Selects one of the controller's card slots.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Slot {
    /// Slot 0.
    _0,
    /// Slot 1.
    _1,
}

/// Card clock source feeding the controller's divider.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// 160 MHz PLL.
    Pll160m,
    /// Crystal oscillator.
    Xtal,
}

/// Clock input sampling phase used for high-speed tuning.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct DelayPhase;

/// Slot configuration.
#[derive(Clone, Copy, Debug, Default)]
pub struct Config {}

/// Error returned by host operations.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {}

/// Error returned when applying a [`Config`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConfigError {}
