//! # Configuration
//!
//! ## Overview
//! This module contains the initial configuation for the system.
//!
//! ## Configuration
//! In the esp_hal::init method, we can configure different parameters for the
//! system:
//! - CPU clock configuration.
//! - Watchdog configuration.
//!
//! ## Example
//!
//! ### Default initialization
//!
//! ```rust, no_run
//! let peripherals = esp_hal::init(esp_hal::config::Config::default());
//! ```
//!
//! ### Custom initialization
//!
//! ```rust, no_run
//! let mut config = esp_hal::config::Config::default();
//! config.cpu_clock = CpuClock::max()
//! config.watchdog.rwdt =
//!     esp_hal::config::WatchdogStatus::Enabled(fugit::MicrosDurationU64::millis(1000 as u64));
//! let peripherals = esp_hal::init(config);
//! ```

use crate::clock::CpuClock;

/// Watchdog status.
#[derive(Default, PartialEq)]
pub enum WatchdogStatus {
    /// Enables a watchdog timer with the specified timeout.
    Enabled(fugit::MicrosDurationU64),
    /// Disables the watchdog timer.
    #[default]
    Disabled,
}

/// Watchdogs configuration.
#[non_exhaustive]
#[derive(Default)]
pub struct WatchdogConfig {
    #[cfg(not(any(esp32, esp32s2)))]
    /// Enable the super watchdog timer, which is slightly less than one second.
    pub swd: bool,
    /// Configures the reset watchdog timer.
    pub rwdt: WatchdogStatus,
    /// Configures the timg0 watchdog timer.
    pub timg0: WatchdogStatus,
    #[cfg(timg1)]
    /// Configures the timg1 watchdog timer.
    ///
    /// By default, the bootloader does not enables this watchdog timer.
    pub timg1: WatchdogStatus,
}

/// System configuration.
#[non_exhaustive]
#[derive(Default)]
pub struct Config {
    /// The CPU clock configuration.
    pub cpu_clock: CpuClock,
    /// Enable watchdog timer(s).
    pub watchdog: WatchdogConfig,
}