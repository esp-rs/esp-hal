//! # Configuration
//!
//! ## Overview
//! This module contains the initial configuration for the system.
//!
//! ## Configuration
//! In the [`esp_hal::init()`][crate::init] method, we can configure different
//! parameters for the system:
//! - CPU clock configuration.
//! - Watchdog configuration.
//!
//! ## Examples
//!
//! ### Default initialization
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! let peripherals = esp_hal::init(esp_hal::Config::default());
//! Ok(())
//! # }
//! ```
//! 
//! ### Custom initialization
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::clock::CpuClock;
//! use esp_hal::time::Duration;
//!
//! let config =
//! esp_hal::Config::default().with_cpu_clock(CpuClock::max()).
//!     with_watchdog(esp_hal::config::WatchdogConfig::default().
//!     with_rwdt(esp_hal::config::WatchdogStatus::Enabled(Duration::from_millis(1000u64))));
//! let peripherals = esp_hal::init(config);
//! # Ok(())
//! # }
//! ```

use crate::time::Duration;

/// Watchdog status.
#[derive(Default, PartialEq, Clone, Copy)]
pub enum WatchdogStatus {
    /// Enables a watchdog timer with the specified timeout.
    Enabled(Duration),
    /// Disables the watchdog timer.
    #[default]
    Disabled,
}

/// Watchdog configuration.
#[non_exhaustive]
#[derive(Default, Clone, Copy, procmacros::BuilderLite)]
pub struct WatchdogConfig {
    #[cfg(not(any(esp32, esp32s2)))]
    /// Enable the super watchdog timer, which has a trigger time of slightly
    /// less than one second.
    swd: bool,
    /// Configures the reset watchdog timer.
    rwdt: WatchdogStatus,
    /// Configures the `timg0` watchdog timer.
    timg0: WatchdogStatus,
    #[cfg(timg1)]
    /// Configures the `timg1` watchdog timer.
    ///
    /// By default, the bootloader does not enable this watchdog timer.
    timg1: WatchdogStatus,
}
