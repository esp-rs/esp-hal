#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Real-Time Control and Low-power Management (RTC_CNTL)
//!
//! ## Overview
//!
//! The RTC_CNTL peripheral is responsible for managing the low-power modes on
//! the chip.
//!
//! ## Configuration
//!
//! It also includes the necessary configurations and constants for clock
//! sources and low-power management. The driver provides the following features
//! and functionalities:
//!
//! * Clock Configuration
//! * Calibration
//! * Low-Power Management
//! * Handling Watchdog Timers
#![cfg_attr(
    lp_timer_driver_supported,
    doc = r#"
## Examples

### Get time in ms from the RTC Timer

```rust, no_run
# {before_snippet}
# use esp_hal::{delay::Delay, rtc_cntl::Rtc};

let rtc = Rtc::new(peripherals.RTC_TIMER);
let delay = Delay::new();

loop {
    // Print the current RTC time in milliseconds
    let time_ms = rtc.current_time_us() / 1000;
    delay.delay_millis(1000);

    // Set the time to half a second in the past
    let new_time = rtc.current_time_us() - 500_000;
    rtc.set_current_time_us(new_time);
}
# }
```

### RWDT usage
```rust, no_run
# {before_snippet}
# use core::cell::RefCell;
# use critical_section::Mutex;
# use esp_hal::delay::Delay;
# use esp_hal::rtc_cntl::Rtc;
# use esp_hal::rtc_cntl::Rwdt;
# use esp_hal::rtc_cntl::RwdtStage;
static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));

let mut delay = Delay::new();
let mut rtc = Rtc::new(peripherals.RTC_TIMER);

rtc.set_interrupt_handler(interrupt_handler);
rtc.rwdt
    .set_timeout(RwdtStage::Stage0, Duration::from_millis(2000));
rtc.rwdt.listen();

critical_section::with(|cs| RWDT.borrow_ref_mut(cs).replace(rtc.rwdt));
# {after_snippet}

// Where the `LP_WDT` interrupt handler is defined as:
# use core::cell::RefCell;
# use critical_section::Mutex;
# use esp_hal::rtc_cntl::Rwdt;
# use esp_hal::rtc_cntl::RwdtStage;
static RWDT: Mutex<RefCell<Option<Rwdt>>> = Mutex::new(RefCell::new(None));

// Handle the corresponding interrupt
#[esp_hal::handler]
fn interrupt_handler() {
    critical_section::with(|cs| {
        println!("RWDT Interrupt");

        let mut rwdt = RWDT.borrow_ref_mut(cs);
        if let Some(rwdt) = rwdt.as_mut() {
            rwdt.clear_interrupt();

            println!("Restarting in 5 seconds...");

            rwdt.set_timeout(RwdtStage::Stage0, Duration::from_millis(5000));
            rwdt.unlisten();
        }
    });
}
```

### Get time in ms from the RTC Timer
```rust, no_run
# {before_snippet}
# use esp_hal::{delay::Delay, rtc_cntl::Rtc};

let rtc = Rtc::new(peripherals.RTC_TIMER);
let delay = Delay::new();

loop {
    // Get the current RTC time in milliseconds
    let time_ms = rtc.current_time_us() / 1000;
    delay.delay_millis(1000);

    // Set the time to half a second in the past
    let new_time = rtc.current_time_us() - 500_000;
    rtc.set_current_time_us(new_time);
}
# }
```
"#
)]
pub use self::rtc::SocResetReason;
#[cfg(sleep_driver_supported)]
use crate::rtc_cntl::sleep::{RtcSleepConfig, WakeSource, WakeTriggers};
#[cfg_attr(not(lp_timer_driver_supported), expect(unused))]
use crate::{
    interrupt::{self, InterruptHandler},
    peripherals::Interrupt,
};
use crate::{peripherals::RTC_TIMER, system::Cpu, time::Duration};
// only include sleep where it's been implemented
#[cfg(sleep_driver_supported)]
pub mod sleep;

// Power-domain locks that keep a domain powered across light sleep.
#[cfg(sleep_pd_retention)]
pub(crate) mod power_domain;

// regDMA-based register retention of the TOP domain's peripherals.
#[cfg(sleep_pd_retention)]
pub(crate) mod retention;

// Software CPU-register retention for CPU power-down.
#[cfg(sleep_pd_retention)]
pub mod cpu_retention;

#[cfg_attr(esp32, path = "rtc/esp32.rs")]
#[cfg_attr(esp32c2, path = "rtc/esp32c2.rs")]
#[cfg_attr(esp32c3, path = "rtc/esp32c3.rs")]
#[cfg_attr(esp32c5, path = "rtc/esp32c5.rs")]
#[cfg_attr(esp32c6, path = "rtc/esp32c6.rs")]
#[cfg_attr(esp32c61, path = "rtc/esp32c61.rs")]
#[cfg_attr(esp32h2, path = "rtc/esp32h2.rs")]
#[cfg_attr(esp32p4, path = "rtc/esp32p4.rs")]
#[cfg_attr(esp32s2, path = "rtc/esp32s2.rs")]
#[cfg_attr(esp32s3, path = "rtc/esp32s3.rs")]
#[cfg_attr(esp32s31, path = "rtc/esp32s31.rs")]
pub(crate) mod rtc;

cfg_select! {
    esp32s31 => {
        use crate::peripherals::LP_SYS as LP_AON;
        use crate::peripherals::LP_WDT;
    }
    soc_has_lp_wdt => {
        use crate::peripherals::LP_WDT;
        use crate::peripherals::LP_AON;
    }
    _ => {
        use crate::peripherals::LPWR;
        use crate::peripherals::LPWR as LP_WDT;
        use crate::peripherals::LPWR as LP_AON;
    }
}

#[rustfmt::skip]
#[cfg(sleep_driver_supported)]
macro_rules! wakeup_docstring {
    (Ext0)          => { "EXT0 wakeup" };
    (Ext1)          => { "EXT1 wakeup, via one or more RTC GPIOs (`RTC_CNTL`)." };
    (Gpio)          => { "GPIO wakeup." };
    (Timer)         => { "Timer wakeup." };
    (Sdio)          => { "SDIO wakeup." };
    (Wifi)          => { "Wi-Fi (MAC) wakeup." };
    (WifiBeacon)    => { "Wi-Fi beacon wakeup." };
    (Uart0)         => { "UART0 wakeup." };
    (Uart1)         => { "UART1 wakeup." };
    (Uart2)         => { "UART2 wakeup." };
    (Uart3)         => { "UART3 wakeup." };
    (Uart4)         => { "UART4 wakeup." };
    (Touch)         => { "Touch sensor wakeup." };
    (Ulp)           => { "ULP wakeup." };
    (UlpRiscv)      => { "ULP RISC-V coprocessor wakeup." };
    (UlpRiscvTrap)  => { "ULP RISC-V coprocessor trap (crash) wakeup." };
    (Bt)            => { "Bluetooth wakeup." };
    (LpCore)        => { "LP core wakeup." };
    (Usb)           => { "USB wakeup." };
    ($($other:tt)*) => { compile_error!(concat!("Unknown wakeup source: ", stringify!($($other)*))) };
}

#[cfg(sleep_driver_supported)]
for_each_wakeup_source! {
    (all $( ($variant:ident, $bit:literal) ),*) => {
        /// A source that can wake the chip from sleep.
        ///
        /// Each variant models a single bit in the wakeup registers. This enum is the internal
        /// representation shared by [`WakeTriggers`][crate::rtc_cntl::sleep::WakeTriggers] (the
        /// sources configured to end a sleep) and [`WakeupReason`] (the source(s) that caused the
        /// most recent wakeup, see [`wakeup_cause`]).
        ///
        /// Note that the available variants depend on the target chip.
        #[derive(Debug, enumset::EnumSetType)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        pub enum WakeupSource {
            $(
                #[doc = wakeup_docstring!($variant)]
                $variant = $bit,
            )*
        }
    };
}

/// The source(s) that caused the most recent wakeup from sleep.
///
/// Returned by [`wakeup_cause`]. This is a thin wrapper around the set of [`WakeupSource`]s that
/// were reported by the hardware. The set is empty if the chip was not woken from sleep.
///
/// Note that the available sources depend on the target chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg(sleep_driver_supported)]
pub struct WakeupReason(enumset::EnumSet<WakeupSource>);

#[cfg(sleep_driver_supported)]
impl WakeupReason {
    /// Returns `true` if the chip was not woken from sleep by any source.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// Returns `true` if the given source caused the most recent wakeup.
    pub fn contains(&self, source: WakeupSource) -> bool {
        self.0.contains(source)
    }

    /// Returns an iterator over the sources that caused the most recent wakeup.
    pub fn iter(&self) -> impl Iterator<Item = WakeupSource> {
        self.0.iter()
    }
}

/// RTC clock.
pub struct Rtc<'d> {
    rtc_timer: RTC_TIMER<'d>,
    /// Reset Watchdog Timer.
    pub rwdt: Rwdt,
    /// Super Watchdog
    #[cfg(soc_has_swd_watchdog)]
    pub swd: Swd,
}

impl<'d> Rtc<'d> {
    /// Create a new instance in [crate::Blocking] mode.
    ///
    /// Optionally an interrupt handler can be bound.
    pub fn new(rtc_timer: RTC_TIMER<'d>) -> Self {
        Self {
            rtc_timer,
            rwdt: Rwdt,
            #[cfg(soc_has_swd_watchdog)]
            swd: Swd,
        }
    }

    /// Get the time since boot in the raw register units.
    #[cfg(lp_timer_driver_supported)]
    fn time_since_boot_raw(&self) -> u64 {
        let rtc = self.rtc_timer.register_block();

        // Load counter value
        cfg_select! {
            any(esp32, esp32s2, esp32s3, esp32c2, esp32c3) => {
                // Keep update high for at least one RTC slowclk period, assumes 150k RTC_SLOWCLK
                // Without this, this function may return a stale value
                const UPDATE_COUNT: usize = if cfg!(esp32) {
                    20
                } else {
                    10
                };
                for _ in 0..UPDATE_COUNT {
                    rtc.time_update().write(|w| w.time_update().set_bit());
                    crate::rom::ets_delay_us(1);
                }
            }
            _ => {
                rtc.update().write(|w| w.main_timer_update().set_bit());
            }
        }

        // Read counter value
        cfg_select! {
            esp32 => {
                while rtc.time_update().read().time_valid().bit_is_clear() {
                    // Might take 1 RTC slowclk period, don't flood RTC bus
                    crate::rom::ets_delay_us(1);
                }

                let h = rtc.time1().read().time_hi().bits();
                let l = rtc.time0().read().time_lo().bits();
            }
            any(esp32s2, esp32s3, esp32c2, esp32c3) => {
                let h = rtc.time_high0().read().timer_value0_high().bits();
                let l = rtc.time_low0().read().timer_value0_low().bits();
            }
            _ => {
                let h = rtc.main_buf0_high().read().main_timer_buf0_high().bits();
                let l = rtc.main_buf0_low().read().main_timer_buf0_low().bits();
            }
        }

        ((h as u64) << 32) | (l as u64)
    }

    /// Get the time elapsed since the last power-on reset.
    ///
    /// It should be noted that any reset or sleep, other than a power-up reset, will not stop or
    /// reset the RTC timer.
    #[cfg(lp_timer_driver_supported)]
    pub fn time_since_power_up(&self) -> Duration {
        Duration::from_micros(crate::clock::rtc_ticks_to_us(self.time_since_boot_raw()))
    }

    /// Read the current value of the boot time registers in microseconds.
    #[cfg(lp_timer_driver_supported)]
    fn boot_time_us(&self) -> u64 {
        // For more info on about how RTC setting works and what it has to do with boot time, see https://github.com/esp-rs/esp-hal/pull/1883
        let (low_reg, high_reg) = cfg_select! {
            esp32s31 => (LP_AON::regs().lp_store(2), LP_AON::regs().lp_store(3)),
            esp32p4 => (LP_AON::regs().lp_store2(), LP_AON::regs().lp_store3()),
            _ => (LP_AON::regs().store2(), LP_AON::regs().store3()),
        };

        let l = low_reg.read().bits() as u64;
        let h = high_reg.read().bits() as u64;

        // https://github.com/espressif/esp-idf/blob/23e4823f17a8349b5e03536ff7653e3e584c9351/components/newlib/port/esp_time_impl.c#L115
        l + (h << 32)
    }

    /// Set the current value of the boot time registers in microseconds.
    #[cfg(lp_timer_driver_supported)]
    fn set_boot_time_us(&self, boot_time_us: u64) {
        // Please see `boot_time_us` for documentation on registers and peripherals
        // used for certain SOCs.

        let (low_reg, high_reg) = cfg_select! {
            esp32s31 => (LP_AON::regs().lp_store(2), LP_AON::regs().lp_store(3)),
            esp32p4 => (LP_AON::regs().lp_store2(), LP_AON::regs().lp_store3()),
            _ => (LP_AON::regs().store2(), LP_AON::regs().store3()),
        };

        // https://github.com/espressif/esp-idf/blob/23e4823/components/newlib/port/esp_time_impl.c#L102-L103

        low_reg // Low bits
            .write(|w| unsafe { w.bits((boot_time_us & 0xffff_ffff) as u32) });
        high_reg // High bits
            .write(|w| unsafe { w.bits((boot_time_us >> 32) as u32) });
    }

    #[procmacros::doc_replace]
    /// Get the current time in microseconds.
    ///
    /// # Example
    ///
    /// This example shows how to get the weekday of the current time in
    /// New York using the `jiff` crate. This example works in core-only
    /// environments without dynamic memory allocation.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// # use esp_hal::rtc_cntl::Rtc;
    /// use jiff::{
    ///     Timestamp,
    ///     tz::{self, TimeZone},
    /// };
    ///
    /// static TZ: TimeZone = tz::get!("America/New_York");
    ///
    /// let rtc = Rtc::new(peripherals.RTC_TIMER);
    /// let now = Timestamp::from_microsecond(rtc.current_time_us() as i64)?;
    /// let weekday_in_new_york = now.to_zoned(TZ.clone()).weekday();
    /// # {after_snippet}
    /// ```
    #[cfg(lp_timer_driver_supported)]
    pub fn current_time_us(&self) -> u64 {
        // Current time is boot time + time since boot

        let rtc_time_us = self.time_since_power_up().as_micros();
        let boot_time_us = self.boot_time_us();
        let wrapped_boot_time_us = u64::MAX - boot_time_us;

        // We can detect if we wrapped the boot time by checking if rtc time is greater
        // than the amount of time we would've wrapped.
        if rtc_time_us > wrapped_boot_time_us {
            // We also just checked that this won't overflow
            rtc_time_us - wrapped_boot_time_us
        } else {
            boot_time_us + rtc_time_us
        }
    }

    /// Set the current time in microseconds.
    #[cfg(lp_timer_driver_supported)]
    pub fn set_current_time_us(&self, current_time_us: u64) {
        // Current time is boot time + time since boot (rtc time)
        // So boot time = current time - time since boot (rtc time)

        let rtc_time_us = self.time_since_power_up().as_micros();
        if current_time_us < rtc_time_us {
            // An overflow would happen if we subtracted rtc_time_us from current_time_us.
            // To work around this, we can wrap around u64::MAX by subtracting the
            // difference between the current time and the time since boot.
            // Subtracting time since boot and adding current new time is equivalent and
            // avoids overflow. We just checked that rtc_time_us is less than time_us
            // so this won't overflow.
            self.set_boot_time_us(u64::MAX - rtc_time_us + current_time_us)
        } else {
            self.set_boot_time_us(current_time_us - rtc_time_us)
        }
    }

    pub(crate) const RTC_DISABLE_ROM_LOG: u32 = 1;

    /// Temporarily disable log messages of the ROM bootloader.
    ///
    /// If you need to permanently disable the ROM bootloader messages, you'll
    /// need to set the corresponding eFuse.
    pub fn disable_rom_message_printing(&self) {
        // Corresponding documentation:
        // ESP32-S3: TRM v1.5 chapter 8.3
        // ESP32-H2: TRM v0.5 chapter 8.2.3

        let reg = cfg_select! {
            esp32s31 => LP_AON::regs().lp_store(4),
            esp32p4 => LP_AON::regs().lp_store4(),
            _ => LP_AON::regs().store4(),
        };
        let disable_mask = cfg_select! {
            any(esp32p4, esp32s31) => Self::RTC_DISABLE_ROM_LOG | (Self::RTC_DISABLE_ROM_LOG << 16),
            _ => Self::RTC_DISABLE_ROM_LOG,
        };
        reg.modify(|r, w| unsafe { w.bits(r.bits() | disable_mask) });
    }

    /// Register an interrupt handler for the RTC.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    #[cfg(lp_timer_driver_supported)]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        let interrupt = cfg_select! {
            esp32p4 => Interrupt::LP_TIMER0,
            soc_has_lp_wdt => Interrupt::LP_WDT,
            _ => Interrupt::RTC_CORE,
        };
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }
        interrupt::bind_handler(interrupt, handler);
    }
}

impl crate::private::Sealed for Rtc<'_> {}

#[instability::unstable]
#[cfg(lp_timer_driver_supported)]
impl crate::interrupt::InterruptConfigurable for Rtc<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

/// Behavior of the RWDT stage if it times out.
#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum RwdtStageAction {
    /// No effect on the system.
    Off         = 0,
    /// Trigger an interrupt.
    Interrupt   = 1,
    /// Reset the CPU core.
    ResetCpu    = 2,
    /// Reset the main system.
    /// The power management unit and RTC peripherals will not be reset.
    ResetCore   = 3,
    /// Reset the main system, power management unit and RTC peripherals.
    ResetSystem = 4,
}

/// RWDT stages.
///
/// Timer stages allow for a timer to have a series of different timeout values
/// and corresponding expiry action.
#[derive(Debug, Clone, Copy)]
pub enum RwdtStage {
    /// RWDT stage 0.
    Stage0,
    /// RWDT stage 1.
    Stage1,
    /// RWDT stage 2.
    Stage2,
    /// RWDT stage 3.
    Stage3,
}

/// RTC Watchdog Timer.
#[non_exhaustive]
pub struct Rwdt;

/// RTC Watchdog Timer driver.
impl Rwdt {
    /// Enable the watchdog timer instance.
    /// Watchdog starts with default settings (`stage 0` resets the system, the
    /// others are deactivated)
    pub fn enable(&mut self) {
        self.set_enabled(true);
    }

    /// Disable the watchdog timer instance.
    pub fn disable(&mut self) {
        self.set_enabled(false);
    }

    fn set_listen(&mut self, enable: bool) {
        let regs = LP_WDT::regs();

        self.set_write_protection(false);

        // Configure STAGE0 to trigger an interrupt upon expiration
        let cfg_reg = cfg_select! {
            esp32p4 => regs.config0(),
            _ => regs.wdtconfig0(),
        };
        cfg_reg.modify(|_, w| unsafe {
            w.wdt_stg0().bits(if enable {
                RwdtStageAction::Interrupt as u8
            } else {
                RwdtStageAction::ResetSystem as u8
            })
        });

        regs.int_ena().modify(|_, w| {
            cfg_select! {
                esp32p4 => w.lp_wdt().bit(enable),
                _ => w.wdt().bit(enable),
            }
        });

        self.set_write_protection(true);
    }

    /// Listen for interrupts on stage 0.
    pub fn listen(&mut self) {
        self.set_listen(true);
    }

    /// Stop listening for interrupts on stage 0.
    pub fn unlisten(&mut self) {
        self.set_listen(false);
    }

    /// Clear interrupt.
    pub fn clear_interrupt(&mut self) {
        self.set_write_protection(false);

        LP_WDT::regs().int_clr().write(|w| {
            cfg_select! {
                esp32p4 => w.lp_wdt().clear_bit_by_one(),
                _ => w.wdt().clear_bit_by_one(),
            }
        });

        self.set_write_protection(true);
    }

    /// Check if the interrupt is set.
    pub fn is_interrupt_set(&self) -> bool {
        cfg_select! {
            esp32p4 => LP_WDT::regs().int_st().read().lp_wdt().bit_is_set(),
            _ => LP_WDT::regs().int_st().read().wdt().bit_is_set(),
        }
    }

    /// Feed the watchdog timer.
    pub fn feed(&mut self) {
        self.set_write_protection(false);

        cfg_select! {
            esp32p4 => LP_WDT::regs().feed().write(|w| w.feed().set_bit()),
            _ => LP_WDT::regs().wdtfeed().write(|w| w.wdt_feed().set_bit()),
        };

        self.set_write_protection(true);
    }

    fn set_write_protection(&mut self, enable: bool) {
        let wkey = if enable { 0u32 } else { 0x50D8_3AA1 };
        let reg = cfg_select! {
            esp32p4 => LP_WDT::regs().wprotect(),
            _ => LP_WDT::regs().wdtwprotect(),
        };
        reg.write(|w| unsafe { w.bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        self.set_write_protection(false);

        let regs = LP_WDT::regs();
        let config0 = cfg_select! {
            esp32p4 => regs.config0(),
            _ => regs.wdtconfig0(),
        };

        config0.write(|w| unsafe {
            if enable {
                w.wdt_flashboot_mod_en().bit(false);
                w.wdt_pause_in_slp().set_bit();
                w.wdt_cpu_reset_length().bits(7);
                w.wdt_sys_reset_length().bits(7);
                w.wdt_stg0().bits(RwdtStageAction::ResetSystem as u8);
                w.wdt_stg1().bits(RwdtStageAction::Off as u8);
                w.wdt_stg2().bits(RwdtStageAction::Off as u8);
                w.wdt_stg3().bits(RwdtStageAction::Off as u8);
                w.wdt_en().set_bit()
            } else {
                w.bits(0)
            }
        });

        self.set_write_protection(true);
    }

    /// Configure timeout value for the selected stage.
    pub fn set_timeout(&mut self, stage: RwdtStage, timeout: Duration) {
        let timeout_raw = crate::clock::us_to_rtc_ticks(timeout.as_micros()) as u32;

        #[cfg(not(esp32))]
        let timeout_raw = timeout_raw >> (1 + crate::efuse::rwdt_multiplier());

        self.set_write_protection(false);

        let regs = LP_WDT::regs();
        let config_reg = cfg_select! {
            esp32p4 => regs.config(stage as usize),
            _ => regs.wdtconfig(stage as usize),
        };

        config_reg.modify(|_, w| unsafe { w.hold().bits(timeout_raw) });

        self.set_write_protection(true);
    }

    /// Set the action for a specific stage.
    pub fn set_stage_action(&mut self, stage: RwdtStage, action: RwdtStageAction) {
        self.set_write_protection(false);

        let regs = LP_WDT::regs();
        let cfg_reg = cfg_select! {
            esp32p4 => regs.config0(),
            _ => regs.wdtconfig0(),
        };
        cfg_reg.modify(|_, w| unsafe {
            match stage {
                RwdtStage::Stage0 => w.wdt_stg0().bits(action as u8),
                RwdtStage::Stage1 => w.wdt_stg1().bits(action as u8),
                RwdtStage::Stage2 => w.wdt_stg2().bits(action as u8),
                RwdtStage::Stage3 => w.wdt_stg3().bits(action as u8),
            }
        });

        self.set_write_protection(true);
    }
}

/// Super Watchdog
#[cfg(soc_has_swd_watchdog)]
#[non_exhaustive]
pub struct Swd;

/// Super Watchdog driver
#[cfg(soc_has_swd_watchdog)]
impl Swd {
    /// Enable the watchdog timer instance
    pub fn enable(&mut self) {
        self.set_enabled(true);
    }

    /// Disable the watchdog timer instance
    pub fn disable(&mut self) {
        self.set_enabled(false);
    }

    /// Enable/disable write protection for WDT registers
    fn set_write_protection(&mut self, enable: bool) {
        let wkey = if enable {
            0u32
        } else if cfg!(any(esp32c2, esp32c3, esp32s2, esp32s3)) {
            0x8F1D_312A
        } else {
            0x50D8_3AA1
        };

        LP_WDT::regs()
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(wkey) });
    }

    fn set_enabled(&mut self, enable: bool) {
        self.set_write_protection(false);

        let reg = cfg_select! {
            esp32p4 => LP_WDT::regs().swd_config(),
            _ => LP_WDT::regs().swd_conf(),
        };
        reg.write(|w| w.swd_auto_feed_en().bit(!enable));

        self.set_write_protection(true);
    }
}

/// Return reset reason.
pub fn reset_reason(cpu: Cpu) -> Option<SocResetReason> {
    let reason = crate::rom::rtc_get_reset_reason(cpu as u32);

    SocResetReason::from_repr(reason as usize)
}

/// Tracks whether the chip has just returned from a light sleep.
#[cfg(sleep_driver_supported)]
static LIGHT_SLEEP_WAKEUP: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);

/// Return the cause(s) of the most recent wakeup.
///
/// A sleep can be ended by more than one source simultaneously, so all matching sources are
/// returned. The result is empty if the chip was not woken from sleep (for example on a cold boot,
/// or after a reset unrelated to deep sleep).
#[cfg(sleep_driver_supported)]
pub fn wakeup_cause() -> WakeupReason {
    if reset_reason(Cpu::ProCpu) != Some(SocResetReason::CoreDeepSleep)
        && !LIGHT_SLEEP_WAKEUP.load(portable_atomic::Ordering::Relaxed)
    {
        return WakeupReason::default();
    }

    let reg = cfg_select! {
        soc_has_pmu => crate::peripherals::PMU::regs().slp_wakeup_status0(),
        esp32 => LPWR::regs().wakeup_state(),
        _ => LPWR::regs().slp_wakeup_cause(),
    };

    #[allow(clippy::unnecessary_cast)]
    let wakeup_cause_bits = reg.read().wakeup_cause().bits() as u32;

    WakeupReason(enumset::EnumSet::from_u32_truncated(wakeup_cause_bits))
}

#[cfg(sleep_light_sleep)]
cfg_select! {
    feature = "rt" => {
        #[unsafe(no_mangle)]
        static ESP_HAL_WAKE_LOCK_COUNT: portable_atomic::AtomicUsize =
            portable_atomic::AtomicUsize::new(0);

        fn wake_lock_count() -> &'static portable_atomic::AtomicUsize {
            &ESP_HAL_WAKE_LOCK_COUNT
        }
    }
    _ => {
        unsafe extern "Rust" {
            static ESP_HAL_WAKE_LOCK_COUNT: portable_atomic::AtomicUsize;
        }

        fn wake_lock_count() -> &'static portable_atomic::AtomicUsize {
            // use of extern static is unsafe and requires unsafe block
            unsafe { &ESP_HAL_WAKE_LOCK_COUNT }
        }
    }
}

/// A guard that prevents the system from entering automatic light sleep.
///
/// While at least one `WakeLock` is held, [`WakeLock::is_active`] returns `true`
/// and the auto-lightsleep idle hook will not put the chip to sleep. The lock is
/// released when the guard is dropped.
#[cfg_attr(
    not(sleep_light_sleep),
    doc = r"

Note: This chip does not support automatic light sleep. On this chip, `WakeLock` does nothing."
)]
#[instability::unstable]
#[non_exhaustive]
pub struct WakeLock;

impl WakeLock {
    /// Acquires a wake lock, preventing automatic light sleep until it is dropped.
    pub fn new() -> Self {
        Self::acquire();
        Self
    }

    /// Acquires a wake lock, preventing automatic light sleep.
    pub fn acquire() {
        #[cfg(sleep_light_sleep)]
        wake_lock_count().fetch_add(1, portable_atomic::Ordering::AcqRel);
    }

    /// Releases a wake lock, allowing automatic light sleep when no wake locks are held.
    ///
    /// Note that this function should only be called to release a wake lock acquired via
    /// [`Self::acquire`].
    pub fn release() {
        #[cfg(sleep_light_sleep)]
        {
            let previous = wake_lock_count().fetch_sub(1, portable_atomic::Ordering::AcqRel);
            debug_assert_ne!(previous, 0, "wake lock counter underflow");
        }
    }

    /// Returns `true` if at least one wake lock is currently held.
    #[instability::unstable]
    pub fn is_active() -> bool {
        cfg_select! {
            sleep_light_sleep => {
                wake_lock_count().load(portable_atomic::Ordering::Acquire)
                    != 0
            }
            _ => true,
        }
    }
}

#[instability::unstable]
impl Clone for WakeLock {
    fn clone(&self) -> Self {
        Self::new()
    }
}

#[instability::unstable]
impl Default for WakeLock {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for WakeLock {
    fn drop(&mut self) {
        Self::release();
    }
}
