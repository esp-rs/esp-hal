#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Control the ULP core
//!
//! ## Overview
//!
//! The `ULP CORE` peripheral allows control over the `Ultra-Low Power
//! (ULP) core` in `ESP` chips. The ULP core is a low-power processor
//! designed for executing tasks in deep sleep mode, enabling efficient power
//! management in ESP systems.
//!
//! The `UlpCore` struct provides an interface to interact with the `ULP`
//! peripheral. It allows starting and configuring the ULP core for operation.
//! The `UlpCore` struct is initialized with a peripheral reference to the `ULP
//! CORE` instance.
//!
//! ## Examples
//!
//! ```rust, no_run
//! # {before_snippet}
//! const CODE: &[u8] = &[
//!     0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x05, 0x01, 0x81, 0x45, 0x85, 0x05, 0x0c, 0xc1, 0xf5,
//!     0xbf, 0x00, 0x00, 0x00, 0x00,
//! ];
//!
//! let mut ulp_core = esp_hal::lp_core::UlpCore::new(peripherals.ULP_RISCV_CORE);
//! // ulp_core.stop(); currently not implemented for ESP32-S2
//!
//! // copy code to RTC ram
//! let lp_ram = 0x5000_0000 as *mut u8;
//! unsafe {
//!     core::ptr::copy_nonoverlapping(CODE as *const _ as *const u8, lp_ram, CODE.len());
//! }
//!
//! // start ULP core
//! ulp_core.run(esp_hal::lp_core::UlpCoreWakeupSource::HpCpu);
//!
//! unsafe {
//!     let data = 0x5000_0010 as *mut u32;
//!     loop {}
//! }
//! # }
//! ```

use crate::{
    peripherals::LPWR,
    rtc_cntl::{
        WakeupSource,
        sleep::{SleepKind, SleepResource, WrappedSleepConfig},
    },
};

/// Enum representing the possible wakeup sources for the ULP core.
#[derive(Debug, Clone, Copy)]
pub enum UlpCoreWakeupSource {
    /// Wakeup source from the HP (High Performance) CPU.
    HpCpu,
    /// Wakeup after the ULP Timer has elapsed.
    /// The actual period between wake-ups is affected by the runtime duration of the ULP program.
    Timer(UlpCoreTimerCycles),
}

/// ULP Timer cycles are clocked at a rate of approximately 8MHz / 32768  = ~244 Hz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct UlpCoreTimerCycles {
    cycles: u32,
}
impl UlpCoreTimerCycles {
    /// Creates a new Ulp Timer cycle count configuration.
    /// ## Panics
    ///
    /// Panics if the cycles value is outside of the value range (0 ..= 0xFFFFFF).
    pub const fn new(cycles: u32) -> Self {
        ::core::assert!(
            cycles <= 0xFFFFFF,
            "ULP Timer cycles must be between 0 and 0xFFFFFF (inclusive)."
        );
        Self { cycles }
    }
    fn cycles(self) -> u32 {
        self.cycles
    }
}
impl Default for UlpCoreTimerCycles {
    fn default() -> Self {
        // ESP32-S2 Technical Reference Manual. Register 1.2. RTC_CNTL_ULP_CP_TIMER_1_REG (0x0130)
        // Field RTC_CNTL_ULP_CP_TIMER_SLP_CYCLE has a default value of 200 cycles.
        Self { cycles: 200 }
    }
}

/// Structure representing the ULP (Ultra-Low Power) core.
pub struct UlpCore<'d> {
    _lp_core: crate::peripherals::ULP_RISCV_CORE<'d>,
}

impl<'d> UlpCore<'d> {
    /// Creates a new instance of the `UlpCore` struct.
    pub fn new(lp_core: crate::peripherals::ULP_RISCV_CORE<'d>) -> Self {
        // clear all of RTC_SLOW_RAM - this makes sure .bss is cleared without relying
        let lp_ram =
            unsafe { core::slice::from_raw_parts_mut(0x5000_0000 as *mut u32, 8 * 1024 / 4) };
        lp_ram.fill(0u32);

        Self { _lp_core: lp_core }
    }

    // currently stopping the ULP doesn't work (while following the procedures
    // outlined in the TRM) - so don't offer this function for now
    //
    // pub fn stop(&mut self) {
    //     ulp_stop();
    // }

    /// Runs the ULP core with the specified wakeup source.
    pub fn run(&mut self, wakeup_src: UlpCoreWakeupSource) {
        ulp_run(wakeup_src);
    }

    /// Lets the ULP core wake the chip from sleep.
    ///
    /// The request stands until [`Self::disable_wakeup`] ends it: it survives a sleep, a
    /// deep-sleep wake, and this driver's `Drop`. It does nothing while the chip is awake.
    pub fn enable_wakeup(&mut self, config: WakeupConfig) {
        enable_wakeup(config);
    }

    /// Stops the ULP core from waking the chip.
    pub fn disable_wakeup(&mut self) {
        WakeupSource::UlpRiscv.disable();
        WakeupSource::UlpRiscvTrap.disable();
    }
}

/// Configures what the ULP core wakes the chip on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct WakeupConfig {
    /// Wakes the chip when the ULP core raises a software interrupt.
    software_interrupt: bool,
    /// Wakes the chip when the ULP core traps.
    trap: bool,
}

impl Default for WakeupConfig {
    fn default() -> Self {
        Self {
            software_interrupt: true,
            trap: true,
        }
    }
}

fn enable_wakeup(config: WakeupConfig) {
    for (source, enable) in [
        (WakeupSource::UlpRiscv, config.software_interrupt),
        (WakeupSource::UlpRiscvTrap, config.trap),
    ] {
        if enable {
            source.enable_with_hooks(Some(keep_low_power_domain), None);
        } else {
            source.disable();
        }
    }
}

/// The ULP core runs from the low-power memory and reads its pads through the low-power
/// peripherals, so a sleep that powers either down leaves it unable to raise the wake.
#[crate::ram]
fn keep_low_power_domain(_kind: SleepKind, config: &mut WrappedSleepConfig<'_>) {
    config.keep_alive(SleepResource::LpMemory);
    // ESP-IDF keeps the low-power peripherals on for `ext0` and GPIO wake only, but powering them
    // down stops the ULP timer and the ULP's own GPIO from working.
    // https://github.com/espressif/esp-idf/issues/10595
    config.keep_alive(SleepResource::LpPeripherals);
}

#[allow(unused)] // TODO: remove cfg when implementation is corrected
fn ulp_stop() {
    LPWR::regs()
        .ulp_cp_timer()
        .modify(|_, w| w.ulp_cp_slp_timer_en().clear_bit());

    // suspends the ulp operation
    LPWR::regs()
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_done().set_bit());

    // Resets the processor
    LPWR::regs()
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_shut_reset_en().set_bit());
}

fn ulp_run(wakeup_src: UlpCoreWakeupSource) {
    let rtc_cntl = LPWR::regs();

    // Reset COCPU when power on
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_shut_reset_en().set_bit());

    // Disable ULP timer
    rtc_cntl
        .ulp_cp_timer()
        .modify(|_, w| w.ulp_cp_slp_timer_en().clear_bit());

    // wait for at least 1 RTC_SLOW_CLK cycle
    crate::rom::ets_delay_us(20);

    // Select ULP-RISC-V to send the DONE signal
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_done_force().set_bit());

    ulp_config_wakeup_source(wakeup_src);

    // Select RISC-V as the ULP_TIMER trigger target
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_sel().clear_bit());

    // Clear any spurious wakeup trigger interrupts upon ULP startup
    crate::rom::ets_delay_us(20);

    rtc_cntl.int_clr().write(|w| {
        w.cocpu()
            .clear_bit_by_one()
            .cocpu_trap()
            .clear_bit_by_one()
            .ulp_cp()
            .clear_bit_by_one()
    });
}

fn ulp_config_wakeup_source(wakeup_src: UlpCoreWakeupSource) {
    // ESP-IDF source: https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_riscv.c#L87
    fn configure_timer(cycles: u32) {
        LPWR::regs()
            .ulp_cp_timer_1()
            .write(|w| unsafe { w.ulp_cp_timer_slp_cycle().bits(cycles << 8) });
        // enable the timer
        LPWR::regs()
            .ulp_cp_ctrl()
            .modify(|_, w| w.ulp_cp_force_start_top().clear_bit());
        LPWR::regs()
            .ulp_cp_timer()
            .modify(|_, w| w.ulp_cp_slp_timer_en().set_bit());
    }
    match wakeup_src {
        UlpCoreWakeupSource::HpCpu => {
            // wake-up immediately
            configure_timer(0);
        }
        UlpCoreWakeupSource::Timer(sleep_cycles) => {
            // configure timer duration
            configure_timer(sleep_cycles.cycles());
        }
    }
}
