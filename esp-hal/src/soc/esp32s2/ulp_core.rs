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
//! let mut ulp_core = esp_hal::ulp_core::UlpCore::new(peripherals.ULP_RISCV_CORE);
//! // ulp_core.stop(); currently not implemented for ESP32-S2
//!
//! // copy code to RTC ram
//! let lp_ram = 0x5000_0000 as *mut u8;
//! unsafe {
//!     core::ptr::copy_nonoverlapping(CODE as *const _ as *const u8, lp_ram, CODE.len());
//! }
//!
//! // start ULP core
//! ulp_core.run(esp_hal::ulp_core::UlpCoreWakeupSource::HpCpu);
//!
//! unsafe {
//!     let data = 0x5000_0010 as *mut u32;
//!     loop {}
//! }
//! # }
//! ```

use crate::peripherals::LPWR;

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
    match wakeup_src {
        UlpCoreWakeupSource::HpCpu => {
            // only wake-up when the HpCpu calls .run()
        }
        UlpCoreWakeupSource::Timer(sleep_cycles) => {
            // configure timer duration
            let cycles = sleep_cycles.cycles() << 8;
            LPWR::regs()
                .ulp_cp_timer_1()
                .write(|w| unsafe { w.ulp_cp_timer_slp_cycle().bits(cycles) });
            // enable the timer
            LPWR::regs()
                .ulp_cp_ctrl()
                .modify(|_, w| w.ulp_cp_force_start_top().clear_bit());
            LPWR::regs()
                .ulp_cp_timer()
                .modify(|_, w| w.ulp_cp_slp_timer_en().set_bit());
        }
    }
}
