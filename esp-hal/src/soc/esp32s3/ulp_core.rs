//! Control the ULP core
//!
//! ## Overview
//!
//! The `ULP CORE` peripheral allows control over the `Ultra-Low Power
//! (ULP) core` in `ESP` chips. The ULP core is a low-power processor
//! designed for executing tasks in deep sleep mode, enabling efficient power
//! management in ESP systems.
//!
//! The `UlpCore` struct provides an interface to interact with the `ULP
//! CORE` peripheral. It allows starting and configuring the ULP core for
//! operation. The `UlpCore` struct is initialized with a peripheral reference
//! to the `ULP CORE` instance.
//!
//! ## Examples
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! const CODE: &[u8] = &[
//!     0x17, 0x05, 0x00, 0x00, 0x13, 0x05, 0x05, 0x01, 0x81, 0x45, 0x85, 0x05,
//!     0x0c, 0xc1, 0xf5, 0xbf, 0x00, 0x00, 0x00, 0x00,
//! ];
//!
//! let mut ulp_core =
//!     esp_hal::ulp_core::UlpCore::new(peripherals.ULP_RISCV_CORE);
//! ulp_core.stop();
//!
//! // copy code to RTC ram
//! let lp_ram = 0x5000_0000 as *mut u8;
//! unsafe {
//!     core::ptr::copy_nonoverlapping(
//!         CODE as *const _ as *const u8,
//!         lp_ram,
//!         CODE.len(),
//!     );
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

use esp32s3 as pac;

use crate::peripheral::{Peripheral, PeripheralRef};

/// Enum representing the possible wakeup sources for the ULP core.
#[derive(Debug, Clone, Copy)]
pub enum UlpCoreWakeupSource {
    /// Wakeup source from the HP (High Performance) CPU.
    HpCpu,
}

/// Structure representing the ULP (Ultra-Low Power) core.
pub struct UlpCore<'d> {
    _lp_core: PeripheralRef<'d, crate::soc::peripherals::ULP_RISCV_CORE>,
}

impl<'d> UlpCore<'d> {
    /// Creates a new instance of the `UlpCore` struct.
    pub fn new(lp_core: impl Peripheral<P = crate::soc::peripherals::ULP_RISCV_CORE> + 'd) -> Self {
        crate::into_ref!(lp_core);

        let mut this = Self { _lp_core: lp_core };
        this.stop();

        // clear all of RTC_SLOW_RAM - this makes sure .bss is cleared without relying
        let lp_ram =
            unsafe { core::slice::from_raw_parts_mut(0x5000_0000 as *mut u32, 8 * 1024 / 4) };
        lp_ram.fill(0u32);

        this
    }

    /// Stops the ULP core.
    pub fn stop(&mut self) {
        ulp_stop();
    }

    /// Runs the ULP core with the specified wakeup source.
    pub fn run(&mut self, wakeup_src: UlpCoreWakeupSource) {
        ulp_run(wakeup_src);
    }
}

fn ulp_stop() {
    let rtc_cntl = unsafe { &*pac::RTC_CNTL::PTR };
    rtc_cntl
        .ulp_cp_timer()
        .modify(|_, w| w.ulp_cp_slp_timer_en().clear_bit());

    // suspends the ulp operation
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_done().set_bit());

    // Resets the processor
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_shut_reset_en().set_bit());

    crate::rom::ets_delay_us(20);

    // above doesn't seem to halt the ULP core - this will
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_clkgate_en().clear_bit());
}

fn ulp_run(wakeup_src: UlpCoreWakeupSource) {
    let rtc_cntl = unsafe { &*pac::RTC_CNTL::PTR };

    // Reset COCPU when power on
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_shut_reset_en().set_bit());

    // The coprocessor cpu trap signal doesn't have a stable reset value,
    // force ULP-RISC-V clock on to stop RTC_COCPU_TRAP_TRIG_EN from waking the CPU
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_clk_fo().set_bit());

    // Disable ULP timer
    rtc_cntl
        .ulp_cp_timer()
        .modify(|_, w| w.ulp_cp_slp_timer_en().clear_bit());

    // wait for at least 1 RTC_SLOW_CLK cycle
    crate::rom::ets_delay_us(20);

    // We do not select RISC-V as the Coprocessor here as this could lead to a hang
    // in the main CPU. Instead, we reset RTC_CNTL_COCPU_SEL after we have enabled
    // the ULP timer.
    //
    // IDF-4510

    // Select ULP-RISC-V to send the DONE signal
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_done_force().set_bit());

    // Set the CLKGATE_EN signal
    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_clkgate_en().set_bit());

    ulp_config_wakeup_source(wakeup_src);

    // Select RISC-V as the ULP_TIMER trigger target
    // Selecting the RISC-V as the Coprocessor at the end is a workaround
    // for the hang issue recorded in IDF-4510.
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

    rtc_cntl
        .cocpu_ctrl()
        .modify(|_, w| w.cocpu_clkgate_en().set_bit());
}

fn ulp_config_wakeup_source(wakeup_src: UlpCoreWakeupSource) {
    match wakeup_src {
        UlpCoreWakeupSource::HpCpu => {
            // use timer to wake up
            let rtc_cntl = unsafe { &*pac::RTC_CNTL::PTR };
            rtc_cntl
                .ulp_cp_ctrl()
                .modify(|_, w| w.ulp_cp_force_start_top().clear_bit());
            rtc_cntl
                .ulp_cp_timer()
                .modify(|_, w| w.ulp_cp_slp_timer_en().set_bit());
        }
    }
}
