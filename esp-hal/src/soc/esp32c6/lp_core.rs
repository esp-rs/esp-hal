//! # Control the LP core
//!
//! ## Overview
//! The `LP_CORE` driver provides an interface for controlling and managing the
//! low power core of `ESP` chips, allowing efficient low power operation and
//! wakeup from sleep based on configurable sources. The low power core is
//! responsible for executing low power tasks while the high power core is in
//! sleep mode.
//!
//! The `LpCore` struct provides methods to stop and run the low power core.
//!
//! The `stop` method stops the low power core, putting it into a sleep state.
//!
//! The `run` method starts the low power core and specifies the wakeup source.
//!
//! ⚠️: The examples for LP Core are quite extensive, so for a more
//! detailed study of how to use this LP Core please visit [the repository
//! with corresponding example].
//!
//! [the repository with corresponding example]: https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/lp_core_basic.rs

use esp32c6 as pac;

use crate::peripheral::{Peripheral, PeripheralRef};

/// Represents the possible wakeup sources for the LP (Low Power) core.
#[derive(Debug, Clone, Copy)]
pub enum LpCoreWakeupSource {
    /// Wakeup source from the HP (High Performance) CPU.
    HpCpu,
}

/// Clock sources for the LP core
#[derive(Debug, Clone, Copy)]
pub enum LpCoreClockSource {
    /// 17.5 MHz clock
    ///
    /// Might not be very accurate
    RcFastClk,
    /// 20 MHz clock
    XtalD2Clk,
}

/// Represents the Low Power (LP) core peripheral.
pub struct LpCore<'d> {
    _lp_core: PeripheralRef<'d, crate::soc::peripherals::LP_CORE>,
}

impl<'d> LpCore<'d> {
    /// Create a new instance using [LpCoreClockSource::RcFastClk]
    pub fn new(lp_core: impl Peripheral<P = crate::soc::peripherals::LP_CORE> + 'd) -> Self {
        LpCore::new_with_clock(lp_core, LpCoreClockSource::RcFastClk)
    }

    /// Create a new instance using the given clock
    pub fn new_with_clock(
        lp_core: impl Peripheral<P = crate::soc::peripherals::LP_CORE> + 'd,
        clk_src: LpCoreClockSource,
    ) -> Self {
        crate::into_ref!(lp_core);

        match clk_src {
            LpCoreClockSource::RcFastClk => unsafe {
                (*crate::soc::peripherals::LP_CLKRST::PTR)
                    .lp_clk_conf()
                    .modify(|_, w| w.fast_clk_sel().clear_bit());
            },
            LpCoreClockSource::XtalD2Clk => unsafe {
                (*crate::soc::peripherals::LP_CLKRST::PTR)
                    .lp_clk_conf()
                    .modify(|_, w| w.fast_clk_sel().set_bit());
            },
        }

        let mut this = Self { _lp_core: lp_core };
        this.stop();

        // clear all of LP_RAM - this makes sure .bss is cleared without relying
        let lp_ram =
            unsafe { core::slice::from_raw_parts_mut(0x5000_0000 as *mut u32, 16 * 1024 / 4) };
        lp_ram.fill(0u32);

        this
    }

    /// Stop the LP core
    pub fn stop(&mut self) {
        ulp_lp_core_stop();
    }

    /// Start the LP core
    pub fn run(&mut self, wakeup_src: LpCoreWakeupSource) {
        ulp_lp_core_run(wakeup_src);
    }
}

fn ulp_lp_core_stop() {
    let pmu = unsafe { &*pac::PMU::PTR };
    pmu.lp_cpu_pwr1()
        .modify(|_, w| unsafe { w.lp_cpu_wakeup_en().bits(0) });
    pmu.lp_cpu_pwr1()
        .modify(|_, w| w.lp_cpu_sleep_req().set_bit());
}

fn ulp_lp_core_run(wakeup_src: LpCoreWakeupSource) {
    let lp_aon = unsafe { &*pac::LP_AON::PTR };
    let pmu = unsafe { &*pac::PMU::PTR };
    let lp_peri = unsafe { &*pac::LP_PERI::PTR };

    // Enable LP-Core
    lp_aon.lpcore().modify(|_, w| w.disable().clear_bit());

    // Allow LP core to access LP memory during sleep
    lp_aon
        .lpbus()
        .modify(|_, w| w.fast_mem_mux_sel().clear_bit());
    lp_aon
        .lpbus()
        .modify(|_, w| w.fast_mem_mux_sel_update().set_bit());

    // Enable stall at sleep request
    pmu.lp_cpu_pwr0()
        .modify(|_, w| w.lp_cpu_slp_stall_en().set_bit());

    // Enable reset after wake-up
    pmu.lp_cpu_pwr0()
        .modify(|_, w| w.lp_cpu_slp_reset_en().set_bit());

    // Set wake-up sources
    let src = match wakeup_src {
        LpCoreWakeupSource::HpCpu => 0x01,
    };
    pmu.lp_cpu_pwr1()
        .modify(|_, w| unsafe { w.lp_cpu_wakeup_en().bits(src) });

    // Enable JTAG debugging
    lp_peri
        .cpu()
        .modify(|_, w| w.lpcore_dbgm_unavaliable().clear_bit());

    // wake up
    match wakeup_src {
        LpCoreWakeupSource::HpCpu => {
            pmu.hp_lp_cpu_comm().write(|w| w.hp_trigger_lp().set_bit());
        }
    }
}
