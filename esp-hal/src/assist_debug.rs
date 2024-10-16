//! # Debug Assistant (ASSIST_DEBUG)
//!
//! ## Overview
//! Debug Assistant is an auxiliary module that features a set of functions to
//! help locate bugs and issues during software debugging. It includes
//! capabilities such as monitoring stack pointer (SP), monitoring memory
//! regions, and handling interrupts related to debugging.
//!
//!
//! ## Configuration
//! While all the targets support program counter (PC) logging it's API is not
//! exposed here. Instead the ROM bootloader will always enable it and print the
//! last seen PC (e.g. _Saved PC:0x42002ff2_). Make sure the reset was triggered
//! by a TIMG watchdog. Not an RTC or SWD watchdog.
//!
//! ## Examples
//! Visit the [Debug Assist] example for an example of using the Debug
//! Assistant.
//!
//! [Debug Assist]: https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/debug_assist.rs
//!
//! ## Implementation State
//! - Bus write access logging is not available via this API
//! - This driver has only blocking API

use crate::{
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::ASSIST_DEBUG,
    InterruptConfigurable,
};

/// The debug assist driver instance.
pub struct DebugAssist<'d> {
    debug_assist: PeripheralRef<'d, ASSIST_DEBUG>,
}

impl<'d> DebugAssist<'d> {
    /// Create a new instance in [crate::Blocking] mode.
    pub fn new(debug_assist: impl Peripheral<P = ASSIST_DEBUG> + 'd) -> Self {
        crate::into_ref!(debug_assist);

        // NOTE: We should enable the debug assist, however, it's always enabled in ROM
        //       code already.

        DebugAssist { debug_assist }
    }
}

impl crate::private::Sealed for DebugAssist<'_> {}

impl InterruptConfigurable for DebugAssist<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(
                crate::peripherals::Interrupt::ASSIST_DEBUG,
                handler.handler(),
            );
            crate::interrupt::enable(
                crate::peripherals::Interrupt::ASSIST_DEBUG,
                handler.priority(),
            )
            .unwrap();
        }
    }
}

#[cfg(assist_debug_sp_monitor)]
impl DebugAssist<'_> {
    /// Enable SP monitoring on main core. When the SP exceeds the
    /// `lower_bound` or `upper_bound` threshold, the module will record the PC
    /// pointer and generate an interrupt.
    pub fn enable_sp_monitor(&mut self, lower_bound: u32, upper_bound: u32) {
        self.debug_assist
            .core_0_sp_min()
            .write(|w| unsafe { w.core_0_sp_min().bits(lower_bound) });

        self.debug_assist
            .core_0_sp_max()
            .write(|w| unsafe { w.core_0_sp_max().bits(upper_bound) });

        self.debug_assist.core_0_montr_ena().modify(|_, w| {
            w.core_0_sp_spill_min_ena()
                .set_bit()
                .core_0_sp_spill_max_ena()
                .set_bit()
        });

        self.clear_sp_monitor_interrupt();

        self.debug_assist.core_0_intr_ena().modify(|_, w| {
            w.core_0_sp_spill_max_intr_ena()
                .set_bit()
                .core_0_sp_spill_min_intr_ena()
                .set_bit()
        });
    }

    /// Disable SP monitoring on main core.
    pub fn disable_sp_monitor(&mut self) {
        self.debug_assist.core_0_intr_ena().modify(|_, w| {
            w.core_0_sp_spill_max_intr_ena()
                .clear_bit()
                .core_0_sp_spill_min_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_0_montr_ena().modify(|_, w| {
            w.core_0_sp_spill_min_ena()
                .clear_bit()
                .core_0_sp_spill_max_ena()
                .clear_bit()
        });
    }

    /// Clear SP monitoring interrupt on main core.
    pub fn clear_sp_monitor_interrupt(&mut self) {
        self.debug_assist.core_0_intr_clr().write(|w| {
            w.core_0_sp_spill_max_clr()
                .set_bit()
                .core_0_sp_spill_min_clr()
                .set_bit()
        });
    }

    /// Check, if SP monitoring interrupt is set on main core.
    pub fn is_sp_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_0_intr_raw()
            .read()
            .core_0_sp_spill_max_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_0_intr_raw()
                .read()
                .core_0_sp_spill_min_raw()
                .bit_is_set()
    }

    /// Get SP monitoring PC value on main core.
    pub fn get_sp_monitor_pc(&self) -> u32 {
        self.debug_assist
            .core_0_sp_pc()
            .read()
            .core_0_sp_pc()
            .bits()
    }
}

#[cfg(all(assist_debug_sp_monitor, multi_core))]
impl<'d> DebugAssist<'d> {
    /// Enable SP monitoring on secondary core. When the SP exceeds the
    /// `lower_bound` or `upper_bound` threshold, the module will record the PC
    /// pointer and generate an interrupt.
    pub fn enable_core1_sp_monitor(&mut self, lower_bound: u32, upper_bound: u32) {
        self.debug_assist
            .core_1_sp_min
            .write(|w| w.core_1_sp_min().bits(lower_bound));

        self.debug_assist
            .core_1_sp_max
            .write(|w| w.core_1_sp_max().bits(upper_bound));

        self.debug_assist.core_1_montr_ena.modify(|_, w| {
            w.core_1_sp_spill_min_ena()
                .set_bit()
                .core_1_sp_spill_max_ena()
                .set_bit()
        });

        self.clear_core1_sp_monitor_interrupt();

        self.debug_assist.core_1_intr_ena.modify(|_, w| {
            w.core_1_sp_spill_max_intr_ena()
                .set_bit()
                .core_1_sp_spill_min_intr_ena()
                .set_bit()
        });
    }

    /// Disable SP monitoring on secondary core.
    pub fn disable_core1_sp_monitor(&mut self) {
        self.debug_assist.core_1_intr_ena.modify(|_, w| {
            w.core_1_sp_spill_max_intr_ena()
                .clear_bit()
                .core_1_sp_spill_min_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_1_montr_ena.modify(|_, w| {
            w.core_1_sp_spill_min_ena()
                .clear_bit()
                .core_1_sp_spill_max_ena()
                .clear_bit()
        });
    }

    /// Clear SP monitoring interrupt on secondary core.
    pub fn clear_core1_sp_monitor_interrupt(&mut self) {
        self.debug_assist.core_1_intr_clr.write(|w| {
            w.core_1_sp_spill_max_clr()
                .set_bit()
                .core_1_sp_spill_min_clr()
                .set_bit()
        });
    }

    /// Check, if SP monitoring interrupt is set on secondary core.
    pub fn is_core1_sp_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_1_intr_raw
            .read()
            .core_1_sp_spill_max_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_1_intr_raw
                .read()
                .core_1_sp_spill_min_raw()
                .bit_is_set()
    }

    /// Get SP monitoring PC value on secondary core.
    pub fn get_core1_sp_monitor_pc(&self) -> u32 {
        self.debug_assist.core_1_sp_pc.read().core_1_sp_pc().bits()
    }
}

#[cfg(assist_debug_region_monitor)]
impl DebugAssist<'_> {
    /// Enable region monitoring of read/write performed by the main CPU in a
    /// certain memory region0. Whenever the bus reads or writes in the
    /// specified memory region, an interrupt will be triggered. Two memory
    /// regions (region0, region1) can be monitored at the same time.
    pub fn enable_region0_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_0_area_dram0_0_min()
            .write(|w| unsafe { w.core_0_area_dram0_0_min().bits(lower_bound) });

        self.debug_assist
            .core_0_area_dram0_0_max()
            .write(|w| unsafe { w.core_0_area_dram0_0_max().bits(upper_bound) });

        self.debug_assist.core_0_montr_ena().modify(|_, w| {
            w.core_0_area_dram0_0_rd_ena()
                .bit(reads)
                .core_0_area_dram0_0_wr_ena()
                .bit(writes)
        });

        self.clear_region0_monitor_interrupt();

        self.debug_assist.core_0_intr_ena().modify(|_, w| {
            w.core_0_area_dram0_0_rd_intr_ena()
                .set_bit()
                .core_0_area_dram0_0_wr_intr_ena()
                .set_bit()
        });
    }

    /// Disable region0 monitoring on main core.
    pub fn disable_region0_monitor(&mut self) {
        self.debug_assist.core_0_intr_ena().modify(|_, w| {
            w.core_0_area_dram0_0_rd_intr_ena()
                .clear_bit()
                .core_0_area_dram0_0_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_0_montr_ena().modify(|_, w| {
            w.core_0_area_dram0_0_rd_ena()
                .clear_bit()
                .core_0_area_dram0_0_wr_ena()
                .clear_bit()
        });
    }

    /// Clear region0 monitoring interrupt on main core.
    pub fn clear_region0_monitor_interrupt(&mut self) {
        self.debug_assist.core_0_intr_clr().write(|w| {
            w.core_0_area_dram0_0_rd_clr()
                .set_bit()
                .core_0_area_dram0_0_wr_clr()
                .set_bit()
        });
    }

    /// Check, if region0 monitoring interrupt is set on main core.
    pub fn is_region0_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_0_intr_raw()
            .read()
            .core_0_area_dram0_0_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_0_intr_raw()
                .read()
                .core_0_area_dram0_0_wr_raw()
                .bit_is_set()
    }

    /// Enable region monitoring of read/write performed by the main CPU in a
    /// certain memory region1. Whenever the bus reads or writes in the
    /// specified memory region, an interrupt will be triggered.
    pub fn enable_region1_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_0_area_dram0_1_min()
            .write(|w| unsafe { w.core_0_area_dram0_1_min().bits(lower_bound) });

        self.debug_assist
            .core_0_area_dram0_1_max()
            .write(|w| unsafe { w.core_0_area_dram0_1_max().bits(upper_bound) });

        self.debug_assist.core_0_montr_ena().modify(|_, w| {
            w.core_0_area_dram0_1_rd_ena()
                .bit(reads)
                .core_0_area_dram0_1_wr_ena()
                .bit(writes)
        });

        self.clear_region1_monitor_interrupt();

        self.debug_assist.core_0_intr_ena().modify(|_, w| {
            w.core_0_area_dram0_1_rd_intr_ena()
                .set_bit()
                .core_0_area_dram0_1_wr_intr_ena()
                .set_bit()
        });
    }

    /// Disable region1 monitoring on main core.
    pub fn disable_region1_monitor(&mut self) {
        self.debug_assist.core_0_intr_ena().modify(|_, w| {
            w.core_0_area_dram0_1_rd_intr_ena()
                .clear_bit()
                .core_0_area_dram0_1_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_0_montr_ena().modify(|_, w| {
            w.core_0_area_dram0_1_rd_ena()
                .clear_bit()
                .core_0_area_dram0_1_wr_ena()
                .clear_bit()
        });
    }

    /// Clear region1 monitoring interrupt on main core.
    pub fn clear_region1_monitor_interrupt(&mut self) {
        self.debug_assist.core_0_intr_clr().write(|w| {
            w.core_0_area_dram0_1_rd_clr()
                .set_bit()
                .core_0_area_dram0_1_wr_clr()
                .set_bit()
        });
    }

    /// Check, if region1 monitoring interrupt is set on main core.
    pub fn is_region1_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_0_intr_raw()
            .read()
            .core_0_area_dram0_1_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_0_intr_raw()
                .read()
                .core_0_area_dram0_1_wr_raw()
                .bit_is_set()
    }

    /// Get region monitoring PC value on main core.
    pub fn get_region_monitor_pc(&self) -> u32 {
        self.debug_assist
            .core_0_area_pc()
            .read()
            .core_0_area_pc()
            .bits()
    }
}

#[cfg(all(assist_debug_region_monitor, multi_core))]
impl<'d> DebugAssist<'d> {
    /// Enable region monitoring of read/write performed by the secondary CPU in
    /// a certain memory region0. Whenever the bus reads or writes in the
    /// specified memory region, an interrupt will be triggered.
    pub fn enable_core1_region0_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_1_area_dram0_0_min()
            .write(|w| unsafe { w.core_1_area_dram0_0_min().bits(lower_bound) });

        self.debug_assist
            .core_1_area_dram0_0_max()
            .write(|w| unsafe { w.core_1_area_dram0_0_max().bits(upper_bound) });

        self.debug_assist.core_1_montr_ena().modify(|_, w| {
            w.core_1_area_dram0_0_rd_ena()
                .bit(reads)
                .core_1_area_dram0_0_wr_ena()
                .bit(writes)
        });

        self.clear_core1_region0_monitor_interrupt();

        self.debug_assist.core_1_intr_ena().modify(|_, w| {
            w.core_1_area_dram0_0_rd_intr_ena()
                .set_bit()
                .core_1_area_dram0_0_wr_intr_ena()
                .set_bit()
        });
    }

    /// Disable region0 monitoring on secondary core.
    pub fn disable_core1_region0_monitor(&mut self) {
        self.debug_assist.core_1_intr_ena().modify(|_, w| {
            w.core_1_area_dram0_0_rd_intr_ena()
                .clear_bit()
                .core_1_area_dram0_0_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_1_montr_ena().modify(|_, w| {
            w.core_1_area_dram0_0_rd_ena()
                .clear_bit()
                .core_1_area_dram0_0_wr_ena()
                .clear_bit()
        });
    }

    /// Clear region0 monitoring interrupt on secondary core.
    pub fn clear_core1_region0_monitor_interrupt(&mut self) {
        self.debug_assist.core_1_intr_clr().write(|w| {
            w.core_1_area_dram0_0_rd_clr()
                .set_bit()
                .core_1_area_dram0_0_wr_clr()
                .set_bit()
        });
    }

    /// Check, if region0 monitoring interrupt is set on secondary core.
    pub fn is_core1_region0_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_1_intr_raw()
            .read()
            .core_1_area_dram0_0_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_1_intr_raw()
                .read()
                .core_1_area_dram0_0_wr_raw()
                .bit_is_set()
    }

    /// Enable region monitoring of read/write performed by the secondary CPU in
    /// a certain memory region1. Whenever the bus reads or writes in the
    /// specified memory region, an interrupt will be triggered.
    pub fn enable_core1_region1_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_1_area_dram0_1_min()
            .write(|w| unsafe { w.core_1_area_dram0_1_min().bits(lower_bound) });

        self.debug_assist
            .core_1_area_dram0_1_max()
            .write(|w| unsafe { w.core_1_area_dram0_1_max().bits(upper_bound) });

        self.debug_assist.core_1_montr_ena().modify(|_, w| {
            w.core_1_area_dram0_1_rd_ena()
                .bit(reads)
                .core_1_area_dram0_1_wr_ena()
                .bit(writes)
        });

        self.clear_core1_region1_monitor_interrupt();

        self.debug_assist.core_1_intr_ena().modify(|_, w| {
            w.core_1_area_dram0_1_rd_intr_ena()
                .set_bit()
                .core_1_area_dram0_1_wr_intr_ena()
                .set_bit()
        });
    }

    /// Disable region1 monitoring on secondary core.
    pub fn disable_core1_region1_monitor(&mut self) {
        self.debug_assist.core_1_intr_ena().modify(|_, w| {
            w.core_1_area_dram0_1_rd_intr_ena()
                .clear_bit()
                .core_1_area_dram0_1_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_1_montr_ena().modify(|_, w| {
            w.core_1_area_dram0_1_rd_ena()
                .clear_bit()
                .core_1_area_dram0_1_wr_ena()
                .clear_bit()
        });
    }

    /// Clear region1 monitoring interrupt on secondary core.
    pub fn clear_core1_region1_monitor_interrupt(&mut self) {
        self.debug_assist.core_1_intr_clr().write(|w| {
            w.core_1_area_dram0_1_rd_clr()
                .set_bit()
                .core_1_area_dram0_1_wr_clr()
                .set_bit()
        });
    }

    /// Check, if region1 monitoring interrupt is set on secondary core.
    pub fn is_core1_region1_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_1_intr_raw()
            .read()
            .core_1_area_dram0_1_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_1_intr_raw()
                .read()
                .core_1_area_dram0_1_wr_raw()
                .bit_is_set()
    }

    /// Get region monitoring PC value on secondary core.
    pub fn get_core1_region_monitor_pc(&self) -> u32 {
        self.debug_assist
            .core_1_area_pc()
            .read()
            .core_1_area_pc()
            .bits()
    }
}
