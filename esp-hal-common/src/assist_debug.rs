//! Debug Assistant
//!
//! Debug Assistant is an auxiliary module that features a set of functions to
//! help locate bugs and issues during software debugging.
//!
//! While all the targets support PC logging it's API is not exposed here.
//! Instead the ROM bootloader will always enable it and print the last seen PC
//! (e.g. _Saved PC:0x42002ff2_). Make sure the reset was triggered by a TIMG
//! watchdog. Not an RTC or SWD watchdog.
//!
//! Not all targets support all the features.
//!  
//! Bus write access logging is not available via this API.

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
};

pub struct DebugAssist<'d> {
    debug_assist: PeripheralRef<'d, crate::peripherals::ASSIST_DEBUG>,
}

impl<'d> DebugAssist<'d> {
    pub fn new(
        debug_assist: impl Peripheral<P = crate::peripherals::ASSIST_DEBUG> + 'd,
        _peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(debug_assist);

        // we should use peripheral clock control to enable the debug assist however
        // it's always enabled in ROM code already

        DebugAssist { debug_assist }
    }
}

#[cfg(assist_debug_sp_monitor)]
impl<'d> DebugAssist<'d> {
    pub fn enable_sp_monitor(&mut self, lower_bound: u32, upper_bound: u32) {
        self.debug_assist
            .core_0_sp_min
            .write(|w| w.core_0_sp_min().variant(lower_bound));

        self.debug_assist
            .core_0_sp_max
            .write(|w| w.core_0_sp_max().variant(upper_bound));

        self.debug_assist.core_0_montr_ena.modify(|_, w| {
            w.core_0_sp_spill_min_ena()
                .set_bit()
                .core_0_sp_spill_max_ena()
                .set_bit()
        });

        self.clear_sp_monitor_interrupt();

        self.debug_assist.core_0_intr_ena.modify(|_, w| {
            w.core_0_sp_spill_max_intr_ena()
                .set_bit()
                .core_0_sp_spill_min_intr_ena()
                .set_bit()
        });
    }

    pub fn disable_sp_monitor(&mut self) {
        self.debug_assist.core_0_intr_ena.modify(|_, w| {
            w.core_0_sp_spill_max_intr_ena()
                .clear_bit()
                .core_0_sp_spill_min_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_0_montr_ena.modify(|_, w| {
            w.core_0_sp_spill_min_ena()
                .clear_bit()
                .core_0_sp_spill_max_ena()
                .clear_bit()
        });
    }

    pub fn clear_sp_monitor_interrupt(&mut self) {
        self.debug_assist.core_0_intr_clr.write(|w| {
            w.core_0_sp_spill_max_clr()
                .set_bit()
                .core_0_sp_spill_min_clr()
                .set_bit()
        });
    }

    pub fn is_sp_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_0_intr_raw
            .read()
            .core_0_sp_spill_max_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_0_intr_raw
                .read()
                .core_0_sp_spill_min_raw()
                .bit_is_set()
    }

    pub fn get_sp_monitor_pc(&self) -> u32 {
        self.debug_assist.core_0_sp_pc.read().core_0_sp_pc().bits()
    }
}

#[cfg(all(assist_debug_sp_monitor, multi_core))]
impl<'d> DebugAssist<'d> {
    pub fn enable_core1_sp_monitor(&mut self, lower_bound: u32, upper_bound: u32) {
        self.debug_assist
            .core_1_sp_min
            .write(|w| w.core_1_sp_min().variant(lower_bound));

        self.debug_assist
            .core_1_sp_max
            .write(|w| w.core_1_sp_max().variant(upper_bound));

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

    pub fn clear_core1_sp_monitor_interrupt(&mut self) {
        self.debug_assist.core_1_intr_clr.write(|w| {
            w.core_1_sp_spill_max_clr()
                .set_bit()
                .core_1_sp_spill_min_clr()
                .set_bit()
        });
    }

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

    pub fn get_core1_sp_monitor_pc(&self) -> u32 {
        self.debug_assist.core_1_sp_pc.read().core_1_sp_pc().bits()
    }
}

#[cfg(assist_debug_region_monitor)]
impl<'d> DebugAssist<'d> {
    pub fn enable_region0_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_0_area_dram0_0_min
            .write(|w| w.core_0_area_dram0_0_min().variant(lower_bound));

        self.debug_assist
            .core_0_area_dram0_0_max
            .write(|w| w.core_0_area_dram0_0_max().variant(upper_bound));

        self.debug_assist.core_0_montr_ena.modify(|_, w| {
            w.core_0_area_dram0_0_rd_ena()
                .bit(reads)
                .core_0_area_dram0_0_wr_ena()
                .bit(writes)
        });

        self.clear_region0_monitor_interrupt();

        self.debug_assist.core_0_intr_ena.modify(|_, w| {
            w.core_0_area_dram0_0_rd_intr_ena()
                .set_bit()
                .core_0_area_dram0_0_wr_intr_ena()
                .set_bit()
        });
    }

    pub fn disable_region0_monitor(&mut self) {
        self.debug_assist.core_0_intr_ena.modify(|_, w| {
            w.core_0_area_dram0_0_rd_intr_ena()
                .clear_bit()
                .core_0_area_dram0_0_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_0_montr_ena.modify(|_, w| {
            w.core_0_area_dram0_0_rd_ena()
                .clear_bit()
                .core_0_area_dram0_0_wr_ena()
                .clear_bit()
        });
    }

    pub fn clear_region0_monitor_interrupt(&mut self) {
        self.debug_assist.core_0_intr_clr.write(|w| {
            w.core_0_area_dram0_0_rd_clr()
                .set_bit()
                .core_0_area_dram0_0_wr_clr()
                .set_bit()
        });
    }

    pub fn is_region0_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_0_intr_raw
            .read()
            .core_0_area_dram0_0_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_0_intr_raw
                .read()
                .core_0_area_dram0_0_wr_raw()
                .bit_is_set()
    }

    pub fn enable_region1_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_0_area_dram0_1_min
            .write(|w| w.core_0_area_dram0_1_min().variant(lower_bound));

        self.debug_assist
            .core_0_area_dram0_1_max
            .write(|w| w.core_0_area_dram0_1_max().variant(upper_bound));

        self.debug_assist.core_0_montr_ena.modify(|_, w| {
            w.core_0_area_dram0_1_rd_ena()
                .bit(reads)
                .core_0_area_dram0_1_wr_ena()
                .bit(writes)
        });

        self.clear_region1_monitor_interrupt();

        self.debug_assist.core_0_intr_ena.modify(|_, w| {
            w.core_0_area_dram0_1_rd_intr_ena()
                .set_bit()
                .core_0_area_dram0_1_wr_intr_ena()
                .set_bit()
        });
    }

    pub fn disable_region1_monitor(&mut self) {
        self.debug_assist.core_0_intr_ena.modify(|_, w| {
            w.core_0_area_dram0_1_rd_intr_ena()
                .clear_bit()
                .core_0_area_dram0_1_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_0_montr_ena.modify(|_, w| {
            w.core_0_area_dram0_1_rd_ena()
                .clear_bit()
                .core_0_area_dram0_1_wr_ena()
                .clear_bit()
        });
    }

    pub fn clear_region1_monitor_interrupt(&mut self) {
        self.debug_assist.core_0_intr_clr.write(|w| {
            w.core_0_area_dram0_1_rd_clr()
                .set_bit()
                .core_0_area_dram0_1_wr_clr()
                .set_bit()
        });
    }

    pub fn is_region1_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_0_intr_raw
            .read()
            .core_0_area_dram0_1_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_0_intr_raw
                .read()
                .core_0_area_dram0_1_wr_raw()
                .bit_is_set()
    }

    pub fn get_region_monitor_pc(&self) -> u32 {
        self.debug_assist
            .core_0_area_pc
            .read()
            .core_0_area_pc()
            .bits()
    }
}

#[cfg(all(assist_debug_region_monitor, multi_core))]
impl<'d> DebugAssist<'d> {
    pub fn enable_core1_region0_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_1_area_dram0_0_min
            .write(|w| w.core_1_area_dram0_0_min().variant(lower_bound));

        self.debug_assist
            .core_1_area_dram0_0_max
            .write(|w| w.core_1_area_dram0_0_max().variant(upper_bound));

        self.debug_assist.core_1_montr_ena.modify(|_, w| {
            w.core_1_area_dram0_0_rd_ena()
                .bit(reads)
                .core_1_area_dram0_0_wr_ena()
                .bit(writes)
        });

        self.clear_core1_region0_monitor_interrupt();

        self.debug_assist.core_1_intr_ena.modify(|_, w| {
            w.core_1_area_dram0_0_rd_intr_ena()
                .set_bit()
                .core_1_area_dram0_0_wr_intr_ena()
                .set_bit()
        });
    }

    pub fn disable_core1_region0_monitor(&mut self) {
        self.debug_assist.core_1_intr_ena.modify(|_, w| {
            w.core_1_area_dram0_0_rd_intr_ena()
                .clear_bit()
                .core_1_area_dram0_0_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_1_montr_ena.modify(|_, w| {
            w.core_1_area_dram0_0_rd_ena()
                .clear_bit()
                .core_1_area_dram0_0_wr_ena()
                .clear_bit()
        });
    }

    pub fn clear_core1_region0_monitor_interrupt(&mut self) {
        self.debug_assist.core_1_intr_clr.write(|w| {
            w.core_1_area_dram0_0_rd_clr()
                .set_bit()
                .core_1_area_dram0_0_wr_clr()
                .set_bit()
        });
    }

    pub fn is_core1_region0_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_1_intr_raw
            .read()
            .core_1_area_dram0_0_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_1_intr_raw
                .read()
                .core_1_area_dram0_0_wr_raw()
                .bit_is_set()
    }

    pub fn enable_core1_region1_monitor(
        &mut self,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        self.debug_assist
            .core_1_area_dram0_1_min
            .write(|w| w.core_1_area_dram0_1_min().variant(lower_bound));

        self.debug_assist
            .core_1_area_dram0_1_max
            .write(|w| w.core_1_area_dram0_1_max().variant(upper_bound));

        self.debug_assist.core_1_montr_ena.modify(|_, w| {
            w.core_1_area_dram0_1_rd_ena()
                .bit(reads)
                .core_1_area_dram0_1_wr_ena()
                .bit(writes)
        });

        self.clear_core1_region1_monitor_interrupt();

        self.debug_assist.core_1_intr_ena.modify(|_, w| {
            w.core_1_area_dram0_1_rd_intr_ena()
                .set_bit()
                .core_1_area_dram0_1_wr_intr_ena()
                .set_bit()
        });
    }

    pub fn disable_core1_region1_monitor(&mut self) {
        self.debug_assist.core_1_intr_ena.modify(|_, w| {
            w.core_1_area_dram0_1_rd_intr_ena()
                .clear_bit()
                .core_1_area_dram0_1_wr_intr_ena()
                .clear_bit()
        });

        self.debug_assist.core_1_montr_ena.modify(|_, w| {
            w.core_1_area_dram0_1_rd_ena()
                .clear_bit()
                .core_1_area_dram0_1_wr_ena()
                .clear_bit()
        });
    }

    pub fn clear_core1_region1_monitor_interrupt(&mut self) {
        self.debug_assist.core_1_intr_clr.write(|w| {
            w.core_1_area_dram0_1_rd_clr()
                .set_bit()
                .core_1_area_dram0_1_wr_clr()
                .set_bit()
        });
    }

    pub fn is_core1_region1_monitor_interrupt_set(&self) -> bool {
        self.debug_assist
            .core_1_intr_raw
            .read()
            .core_1_area_dram0_1_rd_raw()
            .bit_is_set()
            || self
                .debug_assist
                .core_1_intr_raw
                .read()
                .core_1_area_dram0_1_wr_raw()
                .bit_is_set()
    }

    pub fn get_core1_region_monitor_pc(&self) -> u32 {
        self.debug_assist
            .core_1_area_pc
            .read()
            .core_1_area_pc()
            .bits()
    }
}
