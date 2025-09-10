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
//! [Debug Assist]: https://github.com/esp-rs/esp-hal/blob/main/examples/peripheral/debug_assist/src/main.rs
//!
//! ## Implementation State
//! - Bus write access logging is not available via this API
//! - This driver has only blocking API

use crate::{
    interrupt::InterruptHandler,
    pac,
    peripherals::{ASSIST_DEBUG, Interrupt},
};

/// The debug assist driver instance.
pub struct DebugAssist<'d> {
    debug_assist: ASSIST_DEBUG<'d>,
}

impl<'d> DebugAssist<'d> {
    /// Create a new instance in [crate::Blocking] mode.
    pub fn new(debug_assist: ASSIST_DEBUG<'d>) -> Self {
        // NOTE: We should enable the debug assist, however, it's always enabled in ROM
        //       code already.

        DebugAssist { debug_assist }
    }

    /// Register an interrupt handler for the Debug Assist module.
    ///
    /// Note that this will replace any previously registered interrupt
    /// handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::system::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::ASSIST_DEBUG);
        }
        unsafe { crate::interrupt::bind_interrupt(Interrupt::ASSIST_DEBUG, handler.handler()) };
        unwrap!(crate::interrupt::enable(
            Interrupt::ASSIST_DEBUG,
            handler.priority()
        ));
    }

    fn regs(&self) -> &pac::assist_debug::RegisterBlock {
        self.debug_assist.register_block()
    }
}

impl crate::private::Sealed for DebugAssist<'_> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for DebugAssist<'_> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

#[cfg(assist_debug_has_sp_monitor)]
impl DebugAssist<'_> {
    /// Enable SP monitoring on main core. When the SP exceeds the
    /// `lower_bound` or `upper_bound` threshold, the module will record the PC
    /// pointer and generate an interrupt.
    pub fn internal_sp_monitor(&mut self, cpu: usize, lower_bound: u32, upper_bound: u32) {
        let regs = self.regs().cpu(cpu);

        regs.sp_min()
            .write(|w| unsafe { w.sp_min().bits(lower_bound) });

        regs.sp_max()
            .write(|w| unsafe { w.sp_max().bits(upper_bound) });

        regs.montr_ena().modify(|_, w| {
            w.sp_spill_min_ena().set_bit();
            w.sp_spill_max_ena().set_bit()
        });

        regs.intr_clr().write(|w| {
            w.sp_spill_max_clr().set_bit();
            w.sp_spill_min_clr().set_bit()
        });

        regs.intr_ena().modify(|_, w| {
            w.sp_spill_max_intr_ena().set_bit();
            w.sp_spill_min_intr_ena().set_bit()
        });
    }

    fn internal_disable_sp_monitor(&mut self, cpu: usize) {
        let regs = self.regs().cpu(cpu);

        regs.intr_ena().modify(|_, w| {
            w.sp_spill_max_intr_ena().clear_bit();
            w.sp_spill_min_intr_ena().clear_bit()
        });

        regs.montr_ena().modify(|_, w| {
            w.sp_spill_min_ena().clear_bit();
            w.sp_spill_max_ena().clear_bit()
        });
    }

    fn internal_clear_sp_monitor_interrupt(&mut self, cpu: usize) {
        self.regs().cpu(cpu).intr_clr().write(|w| {
            w.sp_spill_max_clr().set_bit();
            w.sp_spill_min_clr().set_bit()
        });
    }

    fn internal_is_sp_monitor_interrupt_set(&self, cpu: usize) -> bool {
        let regs = self.regs().cpu(cpu);
        let intrs = regs.intr_raw().read();

        intrs.sp_spill_max_raw().bit_is_set() || intrs.sp_spill_min_raw().bit_is_set()
    }

    fn internal_sp_monitor_pc(&self, cpu: usize) -> u32 {
        self.regs().cpu(cpu).sp_pc().read().sp_pc().bits()
    }

    /// Enable SP monitoring on main core. When the SP exceeds the
    /// `lower_bound` or `upper_bound` threshold, the module will record the PC
    /// pointer and generate an interrupt.
    pub fn enable_sp_monitor(&mut self, lower_bound: u32, upper_bound: u32) {
        self.internal_sp_monitor(0, lower_bound, upper_bound);
    }

    /// Disable SP monitoring on main core.
    pub fn disable_sp_monitor(&mut self) {
        self.internal_disable_sp_monitor(0)
    }

    /// Clear SP monitoring interrupt on main core.
    pub fn clear_sp_monitor_interrupt(&mut self) {
        self.internal_clear_sp_monitor_interrupt(0)
    }

    /// Check, if SP monitoring interrupt is set on main core.
    pub fn is_sp_monitor_interrupt_set(&self) -> bool {
        self.internal_is_sp_monitor_interrupt_set(0)
    }

    /// Get SP monitoring PC value on main core.
    pub fn sp_monitor_pc(&self) -> u32 {
        self.internal_sp_monitor_pc(0)
    }
}

#[cfg(all(assist_debug_has_sp_monitor, multi_core))]
impl<'d> DebugAssist<'d> {
    /// Enable SP monitoring on secondary core. When the SP exceeds the
    /// `lower_bound` or `upper_bound` threshold, the module will record the PC
    /// pointer and generate an interrupt.
    pub fn enable_core1_sp_monitor(&mut self, lower_bound: u32, upper_bound: u32) {
        self.internal_sp_monitor(1, lower_bound, upper_bound);
    }

    /// Disable SP monitoring on secondary core.
    pub fn disable_core1_sp_monitor(&mut self) {
        self.internal_disable_sp_monitor(1)
    }

    /// Clear SP monitoring interrupt on secondary core.
    pub fn clear_core1_sp_monitor_interrupt(&mut self) {
        self.internal_clear_sp_monitor_interrupt(1)
    }

    /// Check, if SP monitoring interrupt is set on secondary core.
    pub fn is_core1_sp_monitor_interrupt_set(&self) -> bool {
        self.internal_is_sp_monitor_interrupt_set(1)
    }

    /// Get SP monitoring PC value on secondary core.
    pub fn core1_sp_monitor_pc(&self) -> u32 {
        self.internal_sp_monitor_pc(1)
    }
}

#[cfg(assist_debug_has_region_monitor)]
impl DebugAssist<'_> {
    fn internal_enable_region0_monitor(
        &mut self,
        cpu: usize,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        let regs = self.regs().cpu(cpu);

        regs.area_dram0_0_min()
            .write(|w| unsafe { w.area_dram0_0_min().bits(lower_bound) });

        regs.area_dram0_0_max()
            .write(|w| unsafe { w.area_dram0_0_max().bits(upper_bound) });

        regs.montr_ena().modify(|_, w| {
            w.area_dram0_0_rd_ena().bit(reads);
            w.area_dram0_0_wr_ena().bit(writes)
        });

        regs.intr_clr().write(|w| {
            w.area_dram0_0_rd_clr().set_bit();
            w.area_dram0_0_wr_clr().set_bit()
        });

        regs.intr_ena().modify(|_, w| {
            w.area_dram0_0_rd_intr_ena().set_bit();
            w.area_dram0_0_wr_intr_ena().set_bit()
        });
    }

    fn internal_disable_region0_monitor(&mut self, cpu: usize) {
        let regs = self.regs().cpu(cpu);

        regs.intr_ena().modify(|_, w| {
            w.area_dram0_0_rd_intr_ena().clear_bit();
            w.area_dram0_0_wr_intr_ena().clear_bit()
        });

        regs.montr_ena().modify(|_, w| {
            w.area_dram0_0_rd_ena().clear_bit();
            w.area_dram0_0_wr_ena().clear_bit()
        });
    }

    fn internal_clear_region0_monitor_interrupt(&mut self, cpu: usize) {
        self.regs().cpu(cpu).intr_clr().write(|w| {
            w.area_dram0_0_rd_clr().set_bit();
            w.area_dram0_0_wr_clr().set_bit()
        });
    }

    fn internal_is_region0_monitor_interrupt_set(&self, cpu: usize) -> bool {
        let regs = self.regs().cpu(cpu);
        let intrs = regs.intr_raw().read();

        intrs.area_dram0_0_rd_raw().bit_is_set() || intrs.area_dram0_0_wr_raw().bit_is_set()
    }

    fn internal_enable_region1_monitor(
        &mut self,
        cpu: usize,
        lower_bound: u32,
        upper_bound: u32,
        reads: bool,
        writes: bool,
    ) {
        let regs = self.regs().cpu(cpu);

        regs.area_dram0_1_min()
            .write(|w| unsafe { w.area_dram0_1_min().bits(lower_bound) });

        regs.area_dram0_1_max()
            .write(|w| unsafe { w.area_dram0_1_max().bits(upper_bound) });

        regs.montr_ena().modify(|_, w| {
            w.area_dram0_1_rd_ena().bit(reads);
            w.area_dram0_1_wr_ena().bit(writes)
        });

        regs.intr_clr().write(|w| {
            w.area_dram0_1_rd_clr().set_bit();
            w.area_dram0_1_wr_clr().set_bit()
        });

        regs.intr_ena().modify(|_, w| {
            w.area_dram0_1_rd_intr_ena().set_bit();
            w.area_dram0_1_wr_intr_ena().set_bit()
        });
    }

    fn internal_disable_region1_monitor(&mut self, cpu: usize) {
        let regs = self.regs().cpu(cpu);

        regs.intr_ena().modify(|_, w| {
            w.area_dram0_1_rd_intr_ena().clear_bit();
            w.area_dram0_1_wr_intr_ena().clear_bit()
        });

        regs.montr_ena().modify(|_, w| {
            w.area_dram0_1_rd_ena().clear_bit();
            w.area_dram0_1_wr_ena().clear_bit()
        });
    }

    fn internal_clear_region1_monitor_interrupt(&mut self, cpu: usize) {
        self.regs().cpu(cpu).intr_clr().write(|w| {
            w.area_dram0_1_rd_clr().set_bit();
            w.area_dram0_1_wr_clr().set_bit()
        });
    }

    fn internal_is_region1_monitor_interrupt_set(&self, cpu: usize) -> bool {
        let regs = self.regs().cpu(cpu);
        let intrs = regs.intr_raw().read();

        intrs.area_dram0_1_rd_raw().bit_is_set() || intrs.area_dram0_1_wr_raw().bit_is_set()
    }

    fn internal_region_monitor_pc(&self, cpu: usize) -> u32 {
        self.regs().cpu(cpu).area_pc().read().area_pc().bits()
    }

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
        self.internal_enable_region0_monitor(0, lower_bound, upper_bound, reads, writes)
    }

    /// Disable region0 monitoring on main core.
    pub fn disable_region0_monitor(&mut self) {
        self.internal_disable_region0_monitor(0)
    }

    /// Clear region0 monitoring interrupt on main core.
    pub fn clear_region0_monitor_interrupt(&mut self) {
        self.internal_clear_region0_monitor_interrupt(0)
    }

    /// Check, if region0 monitoring interrupt is set on main core.
    pub fn is_region0_monitor_interrupt_set(&self) -> bool {
        self.internal_is_region0_monitor_interrupt_set(0)
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
        self.internal_enable_region1_monitor(0, lower_bound, upper_bound, reads, writes)
    }

    /// Disable region1 monitoring on main core.
    pub fn disable_region1_monitor(&mut self) {
        self.internal_disable_region1_monitor(0)
    }

    /// Clear region1 monitoring interrupt on main core.
    pub fn clear_region1_monitor_interrupt(&mut self) {
        self.internal_clear_region1_monitor_interrupt(0)
    }

    /// Check, if region1 monitoring interrupt is set on main core.
    pub fn is_region1_monitor_interrupt_set(&self) -> bool {
        self.internal_is_region1_monitor_interrupt_set(0)
    }

    /// Get region monitoring PC value on main core.
    pub fn region_monitor_pc(&self) -> u32 {
        self.internal_region_monitor_pc(0)
    }
}

#[cfg(all(assist_debug_has_region_monitor, multi_core))]
impl DebugAssist<'_> {
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
        self.internal_enable_region0_monitor(1, lower_bound, upper_bound, reads, writes)
    }

    /// Disable region0 monitoring on secondary core.
    pub fn disable_core1_region0_monitor(&mut self) {
        self.internal_disable_region0_monitor(1)
    }

    /// Clear region0 monitoring interrupt on secondary core.
    pub fn clear_core1_region0_monitor_interrupt(&mut self) {
        self.internal_clear_region0_monitor_interrupt(1)
    }

    /// Check, if region0 monitoring interrupt is set on secondary core.
    pub fn is_core1_region0_monitor_interrupt_set(&self) -> bool {
        self.internal_is_region0_monitor_interrupt_set(1)
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
        self.internal_enable_region1_monitor(1, lower_bound, upper_bound, reads, writes)
    }

    /// Disable region1 monitoring on secondary core.
    pub fn disable_core1_region1_monitor(&mut self) {
        self.internal_disable_region1_monitor(1)
    }

    /// Clear region1 monitoring interrupt on secondary core.
    pub fn clear_core1_region1_monitor_interrupt(&mut self) {
        self.internal_clear_region1_monitor_interrupt(1)
    }

    /// Check, if region1 monitoring interrupt is set on secondary core.
    pub fn is_core1_region1_monitor_interrupt_set(&self) -> bool {
        self.internal_is_region1_monitor_interrupt_set(1)
    }

    /// Get region monitoring PC value on secondary core.
    pub fn core1_region_monitor_pc(&self) -> u32 {
        self.internal_region_monitor_pc(1)
    }
}
