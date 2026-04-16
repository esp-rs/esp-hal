//! Interrupt handling for ESP32-S2 & ESP32-S3 RISCV ULP cores.
//! Uses custom R-type instructions for ESP32-S2 & ESP32-S3 RISCV ULP cores.
use core::ptr::NonNull;

use super::{Interrupt, InterruptStatus, bound_handler};

/// Setup interrupt handlers, including any default ones
#[inline(always)]
pub fn setup_interrupts() {
    enable_cpu_interrupts();
}

/// Enables or disables a peripheral interrupt.
///
/// Note that interrupts still need to be enabled globally for interrupts
/// to be serviced.
///
/// Internally, this function maps the interrupt to the appropriate CPU interrupt.
#[inline]
pub fn set_enabled(interrupt: Interrupt, enable: bool) {
    // Enable/disable SENS interrupts
    unsafe { &*crate::pac::SENS::PTR }
        .sar_cocpu_int_ena()
        .write(|w| {
            #[cfg(esp32s3)]
            match interrupt {
                Interrupt::TOUCH_DONE_INT => w.sar_cocpu_touch_done_int_ena().bit(enable),
                Interrupt::TOUCH_INACTIVE_INT => w.sar_cocpu_touch_inactive_int_ena().bit(enable),
                Interrupt::TOUCH_ACTIVE_INT => w.sar_cocpu_touch_active_int_ena().bit(enable),
                Interrupt::SARADC1_DONE_INT => w.sar_cocpu_saradc1_int_ena().bit(enable),
                Interrupt::SARADC2_DONE_INT => w.sar_cocpu_saradc2_int_ena().bit(enable),
                Interrupt::TSENS_DONE_INT => w.sar_cocpu_tsens_int_ena().bit(enable),
                Interrupt::RISCV_START_INT => w.sar_cocpu_start_int_ena().bit(enable),
                Interrupt::SW_INT => w.sar_cocpu_sw_int_ena().bit(enable),
                Interrupt::SWD_INT => w.sar_cocpu_swd_int_ena().bit(enable),
                Interrupt::TOUCH_TIME_OUT_INT => w.sar_cocpu_touch_timeout_int_ena().bit(enable),
                Interrupt::TOUCH_APPROACH_LOOP_DONE_INT => {
                    w.sar_cocpu_touch_approach_loop_done_int_ena().bit(enable)
                }
                Interrupt::TOUCH_SCAN_DONE_INT => w.sar_cocpu_touch_scan_done_int_ena().bit(enable),
                // Ignore any other non-SENS interrupts
                _ => w,
            }
            #[cfg(esp32s2)]
            match interrupt {
                Interrupt::TOUCH_DONE_INT => w.cocpu_touch_done_int_ena().bit(enable),
                Interrupt::TOUCH_INACTIVE_INT => w.cocpu_touch_inactive_int_ena().bit(enable),
                Interrupt::TOUCH_ACTIVE_INT => w.cocpu_touch_active_int_ena().bit(enable),
                Interrupt::SARADC1_DONE_INT => w.cocpu_saradc1_int_ena().bit(enable),
                Interrupt::SARADC2_DONE_INT => w.cocpu_saradc2_int_ena().bit(enable),
                Interrupt::TSENS_DONE_INT => w.cocpu_tsens_int_ena().bit(enable),
                Interrupt::RISCV_START_INT => w.cocpu_start_int_ena().bit(enable),
                Interrupt::SW_INT => w.cocpu_sw_int_ena().bit(enable),
                Interrupt::SWD_INT => w.cocpu_swd_int_ena().bit(enable),
                // Ignore any other non-SENS interrupts
                _ => w,
            }
        });

    // GPIO interrupt requires no enable/disable handling here,
    // as it is handled by the gpio module on a per-pin basis.
}

/// Returns a bitmask of active interrupts
pub fn current_interrupts() -> u32 {
    let mut status_bits: u32 = 0b0;
    // The SENS peripheral status is broken out into individual bit flags
    let sens_status = unsafe { &*crate::pac::SENS::PTR }
        .sar_cocpu_int_st()
        .read()
        .bits();
    status_bits |= sens_status;

    // The GPIO peripheral status, is a single bit result, which is 1 if any GPIO interrupt status
    // bit is 1.
    let gpio_status = unsafe { &*crate::pac::RTC_IO::PTR }.status().read().bits();
    let any_gpio_interrupt = gpio_status != 0;

    #[cfg(esp32s2)]
    const SENS_BITFLAGS_LEN: usize = 9;
    #[cfg(esp32s3)]
    const SENS_BITFLAGS_LEN: usize = 12;

    if any_gpio_interrupt {
        status_bits |= 1 << SENS_BITFLAGS_LEN;
    }

    status_bits
}

/// Set the IRQ Mask, returns the previous mask value.
/// IRQ Type   Bit   Description
/// Internal     0   Internal timer interrupt
/// Internal     1   EBREAK/ECALL or Illegal Instruction
/// Internal     2   BUS Error (Unaligned Memory Access)
/// External    31   RTC peripheral interrupts
#[inline(always)]
pub fn mask_cpu_interrupts(new_mask: u32) -> u32 {
    let old_mask: u32;
    unsafe {
        core::arch::asm!(
            "maskirq_insn {}, {}",
            out(reg) old_mask,
            in(reg) new_mask
        );
    }
    old_mask
}

/// Enable all CPU interrupts by setting IRQ mask
#[inline(always)]
pub fn enable_cpu_interrupts() {
    mask_cpu_interrupts(0x0);
}

/// Disable all CPU interrupts by clearing IRQ mask,
/// returns the previous IRQ Mask
#[inline(always)]
pub fn disable_cpu_interrupts() -> u32 {
    let mask = (1 << 31) | (1 << 2) | (1 << 1) | (1 << 0);
    mask_cpu_interrupts(mask)
}

/// Wait for any (masked or unmasked) CPU interrupt
pub fn wait_cpu_interrupt() -> u32 {
    let result: u32;
    unsafe {
        core::arch::asm!(
            "waitirq_insn {}",
            out(reg) result,
        )
    }
    result
}

/// Trap entry point rust (_start_trap_rust)
/// `irqs` is a bitmask of IRQs to handle.
#[doc(hidden)]
#[unsafe(link_section = ".trap.rust")]
#[unsafe(export_name = "_start_trap_rust")]
pub extern "C" fn ulp_start_trap_rust(trap_frame: *const u32, irqs: u32) {
    unsafe extern "C" {
        fn trap_handler(regs: &TrapFrame, pending_irqs: u32);
    }

    unsafe {
        // 'trap_frame' pointer safety:
        // _start_trap must place a valid address in a0, prior to calling _start_trap_rust.
        trap_handler(
            NonNull::new_unchecked(trap_frame as *mut TrapFrame).as_ref(),
            irqs,
        );
    }
}

/// Called by _start_trap_rust, this trap handler will call other interrupt handling
/// functions depending on the bits set in pending_irqs.
#[doc(hidden)]
#[unsafe(no_mangle)]
pub extern "C" fn trap_handler(_regs: &TrapFrame, pending_irqs: u32) {
    // Dispatch peripheral interrupt
    if pending_irqs & (1 << 31) != 0 {
        let status = InterruptStatus::current();

        // Iterate the active interrupts, fetch their handler, and call it if set.
        for interrupt_nr in status.iterator() {
            if let Ok(i) = Interrupt::try_from(interrupt_nr) {
                if let Some(handler) = bound_handler(i) {
                    handler.callback()();
                }
            }
        }
    }
}

/// Default interrupt handler, does nothing.
#[allow(dead_code)]
#[doc(hidden)]
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
pub fn DefaultHandler() {}

/// TODO: Write store_trap / load_trap functions, to generate the context saving assembly code,
//        which is currently hand-written in ulp_riscv_vectors.S
/// Registers saved in trap handler
#[doc(hidden)]
#[repr(C)]
#[derive(Debug)]
pub struct TrapFrame {
    /// `x1`: return address, stores the address to return to after a function call or interrupt.
    pub ra: usize,
    /// `x5`: temporary register `t0`, used for intermediate values.
    pub t0: usize,
    /// `x6`: temporary register `t1`, used for intermediate values.
    pub t1: usize,
    /// `x7`: temporary register `t2`, used for intermediate values.
    pub t2: usize,
    /// `x28`: temporary register `t3`, used for intermediate values.
    pub t3: usize,
    /// `x29`: temporary register `t4`, used for intermediate values.
    pub t4: usize,
    /// `x30`: temporary register `t5`, used for intermediate values.
    pub t5: usize,
    /// `x31`: temporary register `t6`, used for intermediate values.
    pub t6: usize,
    /// `x10`: argument register `a0`. Used to pass the first argument to a function.
    pub a0: usize,
    /// `x11`: argument register `a1`. Used to pass the second argument to a function.
    pub a1: usize,
    /// `x12`: argument register `a2`. Used to pass the third argument to a function.
    pub a2: usize,
    /// `x13`: argument register `a3`. Used to pass the fourth argument to a function.
    pub a3: usize,
    /// `x14`: argument register `a4`. Used to pass the fifth argument to a function.
    pub a4: usize,
    /// `x15`: argument register `a5`. Used to pass the sixth argument to a function.
    pub a5: usize,
    /// `x16`: argument register `a6`. Used to pass the seventh argument to a function.
    pub a6: usize,
    /// `x17`: argument register `a7`. Used to pass the eighth argument to a function.
    pub a7: usize,
}
