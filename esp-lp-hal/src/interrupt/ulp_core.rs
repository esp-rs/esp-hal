//! Interrupt handling for ESP32-S2 & ESP32-S3 RISCV ULP cores.
//! Uses custom R-type instructions for ESP32-S2 & ESP32-S3 RISCV ULP cores.
use super::Interrupt;

/// Setup interrupt handlers, including any default ones
pub fn setup_interrupts() {
    machine_interrupt_enable(true);
}

/// Enables or disables a peripheral interrupt.
///
/// Note that interrupts still need to be enabled globally for interrupts
/// to be serviced.
///
/// Internally, this function maps the interrupt to the appropriate CPU interrupt.
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

/// Clears a peripheral interrupt.
/// For GPIO_INT, clears all interrupts.
pub fn clear(interrupt: Interrupt) {
    // Enable/disable SENS interrupts
    match interrupt {
        Interrupt::GPIO_INT => {
            crate::gpio::interrupt::gpio_interrupt_clear(
                crate::gpio::interrupt::gpio_interrupt_status(),
            );
        }
        _ => {
            unsafe { &*crate::pac::SENS::PTR }
                .sar_cocpu_int_clr()
                .write(|w| unsafe { w.bits(interrupt as u32) });
        }
    }
}

/// Returns a bitmask of active interrupts
pub fn status() -> u32 {
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

/// Set the mie register (or equivalent),
/// returning the previous value.
#[inline(always)]
pub fn machine_interrupt_enable(enable: bool) -> bool {
    // Does not affect the internal exception bits,
    //  which are always enabled (unmasked, value here is 0).
    // IRQ Type   Bit   Description
    // Internal     0   Internal timer interrupt
    // Internal     1   EBREAK/ECALL or Illegal Instruction
    // Internal     2   BUS Error (Unaligned Memory Access)
    // External    31   RTC peripheral interrupts
    let old_mask: u32;

    let disable_exceptions: u32 = 0b111;
    let disable_bit: u32 = 1 << 31;
    let new_mask: u32 = if enable {
        0b0
    } else {
        disable_bit | disable_exceptions
    };

    unsafe {
        core::arch::asm!(
            "maskirq_insn {}, {}",
            out(reg) old_mask,
            in(reg) new_mask
        );
    }

    // Return previous enabled value,
    // where enable == 0 (unmasked)
    (old_mask & disable_bit) == 0
}

/// Returns the cause of a machine interrupt,
/// which contains the exception code, and if a peripheral interrupt was flagged.
#[unsafe(link_section = ".trap.rust")]
#[inline(always)]
pub fn trap_cause() -> riscv::interrupt::Trap<usize, usize> {
    // Exception ID, Description
    // 2, Illegal instructions
    // 3, Breakpoints (EBREAK)
    // 6, Misaligned atomic instructions
    //
    // Bit 31 is used to indicate an interrupt.

    // mcause register does not exist on ULP cores,
    // so the Trap must be formed using q-registers
    let cause: u32;
    unsafe {
        core::arch::asm!(
            "getq_insn {}, q1",
            out(reg) cause
        );
    }

    let interrupt: bool = (cause & (0b1 << 31)) != 0;

    if interrupt {
        let code = (cause & 0b111) as usize;
        riscv::interrupt::Trap::Interrupt(code)
    } else {
        let code = (cause & 0b1111) as usize;
        riscv::interrupt::Trap::Exception(code)
    }
}
