/// Portable interrupt binding and handling code
pub mod generic;
pub use generic::*;

// RISCV ULP specific interrupt handlers
pub mod riscv_ulp;
pub use riscv_ulp::*;

/// Setup interrupt handlers, including any default ones
#[inline(always)]
pub fn setup_interrupts() {
    crate::gpio::bind_default_interrupt_handler();
    enable_cpu_interrupts();
}

/// Returns a bitmask of active interrupts
fn current_interrupts() -> u32 {
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
