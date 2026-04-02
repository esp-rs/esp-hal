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
#[cfg(esp32s2)]
fn current_interrupts() -> u32 {
    let mut status_bits: u32 = 0b0;
    // Add the SENS peripheral status, which is broken out into individual bit flags
    let sens_status = unsafe { &*crate::pac::SENS::PTR }.sar_cocpu_int_st().read();
    status_bits |= (sens_status.cocpu_touch_done_int_st().bit_is_set() as u32) << 0;
    status_bits |= (sens_status.cocpu_touch_inactive_int_st().bit_is_set() as u32) << 1;
    status_bits |= (sens_status.cocpu_touch_active_int_st().bit_is_set() as u32) << 2;
    status_bits |= (sens_status.cocpu_saradc1_int_st().bit_is_set() as u32) << 3;
    status_bits |= (sens_status.cocpu_saradc2_int_st().bit_is_set() as u32) << 4;
    status_bits |= (sens_status.cocpu_tsens_int_st().bit_is_set() as u32) << 5;
    status_bits |= (sens_status.cocpu_start_int_st().bit_is_set() as u32) << 6;
    status_bits |= (sens_status.cocpu_sw_int_st().bit_is_set() as u32) << 7;
    status_bits |= (sens_status.cocpu_swd_int_st().bit_is_set() as u32) << 8;
    // Add the GPIO peripheral status, which is 1 if any of the GPIO pins have an interrupt set.
    let gpio_status = unsafe { &*crate::pac::RTC_IO::PTR }.status().read().bits();
    status_bits |= ((gpio_status != 0) as u32) << 9;
    status_bits
}

/// Returns a bitmask of active interrupts
#[cfg(esp32s3)]
fn current_interrupts() -> u32 {
    let mut status_bits: u32 = 0b0;
    // Add the SENS peripheral status, which is broken out into individual bit flags
    let sens_status = unsafe { &*crate::pac::SENS::PTR }.sar_cocpu_int_st().read();
    status_bits |= (sens_status.sar_cocpu_touch_done_int_st().bit_is_set() as u32) << 0;
    status_bits |= (sens_status.sar_cocpu_touch_inactive_int_st().bit_is_set() as u32) << 1;
    status_bits |= (sens_status.sar_cocpu_touch_active_int_st().bit_is_set() as u32) << 2;
    status_bits |= (sens_status.sar_cocpu_saradc1_int_st().bit_is_set() as u32) << 3;
    status_bits |= (sens_status.sar_cocpu_saradc2_int_st().bit_is_set() as u32) << 4;
    status_bits |= (sens_status.sar_cocpu_tsens_int_st().bit_is_set() as u32) << 5;
    status_bits |= (sens_status.sar_cocpu_start_int_st().bit_is_set() as u32) << 6;
    status_bits |= (sens_status.sar_cocpu_sw_int_st().bit_is_set() as u32) << 7;
    status_bits |= (sens_status.sar_cocpu_swd_int_st().bit_is_set() as u32) << 8;
    status_bits |= (sens_status.sar_cocpu_touch_timeout_int_st().bit_is_set() as u32) << 9;
    status_bits |= (sens_status
        .sar_cocpu_touch_approach_loop_done_int_st()
        .bit_is_set() as u32)
        << 10;
    status_bits |= (sens_status.sar_cocpu_touch_scan_done_int_st().bit_is_set() as u32) << 11;
    // Add the GPIO peripheral status, which is 1 if any of the GPIO pins have an interrupt set.
    let gpio_status = unsafe { &*crate::pac::RTC_IO::PTR }.status().read().bits();
    status_bits |= ((gpio_status != 0) as u32) << 12;
    status_bits
}

/// Enables a peripheral interrupt at a given priority, using vectored CPU interrupts.
///
/// Note that interrupts still need to be enabled globally for interrupts
/// to be serviced.
///
/// Internally, this function maps the interrupt to the appropriate CPU interrupt
/// for the specified priority level.
#[inline]
pub fn enable(interrupt: Interrupt, _level: Priority) {
    #[cfg(esp32s3)]
    match interrupt {
        Interrupt::TOUCH_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_touch_done_int_ena().set_bit()),
        Interrupt::TOUCH_INACTIVE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_touch_inactive_int_ena().set_bit()),
        Interrupt::TOUCH_ACTIVE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_touch_active_int_ena().set_bit()),
        Interrupt::SARADC1_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_saradc1_int_ena().set_bit()),
        Interrupt::SARADC2_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_saradc2_int_ena().set_bit()),
        Interrupt::TSENS_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_tsens_int_ena().set_bit()),
        Interrupt::RISCV_START_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_start_int_ena().set_bit()),
        Interrupt::SW_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_sw_int_ena().set_bit()),
        Interrupt::SWD_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_swd_int_ena().set_bit()),
        Interrupt::TOUCH_TIME_OUT_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_touch_timeout_int_ena().set_bit()),
        Interrupt::TOUCH_APPROACH_LOOP_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_touch_approach_loop_done_int_ena().set_bit()),
        Interrupt::TOUCH_SCAN_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.sar_cocpu_touch_scan_done_int_ena().set_bit()),
        Interrupt::GPIO_INT => 0,
    };

    #[cfg(esp32s2)]
    match interrupt {
        Interrupt::TOUCH_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_touch_done_int_ena().set_bit()),
        Interrupt::TOUCH_INACTIVE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_touch_inactive_int_ena().set_bit()),
        Interrupt::TOUCH_ACTIVE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_touch_active_int_ena().set_bit()),
        Interrupt::SARADC1_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_saradc1_int_ena().set_bit()),
        Interrupt::SARADC2_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_saradc2_int_ena().set_bit()),
        Interrupt::TSENS_DONE_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_tsens_int_ena().set_bit()),
        Interrupt::RISCV_START_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_start_int_ena().set_bit()),
        Interrupt::SW_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_sw_int_ena().set_bit()),
        Interrupt::SWD_INT => unsafe { &*crate::pac::SENS::PTR }
            .sar_cocpu_int_ena()
            .write(|w| w.cocpu_swd_int_ena().set_bit()),
        Interrupt::GPIO_INT => 0,
    };
}
