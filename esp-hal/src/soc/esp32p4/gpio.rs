//! ESP32-P4 LP GPIO (RtcPin) implementation.
//!
//! P4 LP GPIO uses named registers (pin0, pin1, ...) instead of indexed pin(n).
//! LP_IO_MUX uses pad0, pad1, ... with different field names from other chips.
//!
//! LP pins: GPIO0-5, GPIO12-23 (16 pins total, mapped to LP_GPIO pin0-pin15)
//! GPIO0-5 -> LP pin 0-5
//! GPIO12-23 -> LP pin 6-17 (offset by 6)
//!
//! Ref: TRM v0.5 Ch 11 (GPIO) -- LP_IO_MUX section
//!      P4 PAC: lp_gpio (0x5012_A000), lp_io_mux (0x5012_B000)

/// LP GPIO base address (from PAC)
const LP_GPIO_BASE: u32 = 0x5012_A000;
/// LP IO MUX base address (from PAC)
const LP_IO_MUX_BASE: u32 = 0x5012_B000;

/// Convert GPIO number to LP pin index.
/// GPIO0-5 -> LP pin 0-5
/// GPIO12-23 -> LP pin 6-17
#[allow(dead_code)] // used by p4_rtc_pin! macro expansions and future LP wakeup code
const fn gpio_to_lp_pin(gpio: u8) -> Option<u8> {
    match gpio {
        0..=5 => Some(gpio),
        12..=23 => Some(gpio - 6),
        _ => None,
    }
}

/// Implement RtcPin for a P4 LP-capable GPIO.
/// Uses direct MMIO because P4 LP_GPIO PAC has named registers (pin0, pin1, ...)
/// and LP_IO_MUX has different field names from other chips.
macro_rules! p4_rtc_pin {
    ($($gpio_num:literal => $lp_pin:literal),+ $(,)?) => {
        $(
            impl $crate::gpio::RtcPin for paste::paste!($crate::peripherals::[<GPIO $gpio_num>]<'_>) {
                unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
                    // LP_GPIO.PINn register: wakeup_enable + int_type
                    // Ref: P4 PAC lp_gpio/pin0.rs -- reg_gpio_pin0_wakeup_enable, int_type
                    // Each PINn register is at LP_GPIO_BASE + 0x20 + n*4
                    unsafe {
                        let pin_reg = (LP_GPIO_BASE + 0x20 + $lp_pin * 4) as *mut u32;
                        let val = pin_reg.read_volatile();
                        let val = (val & !(0x7 << 7)) | ((level as u32 & 0x7) << 7);
                        let val = if wakeup { val | (1 << 10) } else { val & !(1 << 10) };
                        pin_reg.write_volatile(val);
                    }
                }

                fn rtcio_pad_hold(&self, enable: bool) {
                    // LP_AON (LP_SYS) doesn't have gpio_hold for P4.
                    // P4 uses LP_IO_MUX pad hold instead.
                    // TODO: implement LP pad hold
                    // For now, no-op.
                    let _ = enable;
                }

                fn rtc_set_config(&self, input_enable: bool, mux: bool, func: $crate::gpio::RtcFunction) {
                    unsafe {
                        // LP_IO_MUX.PADn register
                        // pad0 at offset 0x08, pad1 at 0x0C, etc.
                        let pad_reg = (LP_IO_MUX_BASE + 0x08 + $lp_pin * 4) as *mut u32;
                        let mut val = pad_reg.read_volatile();

                        // reg_padN_mux_sel: bit 3 (1=LP mode, 0=IO MUX mode)
                        if mux { val |= 1 << 3; } else { val &= !(1 << 3); }

                        // reg_padN_fun_ie: bit 8 (input enable)
                        if input_enable { val |= 1 << 8; } else { val &= !(1 << 8); }

                        // reg_padN_fun_sel: bits [5:4] (function select)
                        val = (val & !(0x3 << 4)) | (((func as u32) & 0x3) << 4);

                        // reg_padN_slp_sel: bit 6 (0=normal operation)
                        val &= !(1 << 6);

                        pad_reg.write_volatile(val);
                    }
                }
            }

            impl $crate::gpio::RtcPinWithResistors for paste::paste!($crate::peripherals::[<GPIO $gpio_num>]<'_>) {
                fn rtcio_pullup(&self, enable: bool) {
                    unsafe {
                        let pad_reg = (LP_IO_MUX_BASE + 0x08 + $lp_pin * 4) as *mut u32;
                        let val = pad_reg.read_volatile();
                        // reg_padN_rue: bit 2 (pullup enable)
                        if enable {
                            pad_reg.write_volatile(val | (1 << 2));
                        } else {
                            pad_reg.write_volatile(val & !(1 << 2));
                        }
                    }
                }

                fn rtcio_pulldown(&self, enable: bool) {
                    unsafe {
                        let pad_reg = (LP_IO_MUX_BASE + 0x08 + $lp_pin * 4) as *mut u32;
                        let val = pad_reg.read_volatile();
                        // reg_padN_rde: bit 1 (pulldown enable)
                        if enable {
                            pad_reg.write_volatile(val | (1 << 1));
                        } else {
                            pad_reg.write_volatile(val & !(1 << 1));
                        }
                    }
                }
            }
        )+
    };
}

// GPIO0-5 -> LP pin 0-5
p4_rtc_pin! {
    0 => 0,
    1 => 1,
    2 => 2,
    3 => 3,
    4 => 4,
    5 => 5,
}

// GPIO12-23 -> LP pin 6-17
p4_rtc_pin! {
    12 => 6,
    13 => 7,
    14 => 8,
    15 => 9,
    16 => 10,
    17 => 11,
    18 => 12,
    19 => 13,
    20 => 14,
    21 => 15,
    22 => 16,
    23 => 17,
}
