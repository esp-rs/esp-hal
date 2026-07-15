use crate::gpio::{RtcPin, RtcPinWithResistors};

const LP_GPIO_BASE: u32 = 0x5012_A000;
const LP_IO_MUX_BASE: u32 = 0x5012_B000;

macro_rules! lp_pins {
    ($($gpio:literal => $lp_pin:literal),+ $(,)?) => {
        $(
            paste::paste! {
                impl RtcPin for crate::peripherals::[<GPIO $gpio>]<'_> {
                    unsafe fn apply_wakeup(&self, wakeup: bool, level: u8) {
                        unsafe {
                            let pin_reg =
                                (LP_GPIO_BASE + 0x34 + $lp_pin * 4) as *mut u32;
                            let value = pin_reg.read_volatile();
                            let value =
                                (value & !(0x7 << 7)) | ((level as u32 & 0x7) << 7);
                            let value = if wakeup {
                                value | (1 << 10)
                            } else {
                                value & !(1 << 10)
                            };
                            pin_reg.write_volatile(value);
                        }
                    }

                    fn rtcio_pad_hold(&self, _enable: bool) {
                        // TODO: P4 uses LP_IO_MUX pad hold; implement it once
                        // the corresponding PAC fields are available.
                    }

                    fn rtc_set_config(
                        &self,
                        input_enable: bool,
                        mux: bool,
                        func: crate::gpio::RtcFunction,
                    ) {
                        unsafe {
                            let pad_reg =
                                (LP_IO_MUX_BASE + 0x08 + $lp_pin * 4) as *mut u32;
                            let mut value = pad_reg.read_volatile();
                            if mux {
                                value |= 1 << 3;
                            } else {
                                value &= !(1 << 3);
                            }
                            if input_enable {
                                value |= 1 << 8;
                            } else {
                                value &= !(1 << 8);
                            }
                            value =
                                (value & !(0x3 << 4)) | (((func as u32) & 0x3) << 4);
                            value &= !(1 << 6);
                            pad_reg.write_volatile(value);
                        }
                    }
                }

                impl RtcPinWithResistors for crate::peripherals::[<GPIO $gpio>]<'_> {
                    fn rtcio_pullup(&self, enable: bool) {
                        set_pull($lp_pin, 2, enable);
                    }

                    fn rtcio_pulldown(&self, enable: bool) {
                        set_pull($lp_pin, 1, enable);
                    }
                }
            }
        )+
    };
}

lp_pins! {
    0 => 0,
    1 => 1,
    2 => 2,
    3 => 3,
    4 => 4,
    5 => 5,
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

fn set_pull(lp_pin: u32, bit: u32, enable: bool) {
    unsafe {
        let pad_reg = (LP_IO_MUX_BASE + 0x08 + lp_pin * 4) as *mut u32;
        let value = pad_reg.read_volatile();
        pad_reg.write_volatile(if enable {
            value | (1 << bit)
        } else {
            value & !(1 << bit)
        });
    }
}

pub(super) fn init_pin(pin: &impl RtcPin, input_enable: bool) -> u8 {
    let lp_pin = gpio_to_lp_pin(pin.number());
    pin.rtc_set_config(input_enable, true, crate::gpio::RtcFunction::Rtc);
    lp_pin
}

pub(super) fn output_enable(pin: u8, enable: bool) {
    unsafe {
        let register = (LP_GPIO_BASE + if enable { 0x18 } else { 0x1c }) as *mut u32;
        register.write_volatile(1 << pin);
    }
}

pub(super) fn input_enable(pin: u8, enable: bool) {
    set_pad_bit(pin, 8, enable);
}

pub(super) fn pullup_enable(pin: u8, enable: bool) {
    set_pull(pin as u32, 2, enable);
}

pub(super) fn pulldown_enable(pin: u8, enable: bool) {
    set_pull(pin as u32, 1, enable);
}

pub(super) fn set_open_drain_output(pin: u8, enable: bool) {
    crate::peripherals::GPIO::regs()
        .pin(pin as usize)
        .modify(|_, w| w.pad_driver().bit(enable));
}

fn set_pad_bit(lp_pin: u8, bit: u32, enable: bool) {
    unsafe {
        let pad_reg = (LP_IO_MUX_BASE + 0x08 + lp_pin as u32 * 4) as *mut u32;
        let value = pad_reg.read_volatile();
        pad_reg.write_volatile(if enable {
            value | (1 << bit)
        } else {
            value & !(1 << bit)
        });
    }
}

fn gpio_to_lp_pin(gpio: u8) -> u8 {
    match gpio {
        0..=5 => gpio,
        12..=23 => gpio - 6,
        _ => unreachable!(),
    }
}
