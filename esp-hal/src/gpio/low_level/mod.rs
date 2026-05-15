#[cfg_attr(gpio_version = "1", path = "v1.rs")]
#[cfg_attr(gpio_version = "2", path = "v2.rs")]
#[cfg_attr(gpio_version = "3", path = "v3.rs")]
mod version;

use portable_atomic::AtomicU32;
use strum::EnumCount;

use super::AnyPin;
use crate::peripherals::GPIO;

#[doc(hidden)]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash, EnumCount)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GpioBank {
    _0,
    #[cfg(gpio_has_bank_1)]
    _1,
}

impl GpioBank {
    pub(crate) fn async_operations(self) -> &'static AtomicU32 {
        static FLAGS: [AtomicU32; GpioBank::COUNT] = [const { AtomicU32::new(0) }; GpioBank::COUNT];

        &FLAGS[self as usize]
    }

    pub(crate) fn offset(self) -> u8 {
        match self {
            Self::_0 => 0,
            #[cfg(gpio_has_bank_1)]
            Self::_1 => 32,
        }
    }

    pub(crate) fn write_out_en(self, word: u32, enable: bool) {
        if enable {
            self.write_out_en_set(word);
        } else {
            self.write_out_en_clear(word);
        }
    }

    pub(crate) fn write_out_en_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs()
                .enable_w1tc()
                .write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs()
                .enable1_w1tc()
                .write(|w| unsafe { w.bits(word) }),
        };
    }

    pub(crate) fn write_out_en_set(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs()
                .enable_w1ts()
                .write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs()
                .enable1_w1ts()
                .write(|w| unsafe { w.bits(word) }),
        };
    }

    pub(crate) fn read_input(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().in_().read().bits(),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().in1().read().bits(),
        }
    }

    pub(crate) fn read_output(self) -> u32 {
        match self {
            Self::_0 => GPIO::regs().out().read().bits(),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().out1().read().bits(),
        }
    }

    pub(crate) fn read_interrupt_status(self) -> u32 {
        version::read_bank_interrupt_status(self)
    }

    pub(crate) fn write_interrupt_status_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs()
                .status_w1tc()
                .write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs()
                .status1_w1tc()
                .write(|w| unsafe { w.bits(word) }),
        };
    }

    pub(crate) fn write_output(self, word: u32, set: bool) {
        if set {
            self.write_output_set(word);
        } else {
            self.write_output_clear(word);
        }
    }

    pub(crate) fn write_output_set(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs().out_w1ts().write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().out1_w1ts().write(|w| unsafe { w.bits(word) }),
        };
    }

    pub(crate) fn write_output_clear(self, word: u32) {
        match self {
            Self::_0 => GPIO::regs().out_w1tc().write(|w| unsafe { w.bits(word) }),
            #[cfg(gpio_has_bank_1)]
            Self::_1 => GPIO::regs().out1_w1tc().write(|w| unsafe { w.bits(word) }),
        };
    }
}

#[derive(Clone, Copy)]
pub(crate) enum InterruptStatusRegisterAccess {
    Bank0,
    #[cfg(gpio_has_bank_1)]
    Bank1,
}

impl InterruptStatusRegisterAccess {
    pub(crate) fn interrupt_status_read(self) -> u32 {
        version::read_interrupt_status(self)
    }
}

pub(crate) fn bank(_gpio_num: u8) -> GpioBank {
    #[cfg(gpio_has_bank_1)]
    if _gpio_num >= 32 {
        return GpioBank::_1;
    }

    GpioBank::_0
}

pub(crate) fn prepare_pin_pull(pin: &AnyPin<'_>, pull_up: bool, pull_down: bool) {
    version::prepare_pin_pull(pin, pull_up, pull_down);
}

pub(crate) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    version::gpio_intr_enable(int_enable, nmi_enable)
}

pub(crate) fn for_each_interrupt_core(f: impl FnMut(crate::system::Cpu)) {
    version::for_each_interrupt_core(f);
}

#[cfg(feature = "rt")]
pub(crate) fn enable_additional_default_interrupts(
    interrupt: crate::peripherals::Interrupt,
    priority: crate::interrupt::Priority,
) {
    version::enable_additional_default_interrupts(interrupt, priority);
}

/// Set GPIO event listening.
///
/// - `gpio_num`: the pin to configure
/// - `int_ena`: maskable and non-maskable CPU interrupt bits. None to leave unchanged.
/// - `int_type`: interrupt type, see [super::Event] (or 0 to disable)
/// - `wake_up_from_light_sleep`: whether to wake up from light sleep
pub(crate) fn set_int_enable(
    gpio_num: u8,
    int_ena: Option<u8>,
    int_type: u8,
    wake_up_from_light_sleep: bool,
) {
    GPIO::regs().pin(gpio_num as usize).modify(|_, w| unsafe {
        if let Some(int_ena) = int_ena {
            w.int_ena().bits(int_ena);
        }
        w.int_type().bits(int_type);
        w.wakeup_enable().bit(wake_up_from_light_sleep)
    });
}

pub(crate) fn is_int_enabled(gpio_num: u8) -> bool {
    GPIO::regs().pin(gpio_num as usize).read().int_ena().bits() != 0
}
