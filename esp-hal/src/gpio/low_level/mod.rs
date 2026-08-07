#[cfg_attr(gpio_version = "1", path = "v1.rs")]
#[cfg_attr(gpio_version = "2", path = "v2.rs")]
#[cfg_attr(gpio_version = "3", path = "v3.rs")]
mod version;

use portable_atomic::{AtomicU32, Ordering};
use strum::EnumCount;
pub(crate) use version::{
    enable_interrupt,
    gpio_intr_enable,
    prepare_pin_pull,
    set_interrupt_priority,
};

use crate::peripherals::GPIO;

#[doc(hidden)]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash, EnumCount)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GpioBank {
    _0,
    #[cfg(gpio_has_bank_1)]
    _1,
}

/// A set of pins, in the register words that the GPIO peripheral groups them into.
///
/// The peripheral has one register word for each bank, so a set of pins also has one word for each
/// bank. [`Self::word`] returns the word of a bank, which the caller can compare with the mask of
/// the register of that bank.
pub(crate) struct PadMask([AtomicU32; GpioBank::COUNT]);

impl PadMask {
    pub(crate) const fn new() -> Self {
        Self([const { AtomicU32::new(0) }; GpioBank::COUNT])
    }

    pub(crate) fn word(&self, bank: GpioBank) -> &AtomicU32 {
        &self.0[bank as usize]
    }

    #[cfg(sleep_driver_supported)]
    pub(crate) fn set(&self, gpio: u8, member: bool) {
        let bank = bank(gpio);
        let pin = 1 << (gpio - bank.offset());

        if member {
            self.word(bank).fetch_or(pin, Ordering::Relaxed);
        } else {
            self.word(bank).fetch_and(!pin, Ordering::Relaxed);
        }
    }

    #[cfg(sleep_driver_supported)]
    pub(crate) fn contains(&self, gpio: u8) -> bool {
        let bank = bank(gpio);

        self.word(bank).load(Ordering::Relaxed) & (1 << (gpio - bank.offset())) != 0
    }
}

impl GpioBank {
    /// All the banks, so that a caller can read the pins one word at a time.
    #[cfg(sleep_driver_supported)]
    pub(crate) const ALL: [Self; Self::COUNT] = cfg_select! {
        gpio_has_bank_1 => [Self::_0, Self::_1],
        _ => [Self::_0],
    };

    /// The pins of this bank that an async wait owns.
    ///
    /// The interrupt handler clears the bit of a pin to report that its wait is complete. The bit
    /// therefore tells the future that the wait is complete.
    pub(crate) fn async_operations(self) -> &'static AtomicU32 {
        static FLAGS: PadMask = PadMask::new();

        FLAGS.word(self)
    }

    /// The pins of this bank that listen for an interrupt.
    ///
    /// The same bits also give the pins that can end a light sleep, because one write sets the
    /// interrupt enable and the wakeup enable of a pad. Sleep entry reads these bits, and thus
    /// needs no register read for each pin.
    pub(crate) fn listening(self) -> &'static AtomicU32 {
        static FLAGS: PadMask = PadMask::new();

        FLAGS.word(self)
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

    pub(crate) fn read_interrupt_status_of_current_cpu(self) -> u32 {
        version::read_interrupt_status_of_current_cpu(self)
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

pub(crate) fn bank(_gpio_num: u8) -> GpioBank {
    #[cfg(gpio_has_bank_1)]
    if _gpio_num >= 32 {
        return GpioBank::_1;
    }

    GpioBank::_0
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
    let mut listening = false;
    GPIO::regs().pin(gpio_num as usize).modify(|r, w| unsafe {
        let enabled = int_ena.unwrap_or_else(|| r.int_ena().bits());
        listening = enabled != 0;

        w.int_ena().bits(enabled);
        w.int_type().bits(int_type);
        w.wakeup_enable().bit(wake_up_from_light_sleep)
    });

    let bank = bank(gpio_num);
    let pin = 1 << (gpio_num - bank.offset());
    if listening {
        bank.listening().fetch_or(pin, Ordering::Relaxed);
    } else {
        bank.listening().fetch_and(!pin, Ordering::Relaxed);
    }
}

pub(crate) fn is_int_enabled(gpio_num: u8) -> bool {
    GPIO::regs().pin(gpio_num as usize).read().int_ena().bits() != 0
}
