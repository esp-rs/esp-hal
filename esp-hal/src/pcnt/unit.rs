//! # PCNT - Unit Module
//!
//! ## Overview
//! The `unit` module is responsible for configuring and handling individual
//! units of the `PCNT` peripheral. Each unit represents a separate instance of
//! the `PCNT` module, identified by unit numbers like `Unit0`, `Unit1`, and so
//! on. Units can be configured with settings such as low and high limits,
//! thresholds, and optional filtering. The unit module also supports pausing,
//! resuming, and clearing the counter, as well as enabling or disabling
//! interrupts for specific events associated with the unit.

use esp_sync::RawMutex;

use crate::{
    pcnt::{AnyPcnt, channel::Channel},
    system::PeripheralGuard,
};

/// Invalid filter threshold value
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidFilterThreshold;

/// Invalid low limit - must be < 0.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidLowLimit;

/// Invalid high limit - must be > 0.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidHighLimit;

/// the current status of the counter.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ZeroMode {
    /// pulse counter decreases from positive to 0.
    #[default]
    PosZero  = 0,
    /// pulse counter increases from negative to 0
    NegZero  = 1,
    /// pulse counter is negative (not implemented?)
    Negative = 2,
    /// pulse counter is positive (not implemented?)
    Positive = 3,
}

impl From<u8> for ZeroMode {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::PosZero,
            1 => Self::NegZero,
            2 => Self::Negative,
            3 => Self::Positive,
            _ => unreachable!(), // TODO: is this good enough?  should we use some default?
        }
    }
}

/// Events that can occur in a pulse counter unit.
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Events {
    /// Sets when the pulse counter reaches the low limit.
    pub low_limit: bool,
    /// Sets when the pulse counter reaches the high limit.
    pub high_limit: bool,
    /// Sets when the pulse counter crosses threshold 0.
    pub threshold0: bool,
    /// Sets when the pulse counter crosses threshold 1.
    pub threshold1: bool,
    /// Sets when the pulse counter reaches zero.
    pub zero: bool,
}

/// Represents a pulse counter unit.
#[non_exhaustive]
pub struct Unit<'d, const NUM: usize> {
    pcnt: AnyPcnt<'d>,
    /// The counter for PCNT unit.
    pub counter: Counter<'d, NUM>,
    /// The first channel in PCNT unit.
    pub channel0: Channel<'d, NUM, 0>,
    /// The second channel in PCNT unit.
    pub channel1: Channel<'d, NUM, 1>,
}

static MUTEX: RawMutex = RawMutex::new();

impl<'d, const NUM: usize> Unit<'d, NUM> {
    /// return a new Unit
    pub(super) fn new(pcnt: AnyPcnt<'d>) -> Self {
        Self {
            counter: Counter::new(unsafe { pcnt.clone_unchecked() }),
            channel0: Channel::new(unsafe { pcnt.clone_unchecked() }),
            channel1: Channel::new(unsafe { pcnt.clone_unchecked() }),
            pcnt,
        }
    }

    /// Configures a lower limit to the count value.
    ///
    /// When the count drops to this value:
    /// - A low limit interrupt is triggered.
    /// - The count is reset to 0.
    ///
    /// If None is specified, then no interrupt is triggered and
    /// the count wraps around after [i16::MIN].
    ///
    /// The specified value must be negative.
    pub fn set_low_limit(&self, value: Option<i16>) -> Result<(), InvalidLowLimit> {
        let pcnt = self.pcnt.register_block();
        let unit = pcnt.unit(NUM);

        if let Some(value) = value {
            // low limit must be >= or the limit is -32768 and when that's
            // hit the event status claims it was the high limit.
            // tested on an esp32s3
            if !value.is_negative() {
                return Err(InvalidLowLimit);
            } else {
                unit.conf2()
                    .modify(|_, w| unsafe { w.cnt_l_lim().bits(value as u16) });
                unit.conf0().modify(|_, w| w.thr_l_lim_en().set_bit());
            }
        } else {
            unit.conf0().modify(|_, w| w.thr_l_lim_en().clear_bit());
        }
        Ok(())
    }

    /// Configures a high limit to the count value.
    ///
    /// When the count rises to this value:
    /// - A high limit interrupt is triggered.
    /// - The count is reset to 0.
    ///
    /// If None is specified, then no interrupt is triggered and
    /// the count wraps around after [i16::MAX].
    ///
    /// The specified value must be positive.
    pub fn set_high_limit(&self, value: Option<i16>) -> Result<(), InvalidHighLimit> {
        let pcnt = self.pcnt.register_block();
        let unit = pcnt.unit(NUM);

        if let Some(value) = value {
            if !value.is_positive() {
                return Err(InvalidHighLimit);
            } else {
                unit.conf2()
                    .modify(|_, w| unsafe { w.cnt_h_lim().bits(value as u16) });
                unit.conf0().modify(|_, w| w.thr_h_lim_en().set_bit());
            }
        } else {
            unit.conf0().modify(|_, w| w.thr_h_lim_en().clear_bit());
        }
        Ok(())
    }

    /// Configures a threshold value to trigger an interrupt.
    ///
    /// When the count equals this value a threshold0 interrupt is triggered.
    /// If None is specified, then no interrupt is triggered.
    pub fn set_threshold0(&self, value: Option<i16>) {
        let pcnt = self.pcnt.register_block();
        let unit = pcnt.unit(NUM);

        if let Some(value) = value {
            unit.conf1()
                .modify(|_, w| unsafe { w.cnt_thres0().bits(value as u16) });
            unit.conf0().modify(|_, w| w.thr_thres0_en().set_bit());
        } else {
            unit.conf0().modify(|_, w| w.thr_thres0_en().clear_bit());
        }
    }

    /// Configures a threshold value to trigger an interrupt.
    ///
    /// When the count equals this value a threshold1 interrupt is triggered.
    /// If None is specified, then no interrupt is triggered.
    pub fn set_threshold1(&self, value: Option<i16>) {
        let pcnt = self.pcnt.register_block();
        let unit = pcnt.unit(NUM);

        if let Some(value) = value {
            unit.conf1()
                .modify(|_, w| unsafe { w.cnt_thres1().bits(value as u16) });
            unit.conf0().modify(|_, w| w.thr_thres1_en().set_bit());
        } else {
            unit.conf0().modify(|_, w| w.thr_thres1_en().clear_bit());
        }
    }

    /// Configures the glitch filter hardware of the unit.
    ///
    /// `threshold` is the minimum number of APB_CLK cycles for a pulse to be
    /// considered valid. If it is None, the filter is disabled.
    ///
    /// The maximum possible threshold is 1023.
    pub fn set_filter(&self, threshold: Option<u16>) -> Result<(), InvalidFilterThreshold> {
        let pcnt = self.pcnt.register_block();
        let unit = pcnt.unit(NUM);

        match threshold {
            None => {
                unit.conf0().modify(|_, w| w.filter_en().clear_bit());
            }
            Some(threshold) => {
                if threshold > 1023 {
                    return Err(InvalidFilterThreshold);
                }
                unit.conf0().modify(|_, w| unsafe {
                    w.filter_thres().bits(threshold);
                    w.filter_en().set_bit()
                });
            }
        }
        Ok(())
    }

    /// Resets the counter value to zero.
    pub fn clear(&self) {
        MUTEX.lock(|| {
            let regs = self.pcnt.register_block();
            let bits = regs.ctrl().read().bits();
            regs.ctrl().write(|w| {
                unsafe { w.bits(bits) };
                w.cnt_rst_u(NUM as u8).set_bit()
            });
            regs.ctrl().write(|w| {
                unsafe { w.bits(bits) };
                w.cnt_rst_u(NUM as u8).clear_bit()
            });
        });
    }

    /// Pauses the counter.
    pub fn pause(&self) {
        MUTEX.lock(|| {
            self.pcnt
                .register_block()
                .ctrl()
                .modify(|_, w| w.cnt_pause_u(NUM as u8).set_bit());
        });
    }

    /// Resumes the counter.
    pub fn resume(&self) {
        MUTEX.lock(|| {
            self.pcnt
                .register_block()
                .ctrl()
                .modify(|_, w| w.cnt_pause_u(NUM as u8).clear_bit());
        });
    }

    /// Returns the latest events for this unit.
    pub fn events(&self) -> Events {
        let status = self.pcnt.register_block().u_status(NUM).read();

        Events {
            low_limit: status.l_lim().bit(),
            high_limit: status.h_lim().bit(),
            threshold0: status.thres0().bit(),
            threshold1: status.thres1().bit(),
            zero: status.zero().bit(),
        }
    }

    /// Returns the mode of the last zero crossing.
    pub fn zero_mode(&self) -> ZeroMode {
        self.pcnt
            .register_block()
            .u_status(NUM)
            .read()
            .zero_mode()
            .bits()
            .into()
    }

    /// Enables interrupts for this unit.
    pub fn listen(&self) {
        MUTEX.lock(|| {
            self.pcnt
                .register_block()
                .int_ena()
                .modify(|_, w| w.cnt_thr_event_u(NUM as u8).set_bit());
        });
    }

    /// Disables interrupts for this unit.
    pub fn unlisten(&self) {
        MUTEX.lock(|| {
            self.pcnt
                .register_block()
                .int_ena()
                .modify(|_, w| w.cnt_thr_event_u(NUM as u8).clear_bit());
        });
    }

    /// Returns whether an interrupt is active for this unit.
    pub fn interrupt_is_set(&self) -> bool {
        self.pcnt
            .register_block()
            .int_raw()
            .read()
            .cnt_thr_event_u(NUM as u8)
            .bit()
    }

    /// Clears the interrupt bit for this unit.
    pub fn reset_interrupt(&self) {
        self.pcnt
            .register_block()
            .int_clr()
            .write(|w| w.cnt_thr_event_u(NUM as u8).set_bit());
    }

    /// Returns the current counter value.
    pub fn value(&self) -> i16 {
        self.counter.get()
    }
}

impl<const NUM: usize> Drop for Unit<'_, NUM> {
    fn drop(&mut self) {
        // This is here to prevent the destructuring of Unit.
    }
}

// The entire Unit is Send but the individual channels are not.
unsafe impl<const NUM: usize> Send for Unit<'_, NUM> {}

/// Represents the counter within a pulse counter unit.
pub struct Counter<'d, const NUM: usize> {
    pcnt: AnyPcnt<'d>,
    _guard: PeripheralGuard,
}

impl<const NUM: usize> Clone for Counter<'_, NUM> {
    fn clone(&self) -> Self {
        Self {
            pcnt: unsafe { self.pcnt.clone_unchecked() },
            _guard: self._guard.clone(),
        }
    }
}

impl<'d, const NUM: usize> Counter<'d, NUM> {
    fn new(pcnt: AnyPcnt<'d>) -> Self {
        let guard = PeripheralGuard::new(pcnt.peripheral());

        Self {
            pcnt,
            _guard: guard,
        }
    }

    /// Returns the current counter value.
    pub fn get(&self) -> i16 {
        let pcnt = self.pcnt.register_block();
        pcnt.u_cnt(NUM).read().cnt().bits() as i16
    }
}
