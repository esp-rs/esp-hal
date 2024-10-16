//! # PCNT - Unit Module
//!
//! ## Overview
//! The `unit` module is responsible for configuring and handling individual
//! units of the `PCNT` peripheral. Each unit represents a separate instance of
//! the `PCNT` module, identified by unit numbers like `Unit0`, `Unit1`, and so
//! on. Users can interact with these units to configure settings such as low
//! and high limits, thresholds, and optional filtering. The unit module also
//! enables users to pause, resume, and clear the counter, as well as enable or
//! disable interrupts for specific events associated with the unit.

use core::marker::PhantomData;

use critical_section::CriticalSection;

use crate::pcnt::channel::Channel;

/// Invalid filter threshold value
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidFilterThreshold;

/// Invalid low limit - must be < 0
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InvalidLowLimit;

/// Invalid high limit - must be > 0
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
    /// Set when the pulse counter reaches the low limit.
    pub low_limit: bool,
    /// Set when the pulse counter reaches the high limit.
    pub high_limit: bool,
    /// Set when the pulse counter crosses threshold 0.
    pub threshold0: bool,
    /// Set when the pulse counter crosses threshold 1.
    pub threshold1: bool,
    /// Set when the pulse counter reaches zero.
    pub zero: bool,
}

/// Represents a pulse counter unit.
#[non_exhaustive]
pub struct Unit<'d, const NUM: usize> {
    /// The counter for PCNT unit.
    pub counter: Counter<'d, NUM>,
    /// The first channel in PCNT unit.
    pub channel0: Channel<'d, NUM, 0>,
    /// The second channel in PCNT unit.
    pub channel1: Channel<'d, NUM, 1>,
}

impl<const NUM: usize> Unit<'_, NUM> {
    /// return a new Unit
    pub(super) fn new() -> Self {
        Self {
            counter: Counter::new(),
            channel0: Channel::new(),
            channel1: Channel::new(),
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
    /// Note: The specified value must be negative.
    pub fn set_low_limit(&self, value: Option<i16>) -> Result<(), InvalidLowLimit> {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
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
    /// Note: The specified value must be positive.
    pub fn set_high_limit(&self, value: Option<i16>) -> Result<(), InvalidHighLimit> {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
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
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
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
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
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
    /// Note: This maximum possible threshold is 1023.
    pub fn set_filter(&self, threshold: Option<u16>) -> Result<(), InvalidFilterThreshold> {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
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
                    w.filter_thres().bits(threshold).filter_en().set_bit()
                });
            }
        }
        Ok(())
    }

    /// Resets the counter value to zero.
    pub fn clear(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.ctrl().modify(|_, w| w.cnt_rst_u(NUM as u8).set_bit());
            // TODO: does this need a delay? (liebman / Jan 2 2023)
            pcnt.ctrl()
                .modify(|_, w| w.cnt_rst_u(NUM as u8).clear_bit());
        });
    }

    /// Pause the counter
    pub fn pause(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.ctrl()
                .modify(|_, w| w.cnt_pause_u(NUM as u8).set_bit());
        });
    }

    /// Resume the counter
    pub fn resume(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.ctrl()
                .modify(|_, w| w.cnt_pause_u(NUM as u8).clear_bit());
        });
    }

    /// Get the latest events for this unit.
    pub fn get_events(&self) -> Events {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let status = pcnt.u_status(NUM).read();

        Events {
            low_limit: status.l_lim().bit(),
            high_limit: status.h_lim().bit(),
            threshold0: status.thres0().bit(),
            threshold1: status.thres1().bit(),
            zero: status.zero().bit(),
        }
    }

    /// Get the mode of the last zero crossing
    pub fn get_zero_mode(&self) -> ZeroMode {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.u_status(NUM).read().zero_mode().bits().into()
    }

    /// Enable interrupts for this unit.
    pub fn listen(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.int_ena()
                .modify(|_, w| w.cnt_thr_event_u(NUM as u8).set_bit());
        });
    }

    /// Disable interrupts for this unit.
    pub fn unlisten(&self, _cs: CriticalSection<'_>) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.int_ena()
                .modify(|_, w| w.cnt_thr_event_u(NUM as u8).clear_bit());
        });
    }

    /// Returns true if an interrupt is active for this unit.
    pub fn interrupt_is_set(&self) -> bool {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_raw().read().cnt_thr_event_u(NUM as u8).bit()
    }

    /// Clear the interrupt bit for this unit.
    pub fn reset_interrupt(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.int_clr()
                .write(|w| w.cnt_thr_event_u(NUM as u8).set_bit());
        });
    }

    /// Get the current counter value.
    pub fn get_value(&self) -> i16 {
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
#[derive(Clone)]
pub struct Counter<'d, const NUM: usize> {
    _phantom: PhantomData<&'d ()>,
}

impl<const NUM: usize> Counter<'_, NUM> {
    fn new() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }

    /// Get the current counter value.
    pub fn get(&self) -> i16 {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.u_cnt(NUM).read().cnt().bits() as i16
    }
}
