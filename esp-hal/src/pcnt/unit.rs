//! # PCNT - Unit module
//!
//! ## Overview
//! The `unit` module is a part of the `PCNT` peripheral driver
//! for ESP chips. It offers functionalities for configuring and utilizing
//! specific units of the PCNT peripheral.
//!
//! Each unit is identified by a unit number, such as `Unit0`, `Unit1`, `Unit2`,
//! and so on. This module provides methods to configure a unit with specific
//! settings, like low and high limits, thresholds, and optional filtering.
//! Users can easily configure these units based on their requirements.

use critical_section::CriticalSection;

use super::channel;

/// Unit number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Unit0 = 0,
    Unit1 = 1,
    Unit2 = 2,
    Unit3 = 3,
    #[cfg(esp32)]
    Unit4 = 4,
    #[cfg(esp32)]
    Unit5 = 5,
    #[cfg(esp32)]
    Unit6 = 6,
    #[cfg(esp32)]
    Unit7 = 7,
}

/// Unit errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid filter threshold value
    InvalidFilterThresh,
    /// Invalid low limit - must be < 0
    InvalidLowLimit,
    /// Invalid high limit - must be > 0
    InvalidHighLimit,
}

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
    Negitive = 2,
    /// pulse counter is positive (not implemented?)
    Positive = 3,
}

impl From<u8> for ZeroMode {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::PosZero,
            1 => Self::NegZero,
            2 => Self::Negitive,
            3 => Self::Positive,
            _ => unreachable!(), // TODO: is this good enoough?  should we use some default?
        }
    }
}

// Events
#[derive(Copy, Clone, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Events {
    pub low_limit: bool,
    pub high_limit: bool,
    pub thresh0: bool,
    pub thresh1: bool,
    pub zero: bool,
}

/// Unit configuration
#[derive(Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    pub low_limit: i16,
    pub high_limit: i16,
    pub thresh0: i16,
    pub thresh1: i16,
    pub filter: Option<u16>,
}

pub struct Unit {
    number: Number,
}

impl Unit {
    /// return a new Unit
    pub(super) fn new(number: Number) -> Self {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        // disable filter and all events
        pcnt.unit(number as usize).conf0().modify(|_, w| {
            w.filter_en().clear_bit();
            unsafe {
                w.filter_thres().bits(0);
            }
            w.thr_l_lim_en().clear_bit();
            w.thr_h_lim_en().clear_bit();
            w.thr_thres0_en().clear_bit();
            w.thr_thres1_en().clear_bit();
            w.thr_zero_en().clear_bit()
        });
        Self { number }
    }

    pub fn configure(&mut self, config: Config) -> Result<(), Error> {
        // low limit must be >= or the limit is -32768 and when thats
        // hit the event status claims it was the high limit.
        // tested on an esp32s3
        if config.low_limit >= 0 {
            return Err(Error::InvalidLowLimit);
        }
        if config.high_limit <= 0 {
            return Err(Error::InvalidHighLimit);
        }
        let (filter_en, filter) = match config.filter {
            Some(filter) => (true, filter),
            None => (false, 0),
        };
        // filter must be less than 1024
        if filter > 1023 {
            return Err(Error::InvalidFilterThresh);
        }

        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let unit = pcnt.unit(self.number as usize);
        unit.conf2().write(|w| unsafe {
            w.cnt_l_lim().bits(config.low_limit as u16);
            w.cnt_h_lim().bits(config.high_limit as u16)
        });
        unit.conf1().write(|w| unsafe {
            w.cnt_thres0().bits(config.thresh0 as u16);
            w.cnt_thres1().bits(config.thresh1 as u16)
        });
        unit.conf0().modify(|_, w| unsafe {
            w.filter_thres().bits(filter);
            w.filter_en().bit(filter_en)
        });
        self.pause();
        self.clear();
        Ok(())
    }

    pub fn get_channel(&self, number: channel::Number) -> super::channel::Channel {
        super::channel::Channel::new(self.number, number)
    }

    pub fn clear(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.ctrl()
                .modify(|_, w| w.cnt_rst_u(self.number as u8).set_bit());
            // TODO: does this need a delay? (liebman / Jan 2 2023)
            pcnt.ctrl()
                .modify(|_, w| w.cnt_rst_u(self.number as u8).clear_bit());
        });
    }

    /// Pause the counter
    pub fn pause(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.ctrl()
                .modify(|_, w| w.cnt_pause_u(self.number as u8).set_bit());
        });
    }

    /// Resume the counter
    pub fn resume(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.ctrl()
                .modify(|_, w| w.cnt_pause_u(self.number as u8).clear_bit());
        });
    }

    /// Enable which events generate interrupts on this unit.
    pub fn events(&self, events: Events) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.unit(self.number as usize).conf0().modify(|_, w| {
            w.thr_l_lim_en().bit(events.low_limit);
            w.thr_h_lim_en().bit(events.high_limit);
            w.thr_thres0_en().bit(events.thresh0);
            w.thr_thres1_en().bit(events.thresh1);
            w.thr_zero_en().bit(events.zero)
        });
    }

    /// Get the latest events for this unit.
    pub fn get_events(&self) -> Events {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let status = pcnt.u_status(self.number as usize).read();

        Events {
            low_limit: status.l_lim().bit(),
            high_limit: status.h_lim().bit(),
            thresh0: status.thres0().bit(),
            thresh1: status.thres1().bit(),
            zero: status.zero().bit(),
        }
    }

    /// Get the mode of the last zero crossing
    pub fn get_zero_mode(&self) -> ZeroMode {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.u_status(self.number as usize)
            .read()
            .zero_mode()
            .bits()
            .into()
    }

    /// Enable interrupts for this unit.
    pub fn listen(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.int_ena()
                .modify(|_, w| w.cnt_thr_event_u(self.number as u8).set_bit());
        });
    }

    /// Disable interrupts for this unit.
    pub fn unlisten(&self, _cs: CriticalSection) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.int_ena()
                .write(|w| w.cnt_thr_event_u(self.number as u8).clear_bit());
        });
    }

    /// Returns true if an interrupt is active for this unit.
    pub fn interrupt_set(&self) -> bool {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_st()
            .read()
            .cnt_thr_event_u(self.number as u8)
            .bit()
    }

    /// Clear the interrupt bit for this unit.
    pub fn reset_interrupt(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        critical_section::with(|_cs| {
            pcnt.int_clr()
                .write(|w| w.cnt_thr_event_u(self.number as u8).set_bit());
        });
    }

    /// Get the current counter value.
    pub fn get_value(&self) -> i16 {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.u_cnt(self.number as usize).read().cnt().bits() as i16
    }
}
