use super::channel;

/// Unit number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Unit0,
    Unit1,
    Unit2,
    Unit3,
    #[cfg(esp32)]
    Unit4,
    #[cfg(esp32)]
    Unit5,
    #[cfg(esp32)]
    Unit6,
    #[cfg(esp32)]
    Unit7,
}

/// the current status of the counter.
#[derive(Copy, Clone, Debug, Default)]
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
pub struct Events {
    pub low_limit: bool,
    pub high_limit: bool,
    pub thresh0: bool,
    pub thresh1: bool,
    pub zero: bool,
}

/// Unit configuration
#[derive(Copy, Clone, Default)]
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
        Self { number }
    }

    pub fn configure(&mut self, config: Config) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let (conf0, conf1, conf2) = match self.number {
            Number::Unit0 => (&pcnt.u0_conf0, &pcnt.u0_conf1, &pcnt.u0_conf2),
            Number::Unit1 => (&pcnt.u1_conf0, &pcnt.u1_conf1, &pcnt.u1_conf2),
            Number::Unit2 => (&pcnt.u2_conf0, &pcnt.u2_conf1, &pcnt.u2_conf2),
            Number::Unit3 => (&pcnt.u3_conf0, &pcnt.u3_conf1, &pcnt.u3_conf2),
            #[cfg(esp32)]
            Number::Unit4 => (&pcnt.u4_conf0, &pcnt.u4_conf1, &pcnt.u4_conf2),
            #[cfg(esp32)]
            Number::Unit5 => (&pcnt.u5_conf0, &pcnt.u5_conf1, &pcnt.u5_conf2),
            #[cfg(esp32)]
            Number::Unit6 => (&pcnt.u6_conf0, &pcnt.u6_conf1, &pcnt.u6_conf2),
            #[cfg(esp32)]
            Number::Unit7 => (&pcnt.u7_conf0, &pcnt.u7_conf1, &pcnt.u7_conf2),
        };
        // TODO: needs range checking low must be < 0 and high > 0!
        conf2.write(|w| unsafe {
            w.cnt_l_lim()
                .bits(config.low_limit as u16)
                .cnt_h_lim()
                .bits(config.high_limit as u16)
        });
        conf1.write(|w| unsafe {
            w.cnt_thres0()
                .bits(config.thresh0 as u16)
                .cnt_thres1()
                .bits(config.thresh1 as u16)
        });
        if let Some(filter) = config.filter {
            // TODO: needs range checking max is 1023!
            conf0.modify(|_, w| unsafe { w.filter_thres().bits(filter).filter_en().set_bit() });
        } else {
            conf0.modify(|_, w| w.filter_en().clear_bit());
        }
        self.pause();
        self.clear();
    }

    pub fn get_channel(&self, number: channel::Number) -> super::channel::Channel {
        super::channel::Channel::new(self.number, number)
    }

    pub fn clear(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        match self.number {
            Number::Unit0 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u0().set_bit()),
            Number::Unit1 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u1().set_bit()),
            Number::Unit2 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u2().set_bit()),
            Number::Unit3 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u3().set_bit()),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u4().set_bit()),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u5().set_bit()),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u6().set_bit()),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u7().set_bit()),
        }
        // TODO: does this need a delay? (liebman / Jan 2 2023)
        match self.number {
            Number::Unit0 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u0().clear_bit()),
            Number::Unit1 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u1().clear_bit()),
            Number::Unit2 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u2().clear_bit()),
            Number::Unit3 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u3().clear_bit()),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u4().clear_bit()),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u5().clear_bit()),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u6().clear_bit()),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.ctrl.modify(|_, w| w.cnt_rst_u7().clear_bit()),
        }
    }

    /// Pause the counter
    pub fn pause(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        match self.number {
            Number::Unit0 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u0().set_bit()),
            Number::Unit1 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u1().set_bit()),
            Number::Unit2 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u2().set_bit()),
            Number::Unit3 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u3().set_bit()),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u4().set_bit()),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u5().set_bit()),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u6().set_bit()),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u7().set_bit()),
        }
    }

    /// Resume the counter
    pub fn resume(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        match self.number {
            Number::Unit0 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u0().clear_bit()),
            Number::Unit1 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u1().clear_bit()),
            Number::Unit2 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u2().clear_bit()),
            Number::Unit3 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u3().clear_bit()),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u4().clear_bit()),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u5().clear_bit()),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u6().clear_bit()),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.ctrl.modify(|_, w| w.cnt_pause_u7().clear_bit()),
        }
    }

    /// Enable which events generate interrupts on this unit.
    pub fn events(&self, events: Events) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let conf0 = match self.number {
            Number::Unit0 => &pcnt.u0_conf0,
            Number::Unit1 => &pcnt.u1_conf0,
            Number::Unit2 => &pcnt.u2_conf0,
            Number::Unit3 => &pcnt.u3_conf0,
            #[cfg(esp32)]
            Number::Unit4 => &pcnt.u4_conf0,
            #[cfg(esp32)]
            Number::Unit5 => &pcnt.u5_conf0,
            #[cfg(esp32)]
            Number::Unit6 => &pcnt.u6_conf0,
            #[cfg(esp32)]
            Number::Unit7 => &pcnt.u7_conf0,
        };
        conf0.modify(|_, w| {
            w.thr_l_lim_en()
                .bit(events.low_limit)
                .thr_h_lim_en()
                .bit(events.high_limit)
                .thr_thres0_en()
                .bit(events.thresh0)
                .thr_thres1_en()
                .bit(events.thresh1)
                .thr_zero_en()
                .bit(events.zero)
        });
    }

    /// Get the latest events for this unit.
    pub fn get_events(&self) -> Events {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let status = pcnt.u_status[self.number as usize].read();

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
        pcnt.u_status[self.number as usize].read().zero_mode().bits().into()
    }

    /// Enable interrupts for this unit.
    pub fn listen(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_ena.modify(|_, w| match self.number {
            Number::Unit0 => w.cnt_thr_event_u0().set_bit(),
            Number::Unit1 => w.cnt_thr_event_u1().set_bit(),
            Number::Unit2 => w.cnt_thr_event_u2().set_bit(),
            Number::Unit3 => w.cnt_thr_event_u3().set_bit(),
            #[cfg(esp32)]
            Number::Unit4 => w.cnt_thr_event_u4().set_bit(),
            #[cfg(esp32)]
            Number::Unit5 => w.cnt_thr_event_u5().set_bit(),
            #[cfg(esp32)]
            Number::Unit6 => w.cnt_thr_event_u6().set_bit(),
            #[cfg(esp32)]
            Number::Unit7 => w.cnt_thr_event_u7().set_bit(),
        });
    }

    /// Disable interrupts for this unit.
    pub fn unlisten(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_ena.write(|w| match self.number {
            Number::Unit0 => w.cnt_thr_event_u0().clear_bit(),
            Number::Unit1 => w.cnt_thr_event_u1().clear_bit(),
            Number::Unit2 => w.cnt_thr_event_u2().clear_bit(),
            Number::Unit3 => w.cnt_thr_event_u3().clear_bit(),
            #[cfg(esp32)]
            Number::Unit4 => w.cnt_thr_event_u4().clear_bit(),
            #[cfg(esp32)]
            Number::Unit5 => w.cnt_thr_event_u5().clear_bit(),
            #[cfg(esp32)]
            Number::Unit6 => w.cnt_thr_event_u6().clear_bit(),
            #[cfg(esp32)]
            Number::Unit7 => w.cnt_thr_event_u7().clear_bit(),
        });
    }

    /// Returns true if an interrupt is active for this unit.
    pub fn interrupt_set(&self) -> bool {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        match self.number {
            Number::Unit0 => pcnt.int_st.read().cnt_thr_event_u0().bit(),
            Number::Unit1 => pcnt.int_st.read().cnt_thr_event_u1().bit(),
            Number::Unit2 => pcnt.int_st.read().cnt_thr_event_u2().bit(),
            Number::Unit3 => pcnt.int_st.read().cnt_thr_event_u3().bit(),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.int_st.read().cnt_thr_event_u4().bit(),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.int_st.read().cnt_thr_event_u5().bit(),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.int_st.read().cnt_thr_event_u6().bit(),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.int_st.read().cnt_thr_event_u7().bit(),
        }
    }

    /// Clear the interrupt bit for this unit.
    pub fn reset_interrupt(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_clr.write(|w| match self.number {
            Number::Unit0 => w.cnt_thr_event_u0().set_bit(),
            Number::Unit1 => w.cnt_thr_event_u1().set_bit(),
            Number::Unit2 => w.cnt_thr_event_u2().set_bit(),
            Number::Unit3 => w.cnt_thr_event_u3().set_bit(),
            #[cfg(esp32)]
            Number::Unit4 => w.cnt_thr_event_u4().set_bit(),
            #[cfg(esp32)]
            Number::Unit5 => w.cnt_thr_event_u5().set_bit(),
            #[cfg(esp32)]
            Number::Unit6 => w.cnt_thr_event_u6().set_bit(),
            #[cfg(esp32)]
            Number::Unit7 => w.cnt_thr_event_u7().set_bit(),
        });
    }

    /// Get the current counter value.
    pub fn get_value(&self) -> i16 {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.u_cnt[self.number as usize].read().cnt().bits() as i16
    }
}
