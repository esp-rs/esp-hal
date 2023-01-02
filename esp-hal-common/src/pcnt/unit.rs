
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

/// Unit configuration
#[derive(Copy, Clone, Default)]
pub struct Config {
    pub low_limit: i16,
    pub high_limit: i16,
    pub filter: Option<u16>,
}

pub struct Unit {
    number: Number,
}

impl Unit {
    /// return a new Unit
    pub(super) fn new(number: Number) -> Self {
        Self {
            number,
        }
    }

    pub fn configure(&mut self, config: Config) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        let (conf0, conf2) = match self.number {
            Number::Unit0 => (&pcnt.u0_conf0, &pcnt.u0_conf2),
            Number::Unit1 => (&pcnt.u1_conf0, &pcnt.u1_conf2),
            Number::Unit2 => (&pcnt.u2_conf0, &pcnt.u2_conf2),
            Number::Unit3 => (&pcnt.u3_conf0, &pcnt.u3_conf2),
            #[cfg(esp32)]
            Number::Unit4 => (&pcnt.u4_conf0, &pcnt.u4_conf2),
            #[cfg(esp32)]
            Number::Unit5 => (&pcnt.u5_conf0, &pcnt.u5_conf2),
            #[cfg(esp32)]
            Number::Unit6 => (&pcnt.u6_conf0, &pcnt.u6_conf2),
            #[cfg(esp32)]
            Number::Unit7 => (&pcnt.u7_conf0, &pcnt.u7_conf2),
        };
        conf2.write(|w| unsafe {
            w.cnt_l_lim_u().bits(config.low_limit as u16)
            .cnt_h_lim_u().bits(config.high_limit as u16)
        });
        if let Some(filter) = config.filter {
            // TODO: needs range checking mac is 1023!
            conf0.modify(|_, w| unsafe {
                w.filter_thres_u().bits(filter)
                .filter_en_u().set_bit()
            });
        } else {
            conf0.modify(|_, w| w.filter_en_u().clear_bit());
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
            Number::Unit0 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u0().set_bit()),
            Number::Unit1 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u1().set_bit()),
            Number::Unit2 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u2().set_bit()),
            Number::Unit3 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u3().set_bit()),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u4().set_bit()),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u5().set_bit()),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u6().set_bit()),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u7().set_bit()),
        }
        // TODO: does this need a delay? (liebman / Jan 2 2023)
        match self.number {
            Number::Unit0 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u0().clear_bit()),
            Number::Unit1 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u1().clear_bit()),
            Number::Unit2 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u2().clear_bit()),
            Number::Unit3 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u3().clear_bit()),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u4().clear_bit()),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u5().clear_bit()),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u6().clear_bit()),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u7().clear_bit()),
        }        
    }

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

    pub fn listen(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_ena.write(|w| match self.number {
            Number::Unit0 => w.cnt_thr_event_u0_int_ena().set_bit(),
            Number::Unit1 => w.cnt_thr_event_u1_int_ena().set_bit(),
            Number::Unit2 => w.cnt_thr_event_u2_int_ena().set_bit(),
            Number::Unit3 => w.cnt_thr_event_u3_int_ena().set_bit(),
            #[cfg(esp32)]
            Number::Unit4 => w.cnt_thr_event_u4_int_ena().set_bit(),
            #[cfg(esp32)]
            Number::Unit5 => w.cnt_thr_event_u5_int_ena().set_bit(),
            #[cfg(esp32)]
            Number::Unit6 => w.cnt_thr_event_u6_int_ena().set_bit(),
            #[cfg(esp32)]
            Number::Unit7 => w.cnt_thr_event_u7_int_ena().set_bit(),
        });
    }

    pub fn unlisten(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_ena.write(|w| match self.number {
            Number::Unit0 => w.cnt_thr_event_u0_int_ena().clear_bit(),
            Number::Unit1 => w.cnt_thr_event_u1_int_ena().clear_bit(),
            Number::Unit2 => w.cnt_thr_event_u2_int_ena().clear_bit(),
            Number::Unit3 => w.cnt_thr_event_u3_int_ena().clear_bit(),
            #[cfg(esp32)]
            Number::Unit4 => w.cnt_thr_event_u4_int_ena().clear_bit(),
            #[cfg(esp32)]
            Number::Unit5 => w.cnt_thr_event_u5_int_ena().clear_bit(),
            #[cfg(esp32)]
            Number::Unit6 => w.cnt_thr_event_u6_int_ena().clear_bit(),
            #[cfg(esp32)]
            Number::Unit7 => w.cnt_thr_event_u7_int_ena().clear_bit(),
        });
    }

    pub fn interrupt_set(&self) -> bool {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        match self.number {
            Number::Unit0 => pcnt.int_st.read().cnt_thr_event_u0_int_st().bit(),
            Number::Unit1 => pcnt.int_st.read().cnt_thr_event_u1_int_st().bit(),
            Number::Unit2 => pcnt.int_st.read().cnt_thr_event_u2_int_st().bit(),
            Number::Unit3 => pcnt.int_st.read().cnt_thr_event_u3_int_st().bit(),
            #[cfg(esp32)]
            Number::Unit4 => pcnt.int_st.read().cnt_thr_event_u4_int_st().bit(),
            #[cfg(esp32)]
            Number::Unit5 => pcnt.int_st.read().cnt_thr_event_u5_int_st().bit(),
            #[cfg(esp32)]
            Number::Unit6 => pcnt.int_st.read().cnt_thr_event_u6_int_st().bit(),
            #[cfg(esp32)]
            Number::Unit7 => pcnt.int_st.read().cnt_thr_event_u7_int_st().bit(),
        }
    }

    pub fn reset_interrupt(&self) {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.int_clr.write(|w| match self.number {
            Number::Unit0 => w.cnt_thr_event_u0_int_clr().set_bit(),
            Number::Unit1 => w.cnt_thr_event_u1_int_clr().set_bit(),
            Number::Unit2 => w.cnt_thr_event_u2_int_clr().set_bit(),
            Number::Unit3 => w.cnt_thr_event_u3_int_clr().set_bit(),
            #[cfg(esp32)]
            Number::Unit4 => w.cnt_thr_event_u4_int_clr().set_bit(),
            #[cfg(esp32)]
            Number::Unit5 => w.cnt_thr_event_u5_int_clr().set_bit(),
            #[cfg(esp32)]
            Number::Unit6 => w.cnt_thr_event_u6_int_clr().set_bit(),
            #[cfg(esp32)]
            Number::Unit7 => w.cnt_thr_event_u7_int_clr().set_bit(),
        });
    }

    pub fn get_value(&self) -> i16 {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        pcnt.u_cnt[self.number as usize].read().pulse_cnt_u().bits() as i16
    }
}
