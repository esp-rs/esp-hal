use esp32s3::pcnt::RegisterBlock;

use super::channel;

/// Unit number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Unit0,
    Unit1,
    Unit2,
    Unit3,
    #[cfg(not(esp32s3))]
    Unit4,
    #[cfg(not(esp32s3))]
    Unit5,
    #[cfg(not(esp32s3))]
    Unit6,
    #[cfg(not(esp32s3))]
    Unit7,
}

/// Unit configuration
#[derive(Copy, Clone)]
pub struct Config {
    pub low_limit: i16,
    pub high_limit: i16,
}

pub struct Unit<'a> {
    pcnt: &'a RegisterBlock,
    number: Number,
}

impl<'a> Unit<'a> {
    /// return a new Unit
    pub fn new(number: Number) -> Self {
        let pcnt = unsafe { &*crate::peripherals::PCNT::ptr() };
        Self {
            pcnt,
            number,
        }
    }

    pub fn configure(&mut self, config: Config) {

        match self.number {
            Number::Unit0 => {
                self.pcnt.u0_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
            Number::Unit1 => {
                self.pcnt.u1_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
            Number::Unit2 => {
                self.pcnt.u2_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
            Number::Unit3 => {
                self.pcnt.u3_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
            #[cfg(not(esp32s3))]
            Number::Unit4 => {
                self.pcnt.u4_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
            #[cfg(not(esp32s3))]
            Number::Unit5 => {
                self.pcnt.u5_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
            #[cfg(not(esp32s3))]
            Number::Unit6 => {
                self.pcnt.u6_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
            #[cfg(not(esp32s3))]
            Number::Unit7 => {
                self.pcnt.u7_conf2.write(|w| unsafe {
                    w.cnt_l_lim_u().bits(config.low_limit as u16)
                    .cnt_h_lim_u().bits(config.high_limit as u16)
                });
            },
        }
        self.pause();
        self.clear();
    }

    pub fn get_channel(&self, number: channel::Number) -> super::channel::Channel {
        super::channel::Channel::new(self.number, number)
    }

    pub fn clear(&self) {
        match self.number {
            Number::Unit0 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u0().set_bit()),
            Number::Unit1 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u1().set_bit()),
            Number::Unit2 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u2().set_bit()),
            Number::Unit3 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u3().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit4 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u4().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit5 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u5().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit6 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u6().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit7 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u7().set_bit()),
        }
        // TODO: does this need a delay? (liebman / Jan 2 2023)
        match self.number {
            Number::Unit0 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u0().clear_bit()),
            Number::Unit1 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u1().clear_bit()),
            Number::Unit2 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u2().clear_bit()),
            Number::Unit3 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u3().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit4 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u4().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit5 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u5().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit6 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u6().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit7 => self.pcnt.ctrl.modify(|_, w| w.pulse_cnt_rst_u7().clear_bit()),
        }        
    }

    pub fn pause(&self) {
        match self.number {
            Number::Unit0 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u0().set_bit()),
            Number::Unit1 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u1().set_bit()),
            Number::Unit2 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u2().set_bit()),
            Number::Unit3 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u3().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit4 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u4().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit5 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u5().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit6 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u6().set_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit7 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u7().set_bit()),
        }        
    }

    pub fn resume(&self) {
        match self.number {
            Number::Unit0 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u0().clear_bit()),
            Number::Unit1 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u1().clear_bit()),
            Number::Unit2 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u2().clear_bit()),
            Number::Unit3 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u3().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit4 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u4().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit5 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u5().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit6 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u6().clear_bit()),
            #[cfg(not(esp32s3))]
            Number::Unit7 => self.pcnt.ctrl.modify(|_, w| w.cnt_pause_u7().clear_bit()),
        }        
    }

    pub fn get_value(&self) -> i16 {
        self.pcnt.u_cnt[self.number as usize].read().pulse_cnt_u().bits() as i16
    }
}
