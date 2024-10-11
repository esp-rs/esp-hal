//! # LEDC channel
//!
//! ## Overview
//! The LEDC Channel module  provides a high-level interface to
//! configure and control individual PWM channels of the LEDC peripheral.
//!
//! ## Configuration
//! The module allows precise and flexible control over LED lighting and other
//! `Pulse-Width Modulation (PWM)` applications by offering configurable duty
//! cycles and frequencies.

use super::timer::{TimerIFace, TimerSpeed};
use crate::{
    gpio::{interconnect::OutputConnection, OutputSignal},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::ledc::RegisterBlock,
};

/// Fade parameter sub-errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FadeError {
    /// Start duty % out of range
    StartDuty,
    /// End duty % out of range
    EndDuty,
    /// Duty % change from start to end is out of range
    DutyRange,
    /// Duration too long for timer frequency and duty resolution
    Duration,
}

/// Channel errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid duty % value
    Duty,
    /// Timer not configured
    Timer,
    /// Channel not configured
    Channel,
    /// Fade parameters invalid
    Fade(FadeError),
}

/// Channel number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Number {
    /// Channel 0
    Channel0 = 0,
    /// Channel 1
    Channel1 = 1,
    /// Channel 2
    Channel2 = 2,
    /// Channel 3
    Channel3 = 3,
    /// Channel 4
    Channel4 = 4,
    /// Channel 5
    Channel5 = 5,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    /// Channel 6
    Channel6 = 6,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    /// Channel 7
    Channel7 = 7,
}

/// Channel configuration
pub mod config {
    use crate::ledc::timer::{TimerIFace, TimerSpeed};

    #[derive(Debug, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    /// Pin configuration for the LEDC channel.
    pub enum PinConfig {
        /// Push-pull pin configuration.
        PushPull,
        /// Open-drain pin configuration.
        OpenDrain,
    }

    /// Channel configuration
    #[derive(Copy, Clone)]
    pub struct Config<'d, S: TimerSpeed> {
        /// A reference to the timer associated with this channel.
        pub timer: &'d dyn TimerIFace<S>,
        /// The duty cycle percentage (0-100).
        pub duty_pct: u8,
        /// The pin configuration (PushPull or OpenDrain).
        pub pin_config: PinConfig,
    }
}

/// Channel interface
pub trait ChannelIFace<'d, S: TimerSpeed> {
    /// Configure channel
    fn configure(&mut self, config: config::Config<'d, S>) -> Result<(), Error>;

    /// Set channel duty HW
    fn set_duty(&self, duty_pct: u8) -> Result<(), Error>;

    /// Start a duty-cycle fade
    fn start_duty_fade(
        &self,
        start_duty_pct: u8,
        end_duty_pct: u8,
        duration_ms: u16,
    ) -> Result<(), Error>;

    /// Check whether a duty-cycle fade is running
    fn is_duty_fade_running(&self) -> bool;
}

/// Channel HW interface
pub trait ChannelHW {
    /// Configure Channel HW except for the duty which is set via
    /// [`Self::set_duty_hw`].
    fn configure_hw(&mut self) -> Result<(), Error>;
    /// Configure the hardware for the channel with a specific pin
    /// configuration.
    fn configure_hw_with_pin_config(&mut self, cfg: config::PinConfig) -> Result<(), Error>;

    /// Set channel duty HW
    fn set_duty_hw(&self, duty: u32);

    /// Start a duty-cycle fade HW
    fn start_duty_fade_hw(
        &self,
        start_duty: u32,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    );

    /// Check whether a duty-cycle fade is running HW
    fn is_duty_fade_running_hw(&self) -> bool;
}

/// Channel struct
pub struct Channel<'d, S: TimerSpeed> {
    ledc: &'d RegisterBlock,
    timer: Option<&'d dyn TimerIFace<S>>,
    number: Number,
    output_pin: PeripheralRef<'d, OutputConnection>,
}

impl<'d, S: TimerSpeed> Channel<'d, S> {
    /// Return a new channel
    pub fn new(
        number: Number,
        output_pin: impl Peripheral<P = impl Into<OutputConnection> + 'd> + 'd,
    ) -> Self {
        crate::into_ref!(output_pin);
        let ledc = unsafe { &*crate::peripherals::LEDC::ptr() };
        Channel {
            ledc,
            timer: None,
            number,
            output_pin: output_pin.map_into(),
        }
    }
}

impl<'d, S: TimerSpeed> ChannelIFace<'d, S> for Channel<'d, S>
where
    Self: ChannelHW,
{
    /// Configure channel
    fn configure(&mut self, config: config::Config<'d, S>) -> Result<(), Error> {
        self.timer = Some(config.timer);

        self.set_duty(config.duty_pct)?;
        self.configure_hw_with_pin_config(config.pin_config)?;

        Ok(())
    }

    /// Set duty % of channel
    fn set_duty(&self, duty_pct: u8) -> Result<(), Error> {
        let duty_exp;
        if let Some(timer) = self.timer {
            if let Some(timer_duty) = timer.get_duty() {
                duty_exp = timer_duty as u32;
            } else {
                return Err(Error::Timer);
            }
        } else {
            return Err(Error::Channel);
        }

        let duty_range = 2u32.pow(duty_exp);
        let duty_value = (duty_range * duty_pct as u32) / 100;

        if duty_pct > 100u8 {
            // duty_pct greater than 100%
            return Err(Error::Duty);
        }

        self.set_duty_hw(duty_value);

        Ok(())
    }

    /// Start a duty fade from one % to another.
    ///
    /// There's a constraint on the combination of timer frequency, timer PWM
    /// duty resolution (the bit count), the fade "range" (abs(start-end)), and
    /// the duration:
    ///
    /// frequency * duration / ((1<<bit_count) * abs(start-end)) < 1024
    ///
    /// Small percentage changes, long durations, coarse PWM resolutions (that
    /// is, low bit counts), and high timer frequencies will all be more likely
    /// to fail this requirement.  If it does fail, this function will return
    /// an error Result.
    fn start_duty_fade(
        &self,
        start_duty_pct: u8,
        end_duty_pct: u8,
        duration_ms: u16,
    ) -> Result<(), Error> {
        let duty_exp;
        let frequency;
        if start_duty_pct > 100u8 {
            return Err(Error::Fade(FadeError::StartDuty));
        }
        if end_duty_pct > 100u8 {
            return Err(Error::Fade(FadeError::EndDuty));
        }
        if let Some(timer) = self.timer {
            if let Some(timer_duty) = timer.get_duty() {
                if timer.get_frequency() > 0 {
                    duty_exp = timer_duty as u32;
                    frequency = timer.get_frequency();
                } else {
                    return Err(Error::Timer);
                }
            } else {
                return Err(Error::Timer);
            }
        } else {
            return Err(Error::Channel);
        }

        let duty_range = (1u32 << duty_exp) - 1;
        let start_duty_value = (duty_range * start_duty_pct as u32) / 100;
        let end_duty_value = (duty_range * end_duty_pct as u32) / 100;

        // NB: since we do the multiplication first here, there's no loss of
        // precision from using milliseconds instead of (e.g.) nanoseconds.
        let pwm_cycles = (duration_ms as u32) * frequency / 1000;

        let abs_duty_diff = end_duty_value.abs_diff(start_duty_value);
        let duty_steps: u32 = u16::try_from(abs_duty_diff).unwrap_or(65535).into();
        // This conversion may fail if duration_ms is too big, and if either
        // duty_steps gets truncated, or the fade is over a short range of duty
        // percentages, so it's too small.  Returning an Err in either case is
        // fine: shortening the duration_ms will sort things out.
        let cycles_per_step: u16 = (pwm_cycles / duty_steps)
            .try_into()
            .map_err(|_| Error::Fade(FadeError::Duration))
            .and_then(|res| {
                if res > 1023 {
                    Err(Error::Fade(FadeError::Duration))
                } else {
                    Ok(res)
                }
            })?;
        // This can't fail unless abs_duty_diff is bigger than 65536*65535-1,
        // and so duty_steps gets truncated.  But that requires duty_exp to be
        // at least 32, and the hardware only supports up to 20.  Still, handle
        // it in case something changes in the future.
        let duty_per_cycle: u16 = (abs_duty_diff / duty_steps)
            .try_into()
            .map_err(|_| Error::Fade(FadeError::DutyRange))?;

        self.start_duty_fade_hw(
            start_duty_value,
            end_duty_value > start_duty_value,
            duty_steps.try_into().unwrap(),
            cycles_per_step,
            duty_per_cycle,
        );

        Ok(())
    }

    fn is_duty_fade_running(&self) -> bool {
        self.is_duty_fade_running_hw()
    }
}

mod ehal1 {
    use embedded_hal::pwm::{self, ErrorKind, ErrorType, SetDutyCycle};

    use super::{Channel, ChannelHW, Error};
    use crate::ledc::timer::TimerSpeed;

    impl pwm::Error for Error {
        fn kind(&self) -> pwm::ErrorKind {
            ErrorKind::Other
        }
    }

    impl<S: TimerSpeed> ErrorType for Channel<'_, S> {
        type Error = Error;
    }

    impl<S: TimerSpeed> SetDutyCycle for Channel<'_, S>
    where
        Self: ChannelHW,
    {
        fn max_duty_cycle(&self) -> u16 {
            let duty_exp;

            if let Some(timer_duty) = self.timer.and_then(|timer| timer.get_duty()) {
                duty_exp = timer_duty as u32;
            } else {
                return 0;
            }

            let duty_range = 2u32.pow(duty_exp);

            duty_range as u16
        }

        fn set_duty_cycle(&mut self, mut duty: u16) -> Result<(), Self::Error> {
            let max = self.max_duty_cycle();
            duty = if duty > max { max } else { duty };
            self.set_duty_hw(duty.into());
            Ok(())
        }
    }
}

impl<S: crate::ledc::timer::TimerSpeed> Channel<'_, S> {
    #[cfg(esp32)]
    fn set_channel(&mut self, timer_number: u8) {
        if S::IS_HS {
            let ch = self.ledc.hsch(self.number as usize);
            ch.hpoint().write(|w| unsafe { w.hpoint().bits(0x0) });
            ch.conf0()
                .modify(|_, w| unsafe { w.sig_out_en().set_bit().timer_sel().bits(timer_number) });
        } else {
            let ch = self.ledc.lsch(self.number as usize);
            ch.hpoint().write(|w| unsafe { w.hpoint().bits(0x0) });
            ch.conf0()
                .modify(|_, w| unsafe { w.sig_out_en().set_bit().timer_sel().bits(timer_number) });
        }
        self.start_duty_without_fading();
    }
    #[cfg(not(esp32))]
    fn set_channel(&mut self, timer_number: u8) {
        {
            let ch = self.ledc.ch(self.number as usize);
            ch.hpoint().write(|w| unsafe { w.hpoint().bits(0x0) });
            ch.conf0().modify(|_, w| {
                w.sig_out_en().set_bit();
                unsafe { w.timer_sel().bits(timer_number) }
            });
        }
        self.start_duty_without_fading();
    }

    #[cfg(esp32)]
    fn start_duty_without_fading(&self) {
        if S::IS_HS {
            self.ledc
                .hsch(self.number as usize)
                .conf1()
                .write(|w| unsafe {
                    w.duty_start().set_bit();
                    w.duty_inc().set_bit();
                    w.duty_num().bits(0x1);
                    w.duty_cycle().bits(0x1);
                    w.duty_scale().bits(0x0)
                });
        } else {
            self.ledc
                .lsch(self.number as usize)
                .conf1()
                .write(|w| unsafe {
                    w.duty_start().set_bit();
                    w.duty_inc().set_bit();
                    w.duty_num().bits(0x1);
                    w.duty_cycle().bits(0x1);
                    w.duty_scale().bits(0x0)
                });
        }
    }
    #[cfg(any(esp32c6, esp32h2))]
    fn start_duty_without_fading(&self) {
        let cnum = self.number as usize;
        self.ledc
            .ch(cnum)
            .conf1()
            .write(|w| w.duty_start().set_bit());
        self.ledc.ch_gamma_wr(cnum).write(|w| {
            w.ch_gamma_duty_inc().set_bit();
            unsafe {
                w.ch_gamma_duty_num().bits(0x1);
                w.ch_gamma_duty_cycle().bits(0x1);
                w.ch_gamma_scale().bits(0x0)
            }
        });
    }
    #[cfg(not(any(esp32, esp32c6, esp32h2)))]
    fn start_duty_without_fading(&self) {
        self.ledc.ch(self.number as usize).conf1().write(|w| {
            w.duty_start().set_bit();
            w.duty_inc().set_bit();
            unsafe {
                w.duty_num().bits(0x1);
                w.duty_cycle().bits(0x1);
                w.duty_scale().bits(0x0)
            }
        });
    }

    #[cfg(esp32)]
    fn start_duty_fade_inner(
        &self,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        if S::IS_HS {
            self.ledc
                .hsch(self.number as usize)
                .conf1()
                .write(|w| unsafe {
                    w.duty_start()
                        .set_bit()
                        .duty_inc()
                        .variant(duty_inc)
                        .duty_num() // count of incs before stopping
                        .bits(duty_steps)
                        .duty_cycle() // overflows between incs
                        .bits(cycles_per_step)
                        .duty_scale()
                        .bits(duty_per_cycle)
                });
        } else {
            self.ledc
                .lsch(self.number as usize)
                .conf1()
                .write(|w| unsafe {
                    w.duty_start()
                        .set_bit()
                        .duty_inc()
                        .variant(duty_inc)
                        .duty_num() // count of incs before stopping
                        .bits(duty_steps)
                        .duty_cycle() // overflows between incs
                        .bits(cycles_per_step)
                        .duty_scale()
                        .bits(duty_per_cycle)
                });
        }
    }

    #[cfg(any(esp32c6, esp32h2))]
    fn start_duty_fade_inner(
        &self,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        let cnum = self.number as usize;
        self.ledc
            .ch(cnum)
            .conf1()
            .write(|w| w.duty_start().set_bit());
        self.ledc.ch_gamma_wr(cnum).write(|w| unsafe {
            w.ch_gamma_duty_inc()
                .variant(duty_inc)
                .ch_gamma_duty_num() // count of incs before stopping
                .bits(duty_steps)
                .ch_gamma_duty_cycle() // overflows between incs
                .bits(cycles_per_step)
                .ch_gamma_scale()
                .bits(duty_per_cycle)
        });
        self.ledc
            .ch_gamma_wr_addr(cnum)
            .write(|w| unsafe { w.ch_gamma_wr_addr().bits(0) });
        self.ledc
            .ch_gamma_conf(cnum)
            .write(|w| unsafe { w.ch_gamma_entry_num().bits(0x1) });
    }

    #[cfg(not(any(esp32, esp32c6, esp32h2)))]
    fn start_duty_fade_inner(
        &self,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        self.ledc
            .ch(self.number as usize)
            .conf1()
            .write(|w| unsafe {
                w.duty_start().set_bit();
                w.duty_inc().variant(duty_inc);
                // count of incs before stopping
                w.duty_num().bits(duty_steps);
                // overflows between incs
                w.duty_cycle().bits(cycles_per_step);
                w.duty_scale().bits(duty_per_cycle)
            });
    }

    #[cfg(esp32)]
    fn update_channel(&self) {
        if !S::IS_HS {
            self.ledc
                .lsch(self.number as usize)
                .conf0()
                .modify(|_, w| w.para_up().set_bit());
        }
    }
    #[cfg(not(esp32))]
    fn update_channel(&self) {
        self.ledc
            .ch(self.number as usize)
            .conf0()
            .modify(|_, w| w.para_up().set_bit());
    }
}

impl<S> ChannelHW for Channel<'_, S>
where
    S: crate::ledc::timer::TimerSpeed,
{
    /// Configure Channel HW
    fn configure_hw(&mut self) -> Result<(), Error> {
        self.configure_hw_with_pin_config(config::PinConfig::PushPull)
    }
    fn configure_hw_with_pin_config(&mut self, cfg: config::PinConfig) -> Result<(), Error> {
        if let Some(timer) = self.timer {
            if !timer.is_configured() {
                return Err(Error::Timer);
            }

            match cfg {
                config::PinConfig::PushPull => self
                    .output_pin
                    .set_to_push_pull_output(crate::private::Internal),
                config::PinConfig::OpenDrain => self
                    .output_pin
                    .set_to_open_drain_output(crate::private::Internal),
            };

            let timer_number = timer.get_number() as u8;

            self.set_channel(timer_number);
            self.update_channel();

            #[cfg(esp32)]
            let signal = if S::IS_HS {
                #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
                match self.number {
                    Number::Channel0 => OutputSignal::LEDC_HS_SIG0,
                    Number::Channel1 => OutputSignal::LEDC_HS_SIG1,
                    Number::Channel2 => OutputSignal::LEDC_HS_SIG2,
                    Number::Channel3 => OutputSignal::LEDC_HS_SIG3,
                    Number::Channel4 => OutputSignal::LEDC_HS_SIG4,
                    Number::Channel5 => OutputSignal::LEDC_HS_SIG5,
                    Number::Channel6 => OutputSignal::LEDC_HS_SIG6,
                    Number::Channel7 => OutputSignal::LEDC_HS_SIG7,
                }
            } else {
                match self.number {
                    Number::Channel0 => OutputSignal::LEDC_LS_SIG0,
                    Number::Channel1 => OutputSignal::LEDC_LS_SIG1,
                    Number::Channel2 => OutputSignal::LEDC_LS_SIG2,
                    Number::Channel3 => OutputSignal::LEDC_LS_SIG3,
                    Number::Channel4 => OutputSignal::LEDC_LS_SIG4,
                    Number::Channel5 => OutputSignal::LEDC_LS_SIG5,
                    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
                    Number::Channel6 => OutputSignal::LEDC_LS_SIG6,
                    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
                    Number::Channel7 => OutputSignal::LEDC_LS_SIG7,
                }
            };
            #[cfg(not(esp32))]
            let signal = match self.number {
                Number::Channel0 => OutputSignal::LEDC_LS_SIG0,
                Number::Channel1 => OutputSignal::LEDC_LS_SIG1,
                Number::Channel2 => OutputSignal::LEDC_LS_SIG2,
                Number::Channel3 => OutputSignal::LEDC_LS_SIG3,
                Number::Channel4 => OutputSignal::LEDC_LS_SIG4,
                Number::Channel5 => OutputSignal::LEDC_LS_SIG5,
                #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
                Number::Channel6 => OutputSignal::LEDC_LS_SIG6,
                #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
                Number::Channel7 => OutputSignal::LEDC_LS_SIG7,
            };

            self.output_pin
                .connect_peripheral_to_output(signal, crate::private::Internal);
        } else {
            return Err(Error::Timer);
        }

        Ok(())
    }

    /// Set duty in channel HW
    #[cfg(esp32)]
    fn set_duty_hw(&self, duty: u32) {
        if S::IS_HS {
            self.ledc
                .hsch(self.number as usize)
                .duty()
                .write(|w| unsafe { w.duty().bits(duty << 4) });
        } else {
            self.ledc
                .lsch(self.number as usize)
                .duty()
                .write(|w| unsafe { w.duty().bits(duty << 4) });
        }
        self.start_duty_without_fading();
        self.update_channel();
    }

    /// Set duty in channel HW
    #[cfg(not(esp32))]
    fn set_duty_hw(&self, duty: u32) {
        self.ledc
            .ch(self.number as usize)
            .duty()
            .write(|w| unsafe { w.duty().bits(duty << 4) });
        self.start_duty_without_fading();
        self.update_channel();
    }

    /// Start a duty-cycle fade HW
    #[cfg(esp32)]
    fn start_duty_fade_hw(
        &self,
        start_duty: u32,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        if S::IS_HS {
            self.ledc
                .hsch(self.number as usize)
                .duty()
                .write(|w| unsafe { w.duty().bits(start_duty << 4) });
            self.ledc
                .int_clr()
                .write(|w| w.duty_chng_end_hsch(self.number as u8).clear_bit_by_one());
        } else {
            self.ledc
                .lsch(self.number as usize)
                .duty()
                .write(|w| unsafe { w.duty().bits(start_duty << 4) });
            self.ledc
                .int_clr()
                .write(|w| w.duty_chng_end_lsch(self.number as u8).clear_bit_by_one());
        }
        self.start_duty_fade_inner(duty_inc, duty_steps, cycles_per_step, duty_per_cycle);
        self.update_channel();
    }

    /// Start a duty-cycle fade HW
    #[cfg(not(esp32))]
    fn start_duty_fade_hw(
        &self,
        start_duty: u32,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        self.ledc
            .ch(self.number as usize)
            .duty()
            .write(|w| unsafe { w.duty().bits(start_duty << 4) });
        self.ledc
            .int_clr()
            .write(|w| w.duty_chng_end_ch(self.number as u8).clear_bit_by_one());
        self.start_duty_fade_inner(duty_inc, duty_steps, cycles_per_step, duty_per_cycle);
        self.update_channel();
    }

    #[cfg(esp32)]
    fn is_duty_fade_running_hw(&self) -> bool {
        let reg = self.ledc.int_raw().read();
        if S::IS_HS {
            reg.duty_chng_end_hsch(self.number as u8).bit_is_clear()
        } else {
            reg.duty_chng_end_lsch(self.number as u8).bit_is_clear()
        }
    }

    #[cfg(not(esp32))]
    fn is_duty_fade_running_hw(&self) -> bool {
        self.ledc
            .int_raw()
            .read()
            .duty_chng_end_ch(self.number as u8)
            .bit_is_clear()
    }
}
