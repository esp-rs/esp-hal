//! # LEDC channel
//!
//! ## Overview
//! The `LEDC Channel` module is a part of the `LED Controller (LEDC)` driver
//! designed for ESP microcontrollers. It provides a high-level interface to
//! configure and control individual PWM channels of the LEDC peripheral.
//!
//! The module allows precise and flexible control over LED lighting and other
//! `Pulse-Width Modulation (PWM)` applications by offering configurable duty
//! cycles and frequencies.

use paste::paste;

#[cfg(esp32)]
use super::HighSpeed;
use super::{
    timer::{TimerIFace, TimerSpeed},
    LowSpeed,
};
use crate::{
    gpio::{OutputPin, OutputSignal},
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
    Channel0,
    Channel1,
    Channel2,
    Channel3,
    Channel4,
    Channel5,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    Channel6,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
    Channel7,
}

/// Channel configuration
pub mod config {
    use crate::ledc::timer::{TimerIFace, TimerSpeed};

    #[derive(Debug, Clone, Copy, PartialEq)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub enum PinConfig {
        PushPull,
        OpenDrain,
    }

    /// Channel configuration
    #[derive(Copy, Clone)]
    pub struct Config<'a, S: TimerSpeed> {
        pub timer: &'a dyn TimerIFace<S>,
        pub duty_pct: u8,
        pub pin_config: PinConfig,
    }
}

/// Channel interface
pub trait ChannelIFace<'a, S: TimerSpeed + 'a, O: OutputPin + 'a>
where
    Channel<'a, S, O>: ChannelHW<O>,
{
    /// Configure channel
    fn configure(&mut self, config: config::Config<'a, S>) -> Result<(), Error>;

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
pub trait ChannelHW<O: OutputPin> {
    /// Configure Channel HW except for the duty which is set via
    /// [`Self::set_duty_hw`].
    fn configure_hw(&mut self) -> Result<(), Error>;
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
pub struct Channel<'a, S: TimerSpeed, O: OutputPin> {
    ledc: &'a RegisterBlock,
    timer: Option<&'a dyn TimerIFace<S>>,
    number: Number,
    output_pin: PeripheralRef<'a, O>,
}

impl<'a, S: TimerSpeed, O: OutputPin> Channel<'a, S, O> {
    /// Return a new channel
    pub fn new(number: Number, output_pin: impl Peripheral<P = O> + 'a) -> Self {
        crate::into_ref!(output_pin);
        let ledc = unsafe { &*crate::peripherals::LEDC::ptr() };
        Channel {
            ledc,
            timer: None,
            number,
            output_pin,
        }
    }
}

impl<'a, S: TimerSpeed, O: OutputPin> ChannelIFace<'a, S, O> for Channel<'a, S, O>
where
    Channel<'a, S, O>: ChannelHW<O>,
{
    /// Configure channel
    fn configure(&mut self, config: config::Config<'a, S>) -> Result<(), Error> {
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
        let duty_value = (duty_range * duty_pct as u32) as u32 / 100;

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
        let start_duty_value = (duty_range * start_duty_pct as u32) as u32 / 100;
        let end_duty_value = (duty_range * end_duty_pct as u32) as u32 / 100;

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

#[cfg(esp32)]
/// Macro to configure channel parameters in hw
macro_rules! set_channel {
    ($self: ident, $speed: ident, $num: literal, $timer_number: ident) => {{
        paste! {
            $self.ledc.[<$speed sch $num _hpoint>]
                .write(|w| unsafe { w.[<hpoint>]().bits(0x0) });
            $self.ledc.[<$speed sch $num _conf0>].modify(|_, w| unsafe {
                w.[<sig_out_en>]()
                    .set_bit()
                    .[<timer_sel>]()
                    .bits($timer_number)
            });
        }
        start_duty_without_fading!($self, $speed, $num);
    }};
}

#[cfg(not(esp32))]
/// Macro to configure channel parameters in hw
macro_rules! set_channel {
    ($self: ident, $speed: ident, $num: literal, $timer_number: ident) => {{
        paste! {
            $self.ledc.[<ch $num _hpoint>]()
                .write(|w| unsafe { w.[<hpoint>]().bits(0x0) });
            $self.ledc.[<ch $num _conf0>]().modify(|_, w| unsafe {
                w.[<sig_out_en>]()
                    .set_bit()
                    .[<timer_sel>]()
                    .bits($timer_number)
            });
        }
        start_duty_without_fading!($self, $num);
    }};
}

#[cfg(esp32)]
/// Macro to start duty cycle, without fading
macro_rules! start_duty_without_fading {
    ($self: ident, $speed: ident, $num: literal) => {
        paste! {
            $self.ledc.[<$speed sch $num _conf1>].write(|w| unsafe {
                w.[<duty_start>]()
                    .set_bit()
                    .[<duty_inc>]()
                    .set_bit()
                    .[<duty_num>]()
                    .bits(0x1)
                    .[<duty_cycle>]()
                    .bits(0x1)
                    .[<duty_scale>]()
                    .bits(0x0)
                });
        }
    };
}

#[cfg(any(esp32c6, esp32h2))]
/// Macro to start duty cycle, without fading
macro_rules! start_duty_without_fading {
    ($self: ident, $num: literal) => {
        paste! {
            $self.ledc.[<ch $num _conf1>]().write(|w|
                w.[<duty_start>]()
                    .set_bit()
            );
            $self.ledc.[<ch $num _gamma_wr>]().write(|w| unsafe {
                w.[<ch_gamma_duty_inc>]()
                    .set_bit()
                    .[<ch_gamma_duty_num>]()
                    .bits(0x1)
                    .[<ch_gamma_duty_cycle>]()
                    .bits(0x1)
                    .[<ch_gamma_scale>]()
                    .bits(0x0)
                });
        }
    };
}

#[cfg(not(any(esp32, esp32c6, esp32h2)))]
/// Macro to start duty cycle, without fading
macro_rules! start_duty_without_fading {
    ($self: ident, $num: literal) => {
        paste! {
            $self.ledc.[<ch $num _conf1>]().write(|w| unsafe {
                w.[<duty_start>]()
                    .set_bit()
                    .[<duty_inc>]()
                    .set_bit()
                    .[<duty_num>]()
                    .bits(0x1)
                    .[<duty_cycle>]()
                    .bits(0x1)
                    .[<duty_scale>]()
                    .bits(0x0)
                });
        }
    };
}

#[cfg(esp32)]
/// Macro to start duty cycle fade
macro_rules! start_duty_fade {
    ($self: ident, $speed: ident, $num: literal, $duty_inc: ident, $duty_steps: ident, $cycles_per_step: ident, $duty_per_cycle: ident) => {
        paste! {
            $self.ledc.[<$speed sch $num _conf1>].write(|w| unsafe {
                w.[<duty_start>]()
                    .set_bit()
                    .[<duty_inc>]()
                    .variant($duty_inc)
                    .[<duty_num>]()  /* count of incs before stopping */
                    .bits($duty_steps)
                    .[<duty_cycle>]()  /* overflows between incs */
                    .bits($cycles_per_step)
                    .[<duty_scale>]()
                    .bits($duty_per_cycle)
                });
        }
    };
}

#[cfg(any(esp32c6, esp32h2))]
/// Macro to start a duty cycle fade
macro_rules! start_duty_fade {
    ($self: ident, $num: literal, $duty_inc: ident, $duty_steps: ident, $cycles_per_step: ident, $duty_per_cycle: ident) => {
        paste! {
            $self.ledc.[<ch $num _conf1>]().write(|w|
                w.[<duty_start>]()
                    .set_bit()
            );
            $self.ledc.[<ch $num _gamma_wr>]().write(|w| unsafe {
                w.[<ch_gamma_duty_inc>]()
                    .variant($duty_inc)
                    .[<ch_gamma_duty_num>]()  /* count of incs before stopping */
                    .bits($duty_steps)
                    .[<ch_gamma_duty_cycle>]()  /* overflows between incs */
                    .bits($cycles_per_step)
                    .[<ch_gamma_scale>]()
                    .bits($duty_per_cycle)
                });
            $self.ledc.[<ch $num _gamma_wr_addr>]().write(|w| unsafe {
                w.[<ch_gamma_wr_addr>]()
                    .bits(0)
            });
            $self.ledc.[<ch_gamma_conf>]($num).write(|w| unsafe {
                w.[<ch_gamma_entry_num>]()
                    .bits(0x1)
            });
        }
    };
}

#[cfg(not(any(esp32, esp32c6, esp32h2)))]
/// Macro to start a duty cycle fade
macro_rules! start_duty_fade {
    ($self: ident, $num: literal, $duty_inc: ident, $duty_steps: ident, $cycles_per_step: ident, $duty_per_cycle: ident) => {
        paste! {{
            $self.ledc.[<ch $num _conf1>]().write(|w| unsafe {
                w.[<duty_start>]()
                    .set_bit()
                    .[<duty_inc>]()
                    .variant($duty_inc)
                    .[<duty_num>]()  /* count of incs before stopping */
                    .bits($duty_steps)
                    .[<duty_cycle>]()  /* overflows between incs */
                    .bits($cycles_per_step)
                    .[<duty_scale>]()
                    .bits($duty_per_cycle)
                });
        }}
    };
}

#[cfg(esp32)]
/// Macro to set duty parameters in hw
macro_rules! set_duty {
    ($self: ident, $speed: ident, $num: literal, $duty: ident) => {{
        paste! {
            $self.ledc
                .[<$speed sch $num _duty>]
                .write(|w| unsafe { w.[<duty>]().bits($duty << 4) });
        }
        start_duty_without_fading!($self, $speed, $num);
        update_channel!($self, $speed, $num);
    }};
}

#[cfg(not(esp32))]
/// Macro to set duty parameters in hw
macro_rules! set_duty {
    ($self: ident, $speed: ident, $num: literal, $duty: ident) => {{
        paste! {
            $self.ledc
                .[<ch $num _duty>]()
                .write(|w| unsafe { w.[<duty>]().bits($duty << 4) });
        }
        start_duty_without_fading!($self, $num);
        update_channel!($self, $speed, $num);
    }};
}

#[cfg(esp32)]
/// Macro to set duty parameters in hw for a fade
macro_rules! set_duty_fade {
    ($self: ident, $speed: ident, $num: literal, $start_duty: ident, $duty_inc: ident, $duty_steps: ident, $cycles_per_step: ident, $duty_per_cycle: ident) => {{
        paste! {
            $self.ledc
                .[<$speed sch $num _duty>]
                .write(|w| unsafe { w.[<duty>]().bits($start_duty << 4) });
            $self.ledc
                .[<int_clr>]
                .write(|w| { w.[<duty_chng_end_ $speed sch $num _int_clr>]().set_bit() });
        }
        start_duty_fade!(
            $self,
            $speed,
            $num,
            $duty_inc,
            $duty_steps,
            $cycles_per_step,
            $duty_per_cycle
        );
        update_channel!($self, $speed, $num);
    }};
}

#[cfg(esp32c3)]
/// Macro to set duty parameters in hw for a fade
macro_rules! set_duty_fade {
    ($self: ident, $speed: ident, $num: literal, $start_duty: ident, $duty_inc: ident, $duty_steps: ident, $cycles_per_step: ident, $duty_per_cycle: ident) => {{
        paste! {
            $self.ledc
                .[<ch $num _duty>]()
                .write(|w| unsafe { w.[<duty>]().bits($start_duty << 4) });
            $self.ledc
                .[<int_clr>]()
                .write(|w| { w.[<duty_chng_end_ $speed sch $num _int_clr>]().set_bit() });
        }
        start_duty_fade!(
            $self,
            $num,
            $duty_inc,
            $duty_steps,
            $cycles_per_step,
            $duty_per_cycle
        );
        update_channel!($self, $speed, $num);
    }};
}

#[cfg(not(any(esp32, esp32c3)))]
/// Macro to set duty parameters in hw for a fade
macro_rules! set_duty_fade {
    ($self: ident, $speed: ident, $num: literal, $start_duty: ident, $duty_inc: ident, $duty_steps: ident, $cycles_per_step: ident, $duty_per_cycle: ident) => {{
        paste! {
            $self.ledc
                .[<ch $num _duty>]()
                .write(|w| unsafe { w.[<duty>]().bits($start_duty << 4) });
            $self.ledc
                .[<int_clr>]()
                .write(|w| { w.[<duty_chng_end_ch $num _int_clr>]().set_bit() });
        }
        start_duty_fade!(
            $self,
            $num,
            $duty_inc,
            $duty_steps,
            $cycles_per_step,
            $duty_per_cycle
        );
        update_channel!($self, $speed, $num);
    }};
}

/// Macro to check if a duty-cycle fade is running
#[cfg(any(esp32, esp32c3))]
macro_rules! is_duty_fade_running {
    ($self: ident, $speed: ident, $num: literal) => {{
        paste! {
            $self.ledc
                .[<int_raw>]()
                .read()
                .[<duty_chng_end_ $speed sch $num _int_raw>]()
                .bit_is_clear()
        }
    }};
}

#[cfg(not(any(esp32, esp32c3)))]
macro_rules! is_duty_fade_running {
    ($self: ident, $speed: ident, $num: literal) => {{
        paste! {
            $self.ledc
                .[<int_raw>]()
                .read()
                .[<duty_chng_end_ch $num _int_raw>]()
                .bit_is_clear()
        }
    }};
}

#[cfg(esp32)]
/// Macro to update channel configuration (only for LowSpeed channels)
macro_rules! update_channel {
    ($self: ident, l, $num: literal) => {
        paste! {
            $self.ledc
                .[<lsch $num _conf0>]
                .modify(|_, w| w.[<para_up>]().set_bit());
        }
    };
    ($self: ident, h, $num: literal) => {};
}

#[cfg(not(esp32))]
/// Macro to update channel configuration (only for LowSpeed channels)
macro_rules! update_channel {
    ($self: ident, l, $num: literal) => {
        paste! {
            $self.ledc
                .[<ch $num _conf0>]()
                .modify(|_, w| w.[<para_up>]().set_bit());
        }
    };
}

#[cfg(esp32)]
/// Channel HW interface for HighSpeed channels
impl<'a, O> ChannelHW<O> for Channel<'a, HighSpeed, O>
where
    O: OutputPin,
{
    /// Configure Channel HW except for the duty which is set via
    /// [`Self::set_duty_hw`].
    fn configure_hw(&mut self) -> Result<(), Error> {
        self.configure_hw_with_pin_config(config::PinConfig::PushPull)
    }
    fn configure_hw_with_pin_config(&mut self, cfg: config::PinConfig) -> Result<(), Error> {
        if let Some(timer) = self.timer {
            if !timer.is_configured() {
                return Err(Error::Timer);
            }

            match cfg {
                config::PinConfig::PushPull => self.output_pin.set_to_push_pull_output(),
                config::PinConfig::OpenDrain => self.output_pin.set_to_open_drain_output(),
            };

            let timer_number = timer.get_number() as u8;
            match self.number {
                Number::Channel0 => {
                    set_channel!(self, h, 0, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG0);
                }
                Number::Channel1 => {
                    set_channel!(self, h, 1, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG1);
                }
                Number::Channel2 => {
                    set_channel!(self, h, 2, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG2);
                }
                Number::Channel3 => {
                    set_channel!(self, h, 3, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG3);
                }
                Number::Channel4 => {
                    set_channel!(self, h, 4, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG4);
                }
                Number::Channel5 => {
                    set_channel!(self, h, 5, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG5);
                }
                Number::Channel6 => {
                    set_channel!(self, h, 6, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG6);
                }
                Number::Channel7 => {
                    set_channel!(self, h, 7, timer_number);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_HS_SIG7);
                }
            }
        } else {
            return Err(Error::Timer);
        }

        Ok(())
    }

    /// Set duty in channel HW
    fn set_duty_hw(&self, duty: u32) {
        match self.number {
            Number::Channel0 => set_duty!(self, h, 0, duty),
            Number::Channel1 => set_duty!(self, h, 1, duty),
            Number::Channel2 => set_duty!(self, h, 2, duty),
            Number::Channel3 => set_duty!(self, h, 3, duty),
            Number::Channel4 => set_duty!(self, h, 4, duty),
            Number::Channel5 => set_duty!(self, h, 5, duty),
            Number::Channel6 => set_duty!(self, h, 6, duty),
            Number::Channel7 => set_duty!(self, h, 7, duty),
        };
    }

    /// Start a duty-cycle fade HW
    fn start_duty_fade_hw(
        &self,
        start_duty: u32,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        match self.number {
            Number::Channel0 => set_duty_fade!(
                self,
                h,
                0,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel1 => set_duty_fade!(
                self,
                h,
                1,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel2 => set_duty_fade!(
                self,
                h,
                2,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel3 => set_duty_fade!(
                self,
                h,
                3,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel4 => set_duty_fade!(
                self,
                h,
                4,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel5 => set_duty_fade!(
                self,
                h,
                5,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel6 => set_duty_fade!(
                self,
                h,
                6,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel7 => set_duty_fade!(
                self,
                h,
                7,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
        }
    }

    fn is_duty_fade_running_hw(&self) -> bool {
        match self.number {
            Number::Channel0 => is_duty_fade_running!(self, h, 0),
            Number::Channel1 => is_duty_fade_running!(self, h, 1),
            Number::Channel2 => is_duty_fade_running!(self, h, 2),
            Number::Channel3 => is_duty_fade_running!(self, h, 3),
            Number::Channel4 => is_duty_fade_running!(self, h, 4),
            Number::Channel5 => is_duty_fade_running!(self, h, 5),
            Number::Channel6 => is_duty_fade_running!(self, h, 6),
            Number::Channel7 => is_duty_fade_running!(self, h, 7),
        }
    }
}

/// Channel HW interface for LowSpeed channels
impl<'a, O: OutputPin> ChannelHW<O> for Channel<'a, LowSpeed, O>
where
    O: OutputPin,
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
                config::PinConfig::PushPull => {
                    self.output_pin.set_to_push_pull_output();
                }
                config::PinConfig::OpenDrain => {
                    self.output_pin.set_to_open_drain_output();
                }
            }

            let timer_number = timer.get_number() as u8;
            match self.number {
                Number::Channel0 => {
                    set_channel!(self, l, 0, timer_number);
                    update_channel!(self, l, 0);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG0);
                }
                Number::Channel1 => {
                    set_channel!(self, l, 1, timer_number);
                    update_channel!(self, l, 1);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG1);
                }
                Number::Channel2 => {
                    set_channel!(self, l, 2, timer_number);
                    update_channel!(self, l, 2);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG2);
                }
                Number::Channel3 => {
                    set_channel!(self, l, 3, timer_number);
                    update_channel!(self, l, 3);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG3);
                }
                Number::Channel4 => {
                    set_channel!(self, l, 4, timer_number);
                    update_channel!(self, l, 4);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG4);
                }
                Number::Channel5 => {
                    set_channel!(self, l, 5, timer_number);
                    update_channel!(self, l, 5);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG5);
                }
                #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
                Number::Channel6 => {
                    set_channel!(self, l, 6, timer_number);
                    update_channel!(self, l, 6);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG6);
                }
                #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
                Number::Channel7 => {
                    set_channel!(self, l, 7, timer_number);
                    update_channel!(self, l, 7);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG7);
                }
            }
        } else {
            return Err(Error::Timer);
        }

        Ok(())
    }

    /// Set duty in channel HW
    fn set_duty_hw(&self, duty: u32) {
        match self.number {
            Number::Channel0 => set_duty!(self, l, 0, duty),
            Number::Channel1 => set_duty!(self, l, 1, duty),
            Number::Channel2 => set_duty!(self, l, 2, duty),
            Number::Channel3 => set_duty!(self, l, 3, duty),
            Number::Channel4 => set_duty!(self, l, 4, duty),
            Number::Channel5 => set_duty!(self, l, 5, duty),
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            Number::Channel6 => set_duty!(self, l, 6, duty),
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            Number::Channel7 => set_duty!(self, l, 7, duty),
        };
    }

    /// Start a duty-cycle fade HW
    fn start_duty_fade_hw(
        &self,
        start_duty: u32,
        duty_inc: bool,
        duty_steps: u16,
        cycles_per_step: u16,
        duty_per_cycle: u16,
    ) {
        match self.number {
            Number::Channel0 => set_duty_fade!(
                self,
                l,
                0,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel1 => set_duty_fade!(
                self,
                l,
                1,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel2 => set_duty_fade!(
                self,
                l,
                2,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel3 => set_duty_fade!(
                self,
                l,
                3,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel4 => set_duty_fade!(
                self,
                l,
                4,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            Number::Channel5 => set_duty_fade!(
                self,
                l,
                5,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            Number::Channel6 => set_duty_fade!(
                self,
                l,
                6,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            Number::Channel7 => set_duty_fade!(
                self,
                l,
                7,
                start_duty,
                duty_inc,
                duty_steps,
                cycles_per_step,
                duty_per_cycle
            ),
        }
    }

    fn is_duty_fade_running_hw(&self) -> bool {
        match self.number {
            Number::Channel0 => is_duty_fade_running!(self, l, 0),
            Number::Channel1 => is_duty_fade_running!(self, l, 1),
            Number::Channel2 => is_duty_fade_running!(self, l, 2),
            Number::Channel3 => is_duty_fade_running!(self, l, 3),
            Number::Channel4 => is_duty_fade_running!(self, l, 4),
            Number::Channel5 => is_duty_fade_running!(self, l, 5),
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            Number::Channel6 => is_duty_fade_running!(self, l, 6),
            #[cfg(not(any(esp32c2, esp32c3, esp32c6, esp32h2)))]
            Number::Channel7 => is_duty_fade_running!(self, l, 7),
        }
    }
}
