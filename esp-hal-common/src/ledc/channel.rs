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

/// Channel errors
#[derive(Debug)]
pub enum Error {
    /// Invalid duty % value
    Duty,
    /// Timer not configured
    Timer,
    /// Channel not configured
    Channel,
}

/// Channel number
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum Number {
    Channel0,
    Channel1,
    Channel2,
    Channel3,
    Channel4,
    Channel5,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
    Channel6,
    #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
    Channel7,
}

/// Channel configuration
pub mod config {
    use crate::ledc::timer::{TimerIFace, TimerSpeed};

    /// Channel configuration
    #[derive(Copy, Clone)]
    pub struct Config<'a, S: TimerSpeed> {
        pub timer: &'a dyn TimerIFace<S>,
        pub duty_pct: u8,
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
}

/// Channel HW interface
pub trait ChannelHW<O: OutputPin> {
    /// Configure Channel HW except for the duty which is set via
    /// [`Self::set_duty_hw`].
    fn configure_hw(&mut self) -> Result<(), Error>;

    /// Set channel duty HW
    fn set_duty_hw(&self, duty: u32);
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
        self.configure_hw()?;

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
            $self.ledc.[<ch $num _hpoint>]
                .write(|w| unsafe { w.[<hpoint>]().bits(0x0) });
            $self.ledc.[<ch $num _conf0>].modify(|_, w| unsafe {
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

#[cfg(esp32c6)]
/// Macro to start duty cycle, without fading
macro_rules! start_duty_without_fading {
    ($self: ident, $num: literal) => {
        paste! {
            $self.ledc.[<ch $num _conf1>].write(|w|
                w.[<duty_start>]()
                    .set_bit()
            );
            $self.ledc.[<ch $num _gamma_wr>].write(|w| unsafe {
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

#[cfg(not(any(esp32, esp32c6)))]
/// Macro to start duty cycle, without fading
macro_rules! start_duty_without_fading {
    ($self: ident, $num: literal) => {
        paste! {
            $self.ledc.[<ch $num _conf1>].write(|w| unsafe {
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
                .[<ch $num _duty>]
                .write(|w| unsafe { w.[<duty>]().bits($duty << 4) });
        }
        start_duty_without_fading!($self, $num);
        update_channel!($self, $speed, $num);
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
                .[<ch $num _conf0>]
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
        if let Some(timer) = self.timer {
            if !timer.is_configured() {
                return Err(Error::Timer);
            }

            self.output_pin.set_to_push_pull_output();

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
}

/// Channel HW interface for LowSpeed channels
impl<'a, O: OutputPin> ChannelHW<O> for Channel<'a, LowSpeed, O>
where
    O: OutputPin,
{
    /// Configure Channel HW
    fn configure_hw(&mut self) -> Result<(), Error> {
        if let Some(timer) = self.timer {
            if !timer.is_configured() {
                return Err(Error::Timer);
            }

            self.output_pin.set_to_push_pull_output();

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
                #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
                Number::Channel6 => {
                    set_channel!(self, l, 6, timer_number);
                    update_channel!(self, l, 6);
                    self.output_pin
                        .connect_peripheral_to_output(OutputSignal::LEDC_LS_SIG6);
                }
                #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
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
            #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
            Number::Channel6 => set_duty!(self, l, 6, duty),
            #[cfg(not(any(esp32c2, esp32c3, esp32c6)))]
            Number::Channel7 => set_duty!(self, l, 7, duty),
        };
    }
}
