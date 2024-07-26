//! # Capacitive Touch Sensor
//!
//! ## Overview
//!
//! The touch sensor peripheral allows for cheap and robust user interfaces by
//! e.g., dedicating a part of the pcb as touch button.
//!
//! ## Examples
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::touch::{Touch, TouchPad};
//! # use esp_hal::gpio::Io;
//! let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! let touch_pin0 = io.pins.gpio2;
//! let touch = Touch::continous_mode(peripherals.TOUCH, None);
//! let mut touchpad = TouchPad::new(touch_pin0, &touch);
//! // ... give the peripheral some time for the measurement
//! let touch_val = touchpad.read();
//! # }
//! ```
//! 
//! ## Implementation State:
//!
//! Mostly feature complete, missing:
//!
//! - Async support
//! - Touch sensor slope control
//! - Deep Sleep support (wakeup from Deep Sleep)

#![deny(missing_docs)]

use core::marker::PhantomData;

use crate::{
    gpio::TouchPin,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{RTC_CNTL, SENS, TOUCH},
    private::{Internal, Sealed},
};

/// A marker trait describing the mode the touch pad is set to.
pub trait TouchMode: Sealed {}

/// Marker struct for the touch peripherals manual trigger mode. In the
/// technical reference manual, this is referred to as "start FSM via software".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OneShot;

/// Marker struct for the touch peripherals continous reading mode. In the
/// technical reference manual, this is referred to as "start FSM via timer".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Continous;

impl TouchMode for OneShot {}
impl TouchMode for Continous {}
impl Sealed for OneShot {}
impl Sealed for Continous {}

/// Touchpad threshold type.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThresholdMode {
    /// Pad is considered touched if value is greater than threshold.
    GreaterThan,
    /// Pad is considered touched if value is less than threshold.
    LessThan,
}

/// Configurations for the touch pad driver
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchConfig {
    /// The [`ThresholdMode`] for the pads. Defaults to
    /// `ThresholdMode::LessThan`
    pub threshold_mode: Option<ThresholdMode>,
    /// Duration of a single measurement (in cycles of the 8 MHz touch clock).
    /// Defaults to `0x7fff`
    pub measurement_duration: Option<u16>,
    /// Sleep cycles for the touch timer in [`Continous`]-mode. Defaults to
    /// `0x100`
    pub sleep_cycles: Option<u16>,
}

/// This struct marks a successfully initialized touch peripheral
pub struct Touch<'d, MODE: TouchMode> {
    _inner: PeripheralRef<'d, TOUCH>,
    phantom: PhantomData<MODE>,
}
impl<'d, MODE: TouchMode> Touch<'d, MODE> {
    /// Common initialization of the touch peripheral.
    fn initialize_common(config: Option<TouchConfig>) {
        let rtccntl = unsafe { &*RTC_CNTL::ptr() };
        let sens = unsafe { &*SENS::ptr() };

        let mut threshold_mode = false;
        let mut meas_dur = 0x7fff;

        if let Some(config) = config {
            threshold_mode = match config.threshold_mode {
                Some(ThresholdMode::LessThan) => false,
                Some(ThresholdMode::GreaterThan) => true,
                None => false,
            };
            if let Some(dur) = config.measurement_duration {
                meas_dur = dur;
            }
        }

        // stop touch fsm
        rtccntl
            .state0()
            .write(|w| w.touch_slp_timer_en().clear_bit());
        // Disable touch interrupt
        rtccntl.int_ena().write(|w| w.touch().clear_bit());
        // Clear pending interrupts
        rtccntl.int_clr().write(|w| w.touch().bit(true));

        // Disable all interrupts and touch pads
        sens.sar_touch_enable().write(|w| unsafe {
            w.touch_pad_outen1()
                .bits(0b0)
                .touch_pad_outen2()
                .bits(0b0)
                .touch_pad_worken()
                .bits(0b0)
        });

        sens.sar_touch_ctrl1().write(|w| unsafe {
            w
                // Default to trigger when touch is below threshold
                .touch_out_sel()
                .bit(threshold_mode)
                // Interrupt only on set 1
                .touch_out_1en()
                .set_bit()
                .touch_meas_delay()
                .bits(meas_dur)
                // TODO Chip Specific
                .touch_xpd_wait()
                .bits(0xff)
        });
    }
}
impl<'d> Touch<'d, OneShot> {
    /// Initializes the touch peripheral and returns this marker struct.
    /// Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x2000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::one_shot_mode(peripherals.TOUCH, touch_cfg);
    /// # }
    /// ```
    pub fn one_shot_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);
        let rtccntl = unsafe { &*RTC_CNTL::ptr() };
        let sens = unsafe { &*SENS::ptr() };

        // Default nr of sleep cycles from IDF
        let mut sleep_cyc = 0x1000;
        if let Some(config) = config {
            if let Some(slp) = config.sleep_cycles {
                sleep_cyc = slp;
            }
        }

        Self::initialize_common(config);

        sens.sar_touch_ctrl2().write(|w| unsafe {
            w
                // Reset existing touch measurements
                .touch_meas_en_clr()
                .set_bit()
                .touch_sleep_cycles()
                .bits(sleep_cyc)
                // Configure FSM for SW mode
                .touch_start_fsm_en()
                .set_bit()
                .touch_start_en()
                .clear_bit()
                .touch_start_force()
                .set_bit()
        });

        // start touch fsm
        rtccntl.state0().write(|w| w.touch_slp_timer_en().set_bit());

        Self {
            _inner: touch_peripheral,
            phantom: PhantomData,
        }
    }
}
impl<'d> Touch<'d, Continous> {
    /// Initializes the touch peripheral in continous mode and returns this
    /// marker struct. Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x3000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::continous_mode(peripherals.TOUCH, touch_cfg);
    /// # }
    /// ```
    pub fn continous_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);
        let rtccntl = unsafe { &*RTC_CNTL::ptr() };
        let sens = unsafe { &*SENS::ptr() };

        // Default nr of sleep cycles from IDF
        let mut sleep_cyc = 0x1000;
        if let Some(config) = config {
            if let Some(slp) = config.sleep_cycles {
                sleep_cyc = slp;
            }
        }

        Self::initialize_common(config);

        sens.sar_touch_ctrl2().write(|w| unsafe {
            w
                // Reset existing touch measurements
                .touch_meas_en_clr()
                .set_bit()
                .touch_sleep_cycles()
                .bits(sleep_cyc)
                // Configure FSM for timer mode
                .touch_start_fsm_en()
                .set_bit()
                .touch_start_force()
                .clear_bit()
        });

        // start touch fsm
        rtccntl.state0().write(|w| w.touch_slp_timer_en().set_bit());

        Self {
            _inner: touch_peripheral,
            phantom: PhantomData,
        }
    }
}

/// A pin that is configured as a TouchPad.
pub struct TouchPad<P, MODE: TouchMode>
where
    P: TouchPin,
{
    pin: P,
    phantom: PhantomData<MODE>,
}
impl<P: TouchPin> TouchPad<P, OneShot> {
    /// (Re-)Start a touch measurement on the pin. You can get the result by
    /// calling [`read`](Self::read) once it is finished.
    pub fn start_measurement(&mut self) {
        unsafe { &*crate::peripherals::RTC_IO::ptr() }
            .touch_pad2()
            .write(|w| unsafe {
                w.start()
                    .set_bit()
                    .xpd()
                    .set_bit()
                    // clear input_enable
                    .fun_ie()
                    .clear_bit()
                    // Connect pin to analog / RTC module instead of standard GPIO
                    .mux_sel()
                    .set_bit()
                    // Disable pull-up and pull-down resistors on the pin
                    .rue()
                    .clear_bit()
                    .rde()
                    .clear_bit()
                    .tie_opt()
                    .clear_bit()
                    .to_gpio()
                    .set_bit()
                    // Select function "RTC function 1" (GPIO) for analog use
                    .fun_sel()
                    .bits(0b00)
            });

        unsafe { &*crate::peripherals::SENS::PTR }
            .sar_touch_ctrl2()
            .modify(|_, w| w.touch_start_en().clear_bit());
        unsafe { &*crate::peripherals::SENS::PTR }
            .sar_touch_ctrl2()
            .modify(|_, w| w.touch_start_en().set_bit());
    }
}
impl<P: TouchPin, MODE: TouchMode> TouchPad<P, MODE> {
    /// Construct a new instance of [`TouchPad`].
    ///
    /// ## Parameters:
    /// - `pin`: The pin that gets configured as touch pad
    /// - `touch`: The [`Touch`] struct indicating that touch is configured.
    pub fn new(pin: P, _touch: &Touch<MODE>) -> Self {
        // TODO revert this on drop
        pin.set_touch(Internal);

        Self {
            pin,
            phantom: PhantomData,
        }
    }

    /// Read the current touch pad capacitance counter.
    ///
    /// Usually a lower value means higher capacitance, thus indicating touch
    /// event.
    ///
    /// Returns `None` if the value is not yet ready. (Note: Measurement must be
    /// started manually with [`start_measurement`](Self::start_measurement) if
    /// the touch peripheral is in [`OneShot`] mode).
    pub fn read(&mut self) -> Option<u16> {
        if unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_ctrl2()
            .read()
            .touch_meas_done()
            .bit_is_set()
        {
            Some(self.pin.get_touch_measurement(Internal))
        } else {
            None
        }
    }

    /// Enables the touch_pad interrupt.
    ///
    /// The raised interrupt is actually
    /// [`RTC_CORE`](crate::peripherals::Interrupt::RTC_CORE). A handler can
    /// be installed with [`Rtc::set_interrupt_handler()`][1].
    ///
    /// [1]: ../rtc_cntl/struct.Rtc.html#method.set_interrupt_handler
    ///
    /// ## Parameters:
    /// - `threshold`: The threshold above/below which the pin is considered
    ///   touched. Above/below depends on the configuration of `touch` in
    ///   [`new`](Self::new) (defaults to below).
    ///
    /// ## Example
    pub fn enable_interrupt(&mut self, threshold: u16) {
        self.pin.set_threshold(threshold, Internal);

        let rtccntl = unsafe { &*RTC_CNTL::ptr() };
        // enable touch interrupts
        rtccntl.int_ena().write(|w| w.touch().set_bit());

        let sens = unsafe { &*SENS::ptr() };
        sens.sar_touch_enable().modify(|r, w| unsafe {
            w.touch_pad_outen1()
                .bits(r.touch_pad_outen1().bits() | 1 << self.pin.get_touch_nr(Internal))
        });
    }

    /// Disables the touch pad's interrupt.
    ///
    /// If no other touch pad interrupts are active, the touch interrupt is
    /// disabled completely.
    pub fn disable_interrupt(&mut self) {
        let sens = unsafe { &*SENS::ptr() };
        sens.sar_touch_enable().modify(|r, w| unsafe {
            w.touch_pad_outen1()
                .bits(r.touch_pad_outen1().bits() & !(1 << self.pin.get_touch_nr(Internal)))
        });
        if sens.sar_touch_enable().read().touch_pad_outen1().bits() == 0 {
            let rtccntl = unsafe { &*RTC_CNTL::ptr() };
            rtccntl.int_ena().write(|w| w.touch().clear_bit());
        }
    }

    /// Clears a pending touch interrupt.
    ///
    /// ## Note on interrupt clearing behaviour:
    ///
    /// There is only a single interrupt for the touch pad.
    /// [`is_interrupt_set`](Self::is_interrupt_set) can be used to check
    /// which pins are touchted. However, this function clears the interrupt
    /// status for all pins. So only call it when all pins are handled.
    pub fn clear_interrupt(&mut self) {
        let rtccntl = unsafe { &*RTC_CNTL::ptr() };
        rtccntl.int_clr().write(|w| w.touch().clear_bit_by_one());
        let sens = unsafe { &*SENS::ptr() };
        sens.sar_touch_ctrl2()
            .write(|w| w.touch_meas_en_clr().set_bit());
    }

    /// Checks if the pad is touched, based on the configured threshold value.
    pub fn is_interrupt_set(&mut self) -> bool {
        let sens = unsafe { &*SENS::ptr() };
        let ctrl2 = sens.sar_touch_ctrl2().read();
        // Only god knows, why the "interrupt flag" register is called "meas_en" on this
        // chip...
        ctrl2.touch_meas_en().bits() & (1 << self.pin.get_touch_nr(Internal)) != 0
    }
}
