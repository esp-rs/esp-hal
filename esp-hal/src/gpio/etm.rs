//! # Event Task Matrix (ETM)
//!
//! ## Overview
//! GPIO supports ETM function, that is, the ETM task of GPIO can be
//! triggered by the ETM event of any peripheral, or the ETM task of any
//! peripheral can be triggered by the ETM event of GPIO.
//!
//! ## Configuration
//! The GPIO ETM provides several task channels. The ETM tasks that each task
//! channel can receive are:
//! - SET: GPIO goes high when triggered
//! - CLEAR: GPIO goes low when triggered
//! - TOGGLE: GPIO toggle level when triggered.
//!
//! GPIO has several event channels, and the ETM events that each event
//! channel can generate are:
//! - RISE_EDGE: Indicates that the output signal of the corresponding GPIO has
//!   a rising edge
//! - FALL_EDGE: Indicates that the output signal of the corresponding GPIO has
//!   a falling edge
//! - ANY_EDGE: Indicates that the output signal of the corresponding GPIO is
//!   reversed
//!
//! ## Examples
//! ### Toggle an LED When a Button is Pressed
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::gpio::Io;
//! # use esp_hal::gpio::etm::Channels;
//! # use esp_hal::etm::Etm;
//! # use esp_hal::gpio::etm::InputConfig;
//! # use esp_hal::gpio::etm::OutputConfig;
//! # use esp_hal::gpio::Pull;
//! # use esp_hal::gpio::Level;
//! #
//! # let io = Io::new(peripherals.IO_MUX);
//! # let mut led = peripherals.pins.gpio1;
//! # let button = peripherals.pins.gpio9;
//!
//! let gpio_ext = Channels::new(peripherals.GPIO_SD);
//! let led_task = gpio_ext.channel0_task.toggle(
//!     &mut led,
//!     OutputConfig {
//!         open_drain: false,
//!         pull: Pull::None,
//!         initial_state: Level::Low,
//!     },
//! );
//! let button_event = gpio_ext
//!     .channel0_event
//!     .falling_edge(button, InputConfig { pull: Pull::Down });
//! # }
//! ```

use core::marker::PhantomData;

use crate::{
    gpio::{
        interconnect::{InputSignal, OutputSignal},
        Level,
        Pull,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::GPIO_SD,
    private,
};

/// All the GPIO ETM channels
#[non_exhaustive]
pub struct Channels<'d> {
    _gpio_sd: PeripheralRef<'d, GPIO_SD>,
    /// Task channel 0 for triggering GPIO tasks.
    pub channel0_task: TaskChannel<0>,
    /// Event channel 0 for handling GPIO events.
    pub channel0_event: EventChannel<0>,
    /// Task channel 1 for triggering GPIO tasks.
    pub channel1_task: TaskChannel<1>,
    /// Event channel 1 for handling GPIO events.
    pub channel1_event: EventChannel<1>,
    /// Task channel 2 for triggering GPIO tasks.
    pub channel2_task: TaskChannel<2>,
    /// Event channel 2 for handling GPIO events.
    pub channel2_event: EventChannel<2>,
    /// Task channel 3 for triggering GPIO tasks.
    pub channel3_task: TaskChannel<3>,
    /// Event channel 3 for handling GPIO events.
    pub channel3_event: EventChannel<3>,
    /// Task channel 4 for triggering GPIO tasks.
    pub channel4_task: TaskChannel<4>,
    /// Event channel 4 for handling GPIO events.
    pub channel4_event: EventChannel<4>,
    /// Task channel 5 for triggering GPIO tasks.
    pub channel5_task: TaskChannel<5>,
    /// Event channel 5 for handling GPIO events.
    pub channel5_event: EventChannel<5>,
    /// Task channel 6 for triggering GPIO tasks.
    pub channel6_task: TaskChannel<6>,
    /// Event channel 6 for handling GPIO events.
    pub channel6_event: EventChannel<6>,
    /// Task channel 7 for triggering GPIO tasks.
    pub channel7_task: TaskChannel<7>,
    /// Event channel 7 for handling GPIO events.
    pub channel7_event: EventChannel<7>,
}

impl<'d> Channels<'d> {
    /// Create a new instance
    pub fn new(peripheral: impl Peripheral<P = GPIO_SD> + 'd) -> Self {
        crate::into_ref!(peripheral);

        Self {
            _gpio_sd: peripheral,
            channel0_task: TaskChannel {},
            channel0_event: EventChannel {},
            channel1_task: TaskChannel {},
            channel1_event: EventChannel {},
            channel2_task: TaskChannel {},
            channel2_event: EventChannel {},
            channel3_task: TaskChannel {},
            channel3_event: EventChannel {},
            channel4_task: TaskChannel {},
            channel4_event: EventChannel {},
            channel5_task: TaskChannel {},
            channel5_event: EventChannel {},
            channel6_task: TaskChannel {},
            channel6_event: EventChannel {},
            channel7_task: TaskChannel {},
            channel7_event: EventChannel {},
        }
    }
}

/// Configuration for an ETM controlled GPIO input pin
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InputConfig {
    /// Configuration for the internal pull-up resistors
    pub pull: Pull,
}

impl Default for InputConfig {
    fn default() -> Self {
        Self { pull: Pull::None }
    }
}

/// An ETM controlled GPIO event
pub struct EventChannel<const C: u8> {}

impl<const C: u8> EventChannel<C> {
    /// Trigger at rising edge
    pub fn rising_edge<'d>(
        self,
        pin: impl Peripheral<P = impl Into<InputSignal>> + 'd,
        pin_config: InputConfig,
    ) -> Event<'d> {
        self.into_event(pin, pin_config, EventKind::Rising)
    }

    /// Trigger at falling edge
    pub fn falling_edge<'d>(
        self,
        pin: impl Peripheral<P = impl Into<InputSignal>> + 'd,
        pin_config: InputConfig,
    ) -> Event<'d> {
        self.into_event(pin, pin_config, EventKind::Falling)
    }

    /// Trigger at any edge
    pub fn any_edge<'d>(
        self,
        pin: impl Peripheral<P = impl Into<InputSignal>> + 'd,
        pin_config: InputConfig,
    ) -> Event<'d> {
        self.into_event(pin, pin_config, EventKind::Any)
    }

    fn into_event<'d>(
        self,
        pin: impl Peripheral<P = impl Into<InputSignal>> + 'd,
        pin_config: InputConfig,
        kind: EventKind,
    ) -> Event<'d> {
        crate::into_mapped_ref!(pin);

        pin.init_input(pin_config.pull, private::Internal);

        enable_event_channel(C, pin.number());
        Event {
            id: kind.id() + C,
            _pin: PhantomData,
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum EventKind {
    Rising,
    Falling,
    Any,
}

impl EventKind {
    fn id(&self) -> u8 {
        match self {
            EventKind::Rising => 1,
            EventKind::Falling => 9,
            EventKind::Any => 17,
        }
    }
}

/// Event for rising edge
pub struct Event<'d> {
    _pin: PhantomData<&'d mut ()>,
    id: u8,
}

impl private::Sealed for Event<'_> {}

impl crate::etm::EtmEvent for Event<'_> {
    fn id(&self) -> u8 {
        self.id
    }
}

/// Configuration for an ETM controlled GPIO output pin
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OutputConfig {
    /// Set to open-drain output
    pub open_drain: bool,
    /// Only used when open-drain
    pub pull: Pull,
    /// Initial pin state
    pub initial_state: Level,
}

impl Default for OutputConfig {
    fn default() -> Self {
        Self {
            open_drain: false,
            pull: Pull::None,
            initial_state: Level::Low,
        }
    }
}

/// An ETM controlled GPIO task
pub struct TaskChannel<const C: u8> {}

impl<const C: u8> TaskChannel<C> {
    // In theory we could have multiple pins assigned to the same task. Not sure how
    // useful that would be. If we want to support it, the easiest would be
    // to offer additional functions like `set2`, `set3` etc. where the
    // number is the pin-count

    /// Task to set a high level
    pub fn set<'d>(
        self,
        pin: impl Peripheral<P = impl Into<OutputSignal>> + 'd,
        pin_config: OutputConfig,
    ) -> Task<'d> {
        self.into_task(pin, pin_config, TaskKind::Set)
    }

    /// Task to set a low level
    pub fn clear<'d>(
        self,
        pin: impl Peripheral<P = impl Into<OutputSignal>> + 'd,
        pin_config: OutputConfig,
    ) -> Task<'d> {
        self.into_task(pin, pin_config, TaskKind::Clear)
    }

    /// Task to toggle the level
    pub fn toggle<'d>(
        self,
        pin: impl Peripheral<P = impl Into<OutputSignal>> + 'd,
        pin_config: OutputConfig,
    ) -> Task<'d> {
        self.into_task(pin, pin_config, TaskKind::Toggle)
    }

    fn into_task<'d>(
        self,
        pin: impl Peripheral<P = impl Into<OutputSignal>> + 'd,
        pin_config: OutputConfig,
        kind: TaskKind,
    ) -> Task<'d> {
        crate::into_mapped_ref!(pin);

        pin.set_output_high(pin_config.initial_state.into(), private::Internal);
        if pin_config.open_drain {
            pin.pull_direction(pin_config.pull, private::Internal);
            pin.set_to_open_drain_output(private::Internal);
        } else {
            pin.set_to_push_pull_output(private::Internal);
        }

        enable_task_channel(C, pin.number());
        Task {
            id: kind.id() + C,
            _pin: PhantomData,
        }
    }
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum TaskKind {
    Set,
    Clear,
    Toggle,
}

impl TaskKind {
    fn id(&self) -> u8 {
        match self {
            TaskKind::Set => 1,
            TaskKind::Clear => 9,
            TaskKind::Toggle => 17,
        }
    }
}

/// Task for set operation
pub struct Task<'d> {
    _pin: PhantomData<&'d mut ()>,
    id: u8,
}

impl private::Sealed for Task<'_> {}

impl crate::etm::EtmTask for Task<'_> {
    fn id(&self) -> u8 {
        self.id
    }
}

fn enable_task_channel(channel: u8, pin: u8) {
    let gpio_sd = unsafe { GPIO_SD::steal() };
    let ptr = unsafe { gpio_sd.etm_task_p0_cfg().as_ptr().add(pin as usize / 4) };
    let shift = 8 * (pin as usize % 4);
    // bit 0 = en, bit 1-3 = channel
    unsafe {
        ptr.write_volatile(
            ptr.read_volatile() & !(0xf << shift) | 1 << shift | (channel as u32) << (shift + 1),
        );
    }
}

fn enable_event_channel(channel: u8, pin: u8) {
    let gpio_sd = unsafe { GPIO_SD::steal() };
    gpio_sd
        .etm_event_ch_cfg(channel as usize)
        .modify(|_, w| w.event_en().clear_bit());
    gpio_sd
        .etm_event_ch_cfg(channel as usize)
        .modify(|_, w| unsafe { w.event_sel().bits(pin) });
    gpio_sd
        .etm_event_ch_cfg(channel as usize)
        .modify(|_, w| w.event_en().set_bit());
}
