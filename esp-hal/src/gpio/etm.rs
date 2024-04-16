//! # Event Task Matrix Function
//!
//! ## Overview
//!
//! GPIO supports ETM function, that is, the ETM task of GPIO can be
//! triggered by the ETM event of any peripheral, or the ETM task of any
//! peripheral can be triggered by the ETM event of GPIO.
//!
//! The GPIO ETM provides eight task channels. The ETM tasks that each task
//! channel can receive are:
//! - SET: GPIO goes high when triggered
//! - CLEAR: GPIO goes low when triggered
//! - TOGGLE: GPIO toggle level when triggered.
//!
//! GPIO has eight event channels, and the ETM events that each event
//! channel can generate are:
//! - RISE_EDGE: Indicates that the output signal of the corresponding GPIO has
//!   a rising edge
//! - FALL_EDGE: Indicates that the output signal of the corresponding GPIO has
//!   a falling edge
//! - ANY_EDGE: Indicates that the output signal of the corresponding GPIO is
//!   reversed
//!
//! ## Example
//! ```no_run
//! let led_task = gpio_ext.channel0_task.toggle(&mut led);
//! let button_event = gpio_ext.channel0_event.falling_edge(button);
//! ```

use crate::peripheral::{Peripheral, PeripheralRef};

/// All the GPIO ETM channels
#[non_exhaustive]
pub struct GpioEtmChannels<'d> {
    _gpio_sd: PeripheralRef<'d, crate::peripherals::GPIO_SD>,
    pub channel0_task: GpioEtmTaskChannel<0>,
    pub channel0_event: GpioEtmEventChannel<0>,
    pub channel1_task: GpioEtmTaskChannel<1>,
    pub channel1_event: GpioEtmEventChannel<1>,
    pub channel2_task: GpioEtmTaskChannel<2>,
    pub channel2_event: GpioEtmEventChannel<2>,
    pub channel3_task: GpioEtmTaskChannel<3>,
    pub channel3_event: GpioEtmEventChannel<3>,
    pub channel4_task: GpioEtmTaskChannel<4>,
    pub channel4_event: GpioEtmEventChannel<4>,
    pub channel5_task: GpioEtmTaskChannel<5>,
    pub channel5_event: GpioEtmEventChannel<5>,
    pub channel6_task: GpioEtmTaskChannel<6>,
    pub channel6_event: GpioEtmEventChannel<6>,
    pub channel7_task: GpioEtmTaskChannel<7>,
    pub channel7_event: GpioEtmEventChannel<7>,
}

impl<'d> GpioEtmChannels<'d> {
    pub fn new(peripheral: impl Peripheral<P = crate::peripherals::GPIO_SD> + 'd) -> Self {
        crate::into_ref!(peripheral);

        Self {
            _gpio_sd: peripheral,
            channel0_task: GpioEtmTaskChannel {},
            channel0_event: GpioEtmEventChannel {},
            channel1_task: GpioEtmTaskChannel {},
            channel1_event: GpioEtmEventChannel {},
            channel2_task: GpioEtmTaskChannel {},
            channel2_event: GpioEtmEventChannel {},
            channel3_task: GpioEtmTaskChannel {},
            channel3_event: GpioEtmEventChannel {},
            channel4_task: GpioEtmTaskChannel {},
            channel4_event: GpioEtmEventChannel {},
            channel5_task: GpioEtmTaskChannel {},
            channel5_event: GpioEtmEventChannel {},
            channel6_task: GpioEtmTaskChannel {},
            channel6_event: GpioEtmEventChannel {},
            channel7_task: GpioEtmTaskChannel {},
            channel7_event: GpioEtmEventChannel {},
        }
    }
}

/// An ETM controlled GPIO event
pub struct GpioEtmEventChannel<const C: u8> {}

impl<const C: u8> GpioEtmEventChannel<C> {
    /// Trigger at rising edge
    pub fn rising_edge<'d, PIN>(
        self,
        pin: impl Peripheral<P = PIN> + 'd,
    ) -> GpioEtmEventChannelRising<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        crate::into_ref!(pin);
        enable_event_channel(C, pin.number());
        GpioEtmEventChannelRising { _pin: pin }
    }

    /// Trigger at falling edge
    pub fn falling_edge<'d, PIN>(
        self,
        pin: impl Peripheral<P = PIN> + 'd,
    ) -> GpioEtmEventChannelFalling<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        crate::into_ref!(pin);
        enable_event_channel(C, pin.number());
        GpioEtmEventChannelFalling { _pin: pin }
    }

    /// Trigger at any edge
    pub fn any_edge<'d, PIN>(
        self,
        pin: impl Peripheral<P = PIN> + 'd,
    ) -> GpioEtmEventChannelAny<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        crate::into_ref!(pin);
        enable_event_channel(C, pin.number());
        GpioEtmEventChannelAny { _pin: pin }
    }
}

/// Event for rising edge
#[non_exhaustive]
pub struct GpioEtmEventChannelRising<'d, PIN, const C: u8>
where
    PIN: super::Pin,
{
    _pin: PeripheralRef<'d, PIN>,
}

impl<'d, PIN, const C: u8> crate::private::Sealed for GpioEtmEventChannelRising<'d, PIN, C> where
    PIN: super::Pin
{
}

impl<'d, PIN, const C: u8> crate::etm::EtmEvent for GpioEtmEventChannelRising<'d, PIN, C>
where
    PIN: super::Pin,
{
    fn id(&self) -> u8 {
        1 + C
    }
}

/// Event for falling edge
#[non_exhaustive]
pub struct GpioEtmEventChannelFalling<'d, PIN, const C: u8>
where
    PIN: super::Pin,
{
    _pin: PeripheralRef<'d, PIN>,
}

impl<'d, PIN, const C: u8> crate::private::Sealed for GpioEtmEventChannelFalling<'d, PIN, C> where
    PIN: super::Pin
{
}

impl<'d, PIN, const C: u8> crate::etm::EtmEvent for GpioEtmEventChannelFalling<'d, PIN, C>
where
    PIN: super::Pin,
{
    fn id(&self) -> u8 {
        9 + C
    }
}

/// Event for any edge
#[non_exhaustive]
pub struct GpioEtmEventChannelAny<'d, PIN, const C: u8>
where
    PIN: super::Pin,
{
    _pin: PeripheralRef<'d, PIN>,
}

impl<'d, PIN, const C: u8> crate::private::Sealed for GpioEtmEventChannelAny<'d, PIN, C> where
    PIN: super::Pin
{
}

impl<'d, PIN, const C: u8> crate::etm::EtmEvent for GpioEtmEventChannelAny<'d, PIN, C>
where
    PIN: super::Pin,
{
    fn id(&self) -> u8 {
        17 + C
    }
}

/// An ETM controlled GPIO task
pub struct GpioEtmTaskChannel<const C: u8> {}

impl<const C: u8> GpioEtmTaskChannel<C> {
    // In theory we could have multiple pins assigned to the same task. Not sure how
    // useful that would be. If we want to support it, the easiest would be
    // to offer additional functions like `set2`, `set3` etc. where the
    // number is the pin-count

    /// Task to set a high level
    pub fn set<'d, PIN>(self, pin: impl Peripheral<P = PIN> + 'd) -> GpioEtmTaskSet<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        crate::into_ref!(pin);
        enable_task_channel(C, pin.number());
        GpioEtmTaskSet { _pin: pin }
    }

    /// Task to set a low level
    pub fn clear<'d, PIN>(self, pin: impl Peripheral<P = PIN> + 'd) -> GpioEtmTaskClear<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        crate::into_ref!(pin);
        enable_task_channel(C, pin.number());
        GpioEtmTaskClear { _pin: pin }
    }

    /// Task to toggle the level
    pub fn toggle<'d, PIN>(
        self,
        pin: impl Peripheral<P = PIN> + 'd,
    ) -> GpioEtmTaskToggle<'d, PIN, C>
    where
        PIN: super::Pin,
    {
        crate::into_ref!(pin);
        enable_task_channel(C, pin.number());
        GpioEtmTaskToggle { _pin: pin }
    }
}

/// Task for set operation
#[non_exhaustive]
pub struct GpioEtmTaskSet<'d, PIN, const C: u8>
where
    PIN: super::Pin,
{
    _pin: PeripheralRef<'d, PIN>,
}

impl<'d, PIN, const C: u8> crate::private::Sealed for GpioEtmTaskSet<'d, PIN, C> where
    PIN: super::Pin
{
}

impl<'d, PIN, const C: u8> crate::etm::EtmTask for GpioEtmTaskSet<'d, PIN, C>
where
    PIN: super::Pin,
{
    fn id(&self) -> u8 {
        1 + C
    }
}

/// Task for clear operation
#[non_exhaustive]
pub struct GpioEtmTaskClear<'d, PIN, const C: u8> {
    _pin: PeripheralRef<'d, PIN>,
}

impl<'d, PIN, const C: u8> crate::private::Sealed for GpioEtmTaskClear<'d, PIN, C> where
    PIN: super::Pin
{
}

impl<'d, PIN, const C: u8> crate::etm::EtmTask for GpioEtmTaskClear<'d, PIN, C>
where
    PIN: super::Pin,
{
    fn id(&self) -> u8 {
        9 + C
    }
}

/// Task for toggle operation
#[non_exhaustive]
pub struct GpioEtmTaskToggle<'d, PIN, const C: u8> {
    _pin: PeripheralRef<'d, PIN>,
}

impl<'d, PIN, const C: u8> crate::private::Sealed for GpioEtmTaskToggle<'d, PIN, C> where
    PIN: super::Pin
{
}

impl<'d, PIN, const C: u8> crate::etm::EtmTask for GpioEtmTaskToggle<'d, PIN, C>
where
    PIN: super::Pin,
{
    fn id(&self) -> u8 {
        17 + C
    }
}

fn enable_task_channel(channel: u8, pin: u8) {
    let gpio_sd = unsafe { crate::peripherals::GPIO_SD::steal() };
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
    let gpio_sd = unsafe { crate::peripherals::GPIO_SD::steal() };
    gpio_sd
        .etm_event_ch_cfg(channel as usize)
        .modify(|_, w| w.etm_ch0_event_en().clear_bit());
    gpio_sd
        .etm_event_ch_cfg(channel as usize)
        .modify(|_, w| unsafe { w.etm_ch0_event_sel().bits(pin) });
    gpio_sd
        .etm_event_ch_cfg(channel as usize)
        .modify(|_, w| w.etm_ch0_event_en().set_bit());
}
