//! # Event Task Matrix (ETM)
//!
//! ## Overview
//!
//! Normally, if a peripheral X needs to notify peripheral Y of a particular
//! event, this could only be done via a CPU interrupt from peripheral X, where
//! the CPU notifies peripheral Y on behalf of peripheral X. However, in
//! time-critical applications, the latency introduced by CPU interrupts is
//! non-negligible.
//!
//! With the help of the Event Task Matrix (ETM) module, some peripherals can
//! directly notify other peripherals of events through pre-set connections
//! without the intervention of CPU interrupts. This allows precise and low
//! latency synchronization between peripherals, and lessens the CPU’s workload
//! as the CPU no longer needs to handle these events.
//!
//! The ETM module has multiple programmable channels, they are used to connect
//! a particular Event to a particular Task. When an event is activated, the ETM
//! channel will trigger the corresponding task automatically.
//!
//! More information: <https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/etm.html>
//!
//! ## Example
//! ```no_run
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let mut led = io.pins.gpio1.into_push_pull_output();
//! let button = io.pins.gpio9.into_pull_down_input();
//!
//! // setup ETM
//! let gpio_ext = GpioEtmChannels::new(peripherals.GPIO_SD);
//! let led_task = gpio_ext.channel0_task.toggle(&mut led);
//! let button_event = gpio_ext.channel0_event.falling_edge(button);
//!
//! let etm = Etm::new(peripherals.SOC_ETM);
//! let channel0 = etm.channel0;
//!
//! // make sure the configured channel doesn't get dropped - dropping it will
//! // disable the channel
//! let _configured_channel = channel0.setup(&button_event, &led_task);
//!
//! // the LED is controlled by the button without involving the CPU
//! loop {}
//! ```

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
};

/// Unconfigured EtmChannel.
#[non_exhaustive]
pub struct EtmChannel<const C: u8> {}

impl<const C: u8> EtmChannel<C> {
    /// Setup the channel
    ///
    /// Enabled the channel and configures the assigned event and task.
    pub fn setup<'a, E, T>(self, event: &'a E, task: &'a T) -> EtmConfiguredChannel<'a, E, T, C>
    where
        E: EtmEvent,
        T: EtmTask,
    {
        let etm = unsafe { crate::peripherals::SOC_ETM::steal() };

        etm.ch(C as usize)
            .evt_id()
            .modify(|_, w| unsafe { w.evt_id().bits(event.id()) });
        etm.ch(C as usize)
            .task_id()
            .modify(|_, w| unsafe { w.task_id().bits(task.id()) });
        if C < 32 {
            etm.ch_ena_ad0_set().write(|w| w.ch_set(C).set_bit());
        } else {
            etm.ch_ena_ad1_set().write(|w| w.ch_set(C - 32).set_bit());
        }

        EtmConfiguredChannel {
            _event: event,
            _task: task,
        }
    }
}

fn disable_channel(channel: u8) {
    let etm = unsafe { crate::peripherals::SOC_ETM::steal() };
    if channel < 32 {
        etm.ch_ena_ad0_clr().write(|w| w.ch_clr(channel).set_bit());
    } else {
        etm.ch_ena_ad1_clr()
            .write(|w| w.ch_clr(channel - 32).set_bit());
    }
}

/// A readily configured channel
///
/// The channel is enabled and event and task are configured.
#[non_exhaustive]
pub struct EtmConfiguredChannel<'a, E, T, const C: u8>
where
    E: EtmEvent,
    T: EtmTask,
{
    _event: &'a E,
    _task: &'a T,
}

impl<'a, E, T, const C: u8> Drop for EtmConfiguredChannel<'a, E, T, C>
where
    E: EtmEvent,
    T: EtmTask,
{
    fn drop(&mut self) {
        debug!("drop {}", C);
        disable_channel(C);
    }
}

macro_rules! create_etm {
    ($($num:literal),+) => {
        paste::paste! {
            /// ETM Instance
            ///
            /// Provides access to all the [EtmChannel]
            pub struct Etm<'d> {
                _peripheral: PeripheralRef<'d, crate::peripherals::SOC_ETM>,
                $(pub [< channel $num >]: EtmChannel<$num>,)+
            }

            impl<'d> Etm<'d> {
                pub fn new(peripheral: impl Peripheral<P = crate::peripherals::SOC_ETM> + 'd) -> Self {
                    crate::into_ref!(peripheral);

                    PeripheralClockControl::enable(crate::system::Peripheral::Etm);

                    Self {
                        _peripheral: peripheral,
                        $([< channel $num >]: EtmChannel {},)+
                    }
                }
            }
        }
    };
}

create_etm!(
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49
);

#[doc(hidden)]
pub trait EtmEvent: crate::private::Sealed {
    fn id(&self) -> u8;
}

#[doc(hidden)]
pub trait EtmTask: crate::private::Sealed {
    fn id(&self) -> u8;
}
