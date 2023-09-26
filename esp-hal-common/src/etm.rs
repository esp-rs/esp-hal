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
//! More information: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/etm.html
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
//! let etm = Etm::new(peripherals.SOC_ETM, &mut system.peripheral_clock_control);
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

macro_rules! impl_etm_channel {
    ($channel: literal, $bank: literal) => {
        paste::paste! {
            impl EtmChannel<$channel> {
                /// Setup the channel
                ///
                /// Enabled the channel and configures the assigned event and task.
                pub fn setup<'a, E, T>(self, event: &'a E, task: &'a T) -> EtmConfiguredChannel<'a, E,T,$channel>
                where
                    E: EtmEvent,
                    T: EtmTask,
                {
                    let etm = unsafe { crate::peripherals::SOC_ETM::steal() };

                    etm.[< ch $channel _evt_id >].modify(|_, w| w.[< ch $channel _evt_id >]().variant(event.id()));
                    etm.[< ch $channel _task_id >].modify(|_, w| w.[< ch $channel _task_id >]().variant(task.id()));
                    etm.[< ch_ena_ad $bank _set >].write(|w| w.[< ch_set $channel >]().set_bit());

                    EtmConfiguredChannel {
                        _event: event,
                        _task: task,
                    }
                }
            }
        }
    };
}

impl_etm_channel!(0, 0);
impl_etm_channel!(1, 0);
impl_etm_channel!(2, 0);
impl_etm_channel!(3, 0);
impl_etm_channel!(4, 0);
impl_etm_channel!(5, 0);
impl_etm_channel!(6, 0);
impl_etm_channel!(7, 0);
impl_etm_channel!(8, 0);
impl_etm_channel!(9, 0);
impl_etm_channel!(10, 0);
impl_etm_channel!(11, 0);
impl_etm_channel!(12, 0);
impl_etm_channel!(13, 0);
impl_etm_channel!(14, 0);
impl_etm_channel!(15, 0);
impl_etm_channel!(16, 0);
impl_etm_channel!(17, 0);
impl_etm_channel!(18, 0);
impl_etm_channel!(19, 0);
impl_etm_channel!(20, 0);
impl_etm_channel!(21, 0);
impl_etm_channel!(22, 0);
impl_etm_channel!(23, 0);
impl_etm_channel!(24, 0);
impl_etm_channel!(25, 0);
impl_etm_channel!(26, 0);
impl_etm_channel!(27, 0);
impl_etm_channel!(28, 0);
impl_etm_channel!(29, 0);
impl_etm_channel!(30, 0);
impl_etm_channel!(31, 0);
impl_etm_channel!(32, 1);
impl_etm_channel!(33, 1);
impl_etm_channel!(34, 1);
impl_etm_channel!(35, 1);
impl_etm_channel!(36, 1);
impl_etm_channel!(37, 1);
impl_etm_channel!(38, 1);
impl_etm_channel!(39, 1);
impl_etm_channel!(40, 1);
impl_etm_channel!(41, 1);
impl_etm_channel!(42, 1);
impl_etm_channel!(43, 1);
impl_etm_channel!(44, 1);
impl_etm_channel!(45, 1);
impl_etm_channel!(46, 1);
impl_etm_channel!(47, 1);
impl_etm_channel!(48, 1);
impl_etm_channel!(49, 1);

macro_rules! impl_disable_helper {
    ($(($channel:literal, $bank:literal)),+) => {
        paste::paste! {
            fn disable_channel(channel: u8) {
                let etm = unsafe { crate::peripherals::SOC_ETM::steal() };
                match channel {
                    $(
                        $channel => {etm.[< ch_ena_ad $bank _clr>].write(|w| w.[< ch_clr $channel >]().set_bit());},
                    )+
                    _ => panic!("Unknown channel {}", channel),
                }

            }
        }
    };
}
impl_disable_helper!(
    (0, 0),
    (1, 0),
    (2, 0),
    (3, 0),
    (4, 0),
    (5, 0),
    (6, 0),
    (7, 0),
    (8, 0),
    (9, 0),
    (10, 0),
    (11, 0),
    (12, 0),
    (13, 0),
    (14, 0),
    (15, 0),
    (16, 0),
    (17, 0),
    (18, 0),
    (19, 0),
    (20, 0),
    (21, 0),
    (22, 0),
    (23, 0),
    (24, 0),
    (25, 0),
    (26, 0),
    (27, 0),
    (28, 0),
    (29, 0),
    (30, 0),
    (31, 0),
    (32, 1),
    (33, 1),
    (34, 1),
    (35, 1),
    (36, 1),
    (37, 1),
    (38, 1),
    (39, 1),
    (40, 1),
    (41, 1),
    (42, 1),
    (43, 1),
    (44, 1),
    (45, 1),
    (46, 1),
    (47, 1),
    (48, 1),
    (49, 1)
);

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

macro_rules! create_etm_struct {
    ($($num:literal),+) => {
        paste::paste! {
            /// ETM Instance
            ///
            /// Provides access to all the [EtmChannel]
            pub struct Etm<'d> {
                _peripheral: PeripheralRef<'d, crate::peripherals::SOC_ETM>,
                $(pub [< channel $num >]: EtmChannel<$num>,)+
            }
        }
    };
}

macro_rules! create_etm_constructor {
    ($($num:literal),+) => {
        paste::paste! {
            impl<'d> Etm<'d> {
                pub fn new(
                    peripheral: impl Peripheral<P = crate::peripherals::SOC_ETM> + 'd,
                    peripheral_clock_control: &mut PeripheralClockControl,
                ) -> Self {
                    crate::into_ref!(peripheral);
                    peripheral_clock_control.enable(crate::system::Peripheral::Etm);

                    Self {
                        _peripheral: peripheral,
                        $([< channel $num >]: EtmChannel {},)+
                    }
                }
            }
        }
    };
}

create_etm_struct!(
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49
);

create_etm_constructor!(
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49
);

#[doc(hidden)]
pub trait EtmEvent: private::Sealed {
    fn id(&self) -> u8;
}

#[doc(hidden)]
pub trait EtmTask: private::Sealed {
    fn id(&self) -> u8;
}

pub(crate) mod private {
    pub trait Sealed {}
}
