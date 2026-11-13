#![no_std]

// By default, we don't want probe-rs to interfere with test timings by halting
// cores and polling RTT. The tests don't produce output most of the time
// anyway. The only cases where output can be interesting are: during
// development, and when a test fails. In these cases, you can enable
// the `defmt` feature to get the output.

esp_bootloader_esp_idf::esp_app_desc!();

use esp_hal as _;

#[cfg(not(feature = "defmt"))]
#[defmt::global_logger]
struct Logger;

#[cfg(not(feature = "defmt"))]
unsafe impl defmt::Logger for Logger {
    fn acquire() {}
    unsafe fn flush() {}
    unsafe fn release() {}
    unsafe fn write(_bytes: &[u8]) {}
}

#[cfg(feature = "defmt")]
use defmt_rtt as _;

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! assert {
    ($($t:tt)*) => { defmt::assert!($($t)*) }
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! assert_eq {
    ($($t:tt)*) => { defmt::assert_eq!($($t)*) }
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! assert {
    ($($t:tt)*) => { ::core::assert!($($t)*) }
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! assert_eq {
    ($($t:tt)*) => { ::core::assert_eq!($($t)*) }
}

/// Signal to the radio HIL runner that harness firmware is ready.
///
/// The runner waits for this semihosting line before starting the DUT test.
pub fn signal_harness_ready() {
    semihosting::println!("[HARNESS-READY]");
}

/// Shared parameters for the IEEE 802.15.4 radio HIL tests.
///
/// The device-under-test and the `ieee802154_echo_support` firmware must agree
/// on these values so the two boards land on the same channel and PAN and can
/// address frames to each other.
pub mod ieee802154 {
    /// Radio channel used by both boards.
    pub const CHANNEL: u8 = 15;
    /// PAN ID shared by both boards.
    pub const PAN_ID: u16 = 0x4242;
    /// Short address of the device-under-test.
    pub const DUT_ADDRESS: u16 = 0x2222;
    /// Short address of the support (echo) firmware.
    pub const SUPPORT_ADDRESS: u16 = 0x2323;
    /// Payload the DUT transmits and the support firmware echoes back.
    pub const PAYLOAD: &[u8] = b"esp-radio 802.15.4 HIL";
}

/// Shared parameters for the BLE L2CAP HIL test.
///
/// The `ble_peripheral_support` firmware sends the data and the device-under-test
/// checks it.
pub mod ble {
    /// PSM of the L2CAP channel.
    pub const L2CAP_PSM: u16 = 0x0081;
    /// LE data length the support firmware sends with. The ESP32-C6 controller
    /// splits received ACL packets with 189 to 194 data bytes over two mbufs.
    pub const DATA_LENGTH: u16 = 191;
    /// Number of bytes the support firmware sends.
    pub const L2CAP_BYTES: usize = 8 * 1024;

    /// Byte at `index` of the data the support firmware sends.
    pub fn l2cap_byte(index: usize) -> u8 {
        index as u8
    }
}

#[macro_export]
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// A simple looping executor to test async code without esp-hal-embassy (which
// needs `esp-hal/unstable`).
#[cfg(not(feature = "embassy"))]
mod executor {
    use core::marker::PhantomData;

    use embassy_executor::{Spawner, raw};

    #[unsafe(export_name = "__pender")]
    fn __pender(_: *mut ()) {}

    pub struct Executor {
        inner: raw::Executor,
        not_send: PhantomData<*mut ()>,
    }

    impl Executor {
        pub fn new() -> Self {
            Self {
                inner: raw::Executor::new(core::ptr::null_mut()),
                not_send: PhantomData,
            }
        }

        pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
            init(self.inner.spawner());

            loop {
                unsafe { self.inner.poll() };
            }
        }
    }
}

#[cfg(feature = "embassy")]
pub use esp_rtos::embassy::Executor;
#[cfg(not(feature = "embassy"))]
pub use executor::Executor;
