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

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! assert_ne {
    ($($t:tt)*) => { defmt::assert_ne!($($t)*) }
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! panic {
    ($($t:tt)*) => { defmt::panic!($($t)*) }
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

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! assert_ne {
    ($($t:tt)*) => { ::core::assert_ne!($($t)*) }
}

#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! panic {
    ($($t:tt)*) => { ::core::panic!($($t)*) }
}

#[macro_export]
macro_rules! i2c_pins {
    ($peripherals:expr) => {{
        // Order: (SDA, SCL)
        cfg_select! {
            any(esp32s2, esp32s3) => ($peripherals.GPIO3, $peripherals.GPIO2),
            esp32 => ($peripherals.GPIO32, $peripherals.GPIO33),
            any(esp32c6, esp32c61) => ($peripherals.GPIO6, $peripherals.GPIO7),
            esp32s31 => ($peripherals.GPIO7, $peripherals.GPIO6),
            esp32h2 => ($peripherals.GPIO12, $peripherals.GPIO22),
            esp32c2 => ($peripherals.GPIO18, $peripherals.GPIO9),
            any(esp32c5, esp32p4) => ($peripherals.GPIO2, $peripherals.GPIO3),
            // esp32c3
            _ => ($peripherals.GPIO4, $peripherals.GPIO5),
        }
    }};
}

#[macro_export]
macro_rules! common_test_pins {
    ($peripherals:expr) => {{
        cfg_select! {
            any(esp32s2, esp32s3, esp32c5) => ($peripherals.GPIO9, $peripherals.GPIO10),
            esp32 => ($peripherals.GPIO2, $peripherals.GPIO4),
            esp32p4 => ($peripherals.GPIO5, $peripherals.GPIO6),
            // esp32c6, esp32c61, esp32h2, esp32c2, esp32c3, esp32s31
            _ => ($peripherals.GPIO2, $peripherals.GPIO3),
        }
    }};
}

// A GPIO that's not connected to anything. We use the BOOT pin for this, but
// beware: it has a pullup.
#[macro_export]
macro_rules! unconnected_pin {
    ($peripherals:expr) => {{
        cfg_select! {
            any(esp32, esp32s2, esp32s3) => $peripherals.GPIO0,
            esp32c2 => $peripherals.GPIO8,
            esp32c5 => $peripherals.GPIO28,
            esp32p4 => $peripherals.GPIO35,
            esp32s31 => $peripherals.GPIO61,
            // esp32c3, esp32c6, esp32c61, esp32h2
            _ => $peripherals.GPIO9,
        }
    }};
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

/// Disables every watchdog timer with raw register writes.
///
/// This prevents the firmware reset loop if no debugger is attached. The function must not depend
/// on esp-hal drivers, because a driver that is broken or half-initialized must not stop the tests
/// from running.
#[embedded_test::setup]
fn disable_watchdogs_before_semihosting() {
    cfg_select! {
        soc_has_lp_wdt => {
            use esp_hal::peripherals::LP_WDT;
        }
        _ => {
            use esp_hal::peripherals::LPWR as LP_WDT;
        }
    }

    // RWDT
    let lp_wdt = LP_WDT::regs();
    cfg_select! {
        esp32p4 => {
            lp_wdt.wprotect().write(|w| unsafe { w.bits(0x50D8_3AA1) });
            lp_wdt.config0().write(|w| unsafe { w.bits(0) });
            lp_wdt.wprotect().write(|w| unsafe { w.bits(0) });
        }
        _ => {
            lp_wdt
                .wdtwprotect()
                .write(|w| unsafe { w.bits(0x50D8_3AA1) });
            lp_wdt.wdtconfig0().write(|w| unsafe { w.bits(0) });
            lp_wdt.wdtwprotect().write(|w| unsafe { w.bits(0) });
        }
    }

    // SWD
    #[cfg(soc_has_swd_watchdog)]
    {
        const SWD_WKEY: u32 = cfg_select! {
            any(esp32c2, esp32c3, esp32s2, esp32s3) => 0x8F1D_312A,
            _ => 0x50D8_3AA1,
        };

        lp_wdt
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(SWD_WKEY) });
        cfg_select! {
            esp32p4 => lp_wdt
                .swd_config()
                .write(|w| w.swd_auto_feed_en().set_bit()),
            _ => lp_wdt.swd_conf().write(|w| w.swd_auto_feed_en().set_bit()),
        };
        lp_wdt
            .swd_wprotect()
            .write(|w| unsafe { w.swd_wkey().bits(0) });
    }

    // MWDT
    #[cfg(timergroup_timg0)]
    {
        let timg0 = esp_hal::peripherals::TIMG0::regs();
        timg0
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1) });
        timg0.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
        timg0
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0) });
    }

    #[cfg(timergroup_timg1)]
    {
        let timg1 = esp_hal::peripherals::TIMG1::regs();
        timg1
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0x50D8_3AA1) });
        timg1.wdtconfig0().modify(|_, w| w.wdt_en().clear_bit());
        timg1
            .wdtwprotect()
            .write(|w| unsafe { w.wdt_wkey().bits(0) });
    }
}
