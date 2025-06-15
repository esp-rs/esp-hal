#![no_std]

// By default, we don't want probe-rs to interfere with test timings by halting
// cores and polling RTT. The tests don't produce output most of the time
// anyway. The only cases where output can be interesting are: during
// development, and when a test fails. In these cases, you can enable
// the `defmt` feature to get the output.

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
// Make sure esp_backtrace is not removed.
use esp_backtrace as _;

#[macro_export]
macro_rules! i2c_pins {
    ($peripherals:expr) => {{
        // Order: (SDA, SCL)
        cfg_if::cfg_if! {
            if #[cfg(any(esp32s2, esp32s3))] {
                ($peripherals.GPIO2, $peripherals.GPIO3)
            } else if #[cfg(esp32)] {
                ($peripherals.GPIO32, $peripherals.GPIO33)
            } else if #[cfg(esp32c6)] {
                ($peripherals.GPIO6, $peripherals.GPIO7)
            } else if #[cfg(esp32h2)] {
                ($peripherals.GPIO12, $peripherals.GPIO22)
            } else if #[cfg(esp32c2)] {
                ($peripherals.GPIO18, $peripherals.GPIO9)
            } else { // esp32c3
                ($peripherals.GPIO4, $peripherals.GPIO5)
            }
        }
    }};
}

#[macro_export]
macro_rules! common_test_pins {
    ($peripherals:expr) => {{
        cfg_if::cfg_if! {
            if #[cfg(any(esp32s2, esp32s3))] {
                ($peripherals.GPIO9, $peripherals.GPIO10)
            } else if #[cfg(esp32)] {
                ($peripherals.GPIO2, $peripherals.GPIO4)
            } else {
                ($peripherals.GPIO2, $peripherals.GPIO3)
            }
        }
    }};
}

// A GPIO that's not connected to anything. We use the BOOT pin for this, but
// beware: it has a pullup.
#[macro_export]
macro_rules! unconnected_pin {
    ($peripherals:expr) => {{
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2, esp32s3))] {
                $peripherals.GPIO0
            } else if #[cfg(esp32c6)] {
                $peripherals.GPIO9
            } else if #[cfg(esp32h2)] {
                $peripherals.GPIO9
            } else if #[cfg(esp32c2)] {
                $peripherals.GPIO8
            } else {
                $peripherals.GPIO9
            }
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
pub use esp_hal_embassy::Executor;
#[cfg(not(feature = "embassy"))]
pub use executor::Executor;

/// Initialize esp-hal-embassy with 2 timers.
#[macro_export]
macro_rules! init_embassy {
    ($peripherals:expr, 2) => {{
        cfg_if::cfg_if! {
            if #[cfg(timergroup_timg_has_timer1)] {
                use esp_hal::timer::timg::TimerGroup;
                let timg0 = TimerGroup::new($peripherals.TIMG0);
                esp_hal_embassy::init([
                    timg0.timer0,
                    timg0.timer1,
                ]);
            } else if #[cfg(timergroup_timg1)] {
                use esp_hal::timer::timg::TimerGroup;
                let timg0 = TimerGroup::new($peripherals.TIMG0);
                let timg1 = TimerGroup::new($peripherals.TIMG1);
                esp_hal_embassy::init([
                    timg0.timer0,
                    timg1.timer0,
                ]);
            } else if #[cfg(systimer)] {
                use esp_hal::timer::systimer::SystemTimer;
                let systimer = SystemTimer::new($peripherals.SYSTIMER);
                esp_hal_embassy::init([
                    systimer.alarm0,
                    systimer.alarm1,
                ]);
            }
        }
    }};
}
