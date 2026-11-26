#![no_std]

// By default, the tests log through esp-log on the serial port that probe-rs reads, because
// polling RTT halts the cores. The tests that use UART0 or the USB-Serial-JTAG port for themselves
// log over RTT.

#[cfg(all(feature = "log-serial", feature = "log-rtt"))]
compile_error!(
    "Enable only one of `log-serial` and `log-rtt`. Add `//% DEFAULT-FEATURES: false` to use `log-rtt`."
);

esp_bootloader_esp_idf::esp_app_desc!();

use esp_hal as _;
#[cfg(feature = "log-serial")]
#[doc(hidden)]
pub use esp_log;

/// Installs the logger on the serial port that probe-rs reads: USB-Serial-JTAG where the chip has
/// it, otherwise UART0 on its default TX pin. Moves the fields it uses out of `$peripherals`.
///
/// The drivers are unstable, so this does nothing unless the test enables `unstable`. It also does
/// nothing with `log-rtt`.
#[macro_export]
macro_rules! init_logger {
    ($peripherals:expr) => {
        #[cfg(all(feature = "log-serial", feature = "unstable"))]
        $crate::init_logger!(@install $peripherals);
        #[cfg(not(all(feature = "log-serial", feature = "unstable")))]
        let _ = &$peripherals;
    };
    (@install $peripherals:expr) => {{
        cfg_select! {
            // `esp_log::init_usb!` uses the pins of the default PHY, but the eFuse of some boards
            // selects the other one.
            soc_has_usb_device => {
                static WRITER: static_cell::StaticCell<
                    esp_hal::usb::usb_serial_jtag::UsbSerialJtagTx<'static, esp_hal::Blocking>,
                > = static_cell::StaticCell::new();
                let (_, tx) = $crate::usb_serial_jtag!($peripherals, Default::default())
                    .unwrap()
                    .split();
                $crate::esp_log::init(WRITER.init(tx));
            }
            _ => {
                $crate::esp_log::init_uart!($peripherals);
            }
        }
    }};
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    // Without a debugger, a semihosting call raises a debug exception that panics again. On
    // Xtensa, the nested panics run at the debug interrupt level, so the stack grows into the
    // peripheral registers until the chip resets, and the chip state can stay damaged after the
    // reset.
    #[cfg(target_arch = "xtensa")]
    if !esp_hal::debugger::debugger_connected() {
        loop {
            core::hint::spin_loop();
        }
    }

    semihosting::eprintln!("{info}");
    semihosting::process::exit(101)
}

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
macro_rules! assert_matches {
    ($left:expr, $pattern:pat $(if $guard:expr)? $(,)?) => {
        defmt::assert!(::core::matches!($left, $pattern $(if $guard)?))
    };
    ($left:expr, $pattern:pat $(if $guard:expr)?, $($arg:tt)+) => {
        defmt::assert!(::core::matches!($left, $pattern $(if $guard)?), $($arg)+)
    };
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
macro_rules! assert_matches {
    ($($t:tt)*) => { ::core::assert_matches!($($t)*) }
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
            esp32s31 => ($peripherals.GPIO1, $peripherals.GPIO0),
            esp32h2 => ($peripherals.GPIO12, $peripherals.GPIO22),
            esp32h4 => ($peripherals.GPIO26, $peripherals.GPIO27),
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
            // esp32c6, esp32c61, esp32h2, esp32h4, esp32c2, esp32c3, esp32s31
            _ => ($peripherals.GPIO2, $peripherals.GPIO3),
        }
    }};
}

// A pair of connected LP pads.
#[macro_export]
macro_rules! lp_test_pins {
    ($peripherals:expr) => {{
        cfg_select! {
            esp32c5 => ($peripherals.GPIO4, $peripherals.GPIO5),
            esp32h2 => ($peripherals.GPIO10, $peripherals.GPIO11),
            // The pair of every other board is the one of `common_test_pins!`.
            _ => $crate::common_test_pins!($peripherals),
        }
    }};
}

// A pair of connected HP pads.
//
// On the C2 the second pin is an LP pad, because the board has no second free HP pad. It only
// observes the first one, so the pad under test is still HP.
//
// The ESP32 is missing, the devkit we use has no free HP pad at all.
#[macro_export]
macro_rules! hp_test_pins {
    ($peripherals:expr) => {{
        cfg_select! {
            esp32c2 => ($peripherals.GPIO10, $peripherals.GPIO0),
            esp32c3 => ($peripherals.GPIO6, $peripherals.GPIO7),
            any(esp32c6, esp32c61) => ($peripherals.GPIO22, $peripherals.GPIO23),
            esp32h4 => ($peripherals.GPIO19, $peripherals.GPIO20),
            esp32p4 => ($peripherals.GPIO46, $peripherals.GPIO53),
            esp32s2 => ($peripherals.GPIO37, $peripherals.GPIO38),
            esp32s3 => ($peripherals.GPIO40, $peripherals.GPIO41),
            esp32s31 => ($peripherals.GPIO43, $peripherals.GPIO44),
            // The HP pair of the C5 and of the H2 is the one of `common_test_pins!`.
            any(esp32c5, esp32h2) => $crate::common_test_pins!($peripherals),
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
            // H4 has no BOOT GPIO pin, the boot mode is selected by the dedicated STRAP_MODE_0
            // pin, which is not a GPIO.
            esp32h4 => $peripherals.GPIO34,
            // esp32c3, esp32c6, esp32c61, esp32h2
            _ => $peripherals.GPIO9,
        }
    }};
}

// The ADC1 GPIO connected to the MCP4725 DAC's VOUT pin.
#[macro_export]
macro_rules! adc_pin {
    ($peripherals:expr) => {{
        cfg_select! {
            esp32 => $peripherals.GPIO36,
            any(esp32c2, esp32c3) => $peripherals.GPIO1,
            esp32c5 => $peripherals.GPIO6,
            any(esp32c6, esp32c61, esp32h2) => $peripherals.GPIO4,
            esp32h4 => $peripherals.GPIO28,
            esp32p4 => $peripherals.GPIO20,
            any(esp32s2, esp32s3) => $peripherals.GPIO7,
            esp32s31 => $peripherals.GPIO47,
        }
    }};
}

// Creates the USB Serial/JTAG driver on the D+ and D- pins of the PHY that the controller uses.
// With `other_phy`, it passes the pins of the other PHY of the chip instead.
//
// The H4 rig board has the `USB_PHY_SEL` eFuse burned, so the pins differ between boards.
#[macro_export]
macro_rules! usb_serial_jtag {
    ($peripherals:expr, $config:expr) => {
        $crate::usb_serial_jtag!(@new $peripherals, $config, $crate::usb_phy_sel_burned!())
    };
    ($peripherals:expr, $config:expr, other_phy) => {
        $crate::usb_serial_jtag!(@new $peripherals, $config, !$crate::usb_phy_sel_burned!())
    };
    (@new $peripherals:expr, $config:expr, $usb_fs_phy:expr) => {{
        // Order: (D+, D-)
        cfg_select! {
            esp32c3 => $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO19, GPIO18),
            esp32c5 => $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO14, GPIO13),
            any(esp32c6, esp32c61) => {
                $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO13, GPIO12)
            }
            esp32h2 => $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO27, GPIO26),
            esp32h4 => {
                if $usb_fs_phy {
                    $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO22, GPIO21)
                } else {
                    $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO14, GPIO13)
                }
            }
            esp32p4 => {
                if $usb_fs_phy {
                    $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO27, GPIO26)
                } else {
                    $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO25, GPIO24)
                }
            }
            esp32s3 => $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO20, GPIO19),
            esp32s31 => $crate::usb_serial_jtag!(@pins $peripherals, $config, GPIO34, GPIO33),
        }
    }};
    (@pins $peripherals:expr, $config:expr, $dp:ident, $dm:ident) => {
        esp_hal::usb::usb_serial_jtag::UsbSerialJtag::new(
            $peripherals.USB_DEVICE,
            $peripherals.$dp,
            $peripherals.$dm,
            $config,
        )
    };
}

// Returns whether the `USB_PHY_SEL` eFuse moves USB Serial/JTAG to the USB FS PHY. The tests do not
// write the PHY selection register, so the eFuse selects the PHY.
#[doc(hidden)]
#[macro_export]
macro_rules! usb_phy_sel_burned {
    () => {
        cfg_select! {
            all(usb_serial_jtag_phy_pads_usj, usb_serial_jtag_phy_pads_usb_fs) => {
                esp_hal::efuse::read_bit(esp_hal::efuse::USB_PHY_SEL)
            }
            _ => false,
        }
    };
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

/// Starts the RTT logger if `log-rtt` is enabled, and disables every watchdog timer with raw
/// register writes.
///
/// Disabling the watchdogs prevents the firmware reset loop if no debugger is attached. The
/// function must not depend on esp-hal drivers, because a driver that is broken or half-initialized
/// must not stop the tests from running.
#[embedded_test::setup]
fn init_rtt_and_disable_watchdogs() {
    #[cfg(feature = "log-rtt")]
    {
        let channels = cfg_select! {
            esp32p4 => rtt_target::rtt_init! {
                up: {
                    0: {
                        size: 1024,
                        name: "defmt",
                        section: ".uncached.data",
                    }
                }
                section_cb: ".uncached.data"
            },
            _ => rtt_target::rtt_init! {
                up: {
                    0: {
                        size: 1024,
                        name: "defmt",
                    }
                }
            },
        };

        rtt_target::set_defmt_channel(channels.up.0);
    }

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
