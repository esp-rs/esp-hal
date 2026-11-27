//! Macros that install the logger on UART0 or USB-Serial-JTAG with the default pins of the chip.

/// Installs the logger on UART0, on the default TX pin of the chip.
///
/// Takes the `Peripherals` that `esp_hal::init` returns, and moves `UART0` and the TX pin out of
/// it. The UART uses the default configuration of `esp_hal::uart::Config`: 115200 baud, 8 data
/// bits, no parity and 1 stop bit.
///
/// Use [`init_uart_split!`](crate::init_uart_split) to also get the receiver.
///
/// | Chip | TX pin |
/// | --- | --- |
/// | ESP32 | GPIO1 |
/// | ESP32-C2 | GPIO20 |
/// | ESP32-C3 | GPIO21 |
/// | ESP32-C5, ESP32-C61 | GPIO11 |
/// | ESP32-C6 | GPIO16 |
/// | ESP32-H2, ESP32-H4 | GPIO24 |
/// | ESP32-P4 | GPIO37 |
/// | ESP32-S2, ESP32-S3 | GPIO43 |
/// | ESP32-S31 | GPIO58 |
///
/// ```rust,ignore
/// let peripherals = esp_hal::init(esp_hal::Config::default());
/// esp_log::init_uart!(peripherals);
/// log::info!("Hello");
/// ```
///
/// The macro creates the driver through the `esp_hal` crate of the application. If the application
/// renames its esp-hal dependency, pass the name as the second argument, for example
/// `esp_log::init_uart!(peripherals, hal)`. All macros of this crate take this argument.
#[macro_export]
macro_rules! init_uart {
    ($peripherals:expr) => {
        $crate::init_uart!($peripherals, ::esp_hal)
    };
    ($peripherals:expr, $hal:path) => {{
        use $hal as _hal;
        let tx = _hal::uart::UartTx::new($peripherals.UART0, ::core::default::Default::default())
            .unwrap()
            .with_tx($crate::__uart0_pin!(tx, $peripherals));
        $crate::__log_on_uart!(tx, $hal);
    }};
}

/// Installs the logger on UART0, and returns the receiver of UART0.
///
/// Like [`init_uart!`](crate::init_uart), and also moves the default RX pin out of the
/// `Peripherals`. Returns the `esp_hal::uart::UartRx` on that pin, in blocking mode.
///
/// | Chip | RX pin |
/// | --- | --- |
/// | ESP32 | GPIO3 |
/// | ESP32-C2 | GPIO19 |
/// | ESP32-C3 | GPIO20 |
/// | ESP32-C5 | GPIO12 |
/// | ESP32-C6 | GPIO17 |
/// | ESP32-C61 | GPIO10 |
/// | ESP32-H2, ESP32-H4 | GPIO23 |
/// | ESP32-P4 | GPIO38 |
/// | ESP32-S2, ESP32-S3 | GPIO44 |
/// | ESP32-S31 | GPIO59 |
///
/// ```rust,ignore
/// let peripherals = esp_hal::init(esp_hal::Config::default());
/// let mut rx = esp_log::init_uart_split!(peripherals);
/// let mut buf = [0u8; 16];
/// let len = rx.read(&mut buf).unwrap();
/// ```
#[macro_export]
macro_rules! init_uart_split {
    ($peripherals:expr) => {
        $crate::init_uart_split!($peripherals, ::esp_hal)
    };
    ($peripherals:expr, $hal:path) => {{
        use $hal as _hal;
        let (rx, tx) =
            _hal::uart::Uart::new($peripherals.UART0, ::core::default::Default::default())
                .unwrap()
                .with_tx($crate::__uart0_pin!(tx, $peripherals))
                .with_rx($crate::__uart0_pin!(rx, $peripherals))
                .split();
        $crate::__log_on_uart!(tx, $hal);
        rx
    }};
}

/// Installs the logger on USB-Serial-JTAG.
///
/// Takes the `Peripherals` that `esp_hal::init` returns, and moves `USB_DEVICE` and the D+ and D-
/// pins out of it.
///
/// Use [`init_usb_split!`](crate::init_usb_split) to also get the receiver.
///
/// The logger keeps only the transmitter, so the chip can enter light sleep. The receiver that
/// [`init_usb_split!`](crate::init_usb_split) returns prevents automatic light sleep while it
/// exists.
///
/// The macro uses the pins of the default USB PHY. On the ESP32-H4 and the ESP32-P4, the
/// `USB_PHY_SEL` eFuse can move USB-Serial-JTAG to the other PHY. On such a chip, the driver
/// returns an error and the macro panics. Create the driver with the pins of the other PHY, and
/// pass its transmitter to [`init`](crate::init).
///
/// | Chip | D+ | D- |
/// | --- | --- | --- |
/// | ESP32-C3 | GPIO19 | GPIO18 |
/// | ESP32-C5, ESP32-H4 | GPIO14 | GPIO13 |
/// | ESP32-C6, ESP32-C61 | GPIO13 | GPIO12 |
/// | ESP32-H2 | GPIO27 | GPIO26 |
/// | ESP32-P4 | GPIO25 | GPIO24 |
/// | ESP32-S3 | GPIO20 | GPIO19 |
/// | ESP32-S31 | GPIO34 | GPIO33 |
///
/// ```rust,ignore
/// let peripherals = esp_hal::init(esp_hal::Config::default());
/// esp_log::init_usb!(peripherals);
/// log::info!("Hello");
/// ```
#[macro_export]
macro_rules! init_usb {
    ($peripherals:expr) => {
        $crate::init_usb!($peripherals, ::esp_hal)
    };
    ($peripherals:expr, $hal:path) => {{
        let (_, tx) = $crate::__usb_serial_jtag!($peripherals, $hal).split();
        $crate::__log_on_usb_serial_jtag!(tx, $hal);
    }};
}

/// Installs the logger on USB-Serial-JTAG, and returns the receiver.
///
/// Like [`init_usb!`](crate::init_usb). Returns the
/// `esp_hal::usb::usb_serial_jtag::UsbSerialJtagRx`, in blocking mode.
///
/// ```rust,ignore
/// let peripherals = esp_hal::init(esp_hal::Config::default());
/// let mut rx = esp_log::init_usb_split!(peripherals);
/// ```
#[macro_export]
macro_rules! init_usb_split {
    ($peripherals:expr) => {
        $crate::init_usb_split!($peripherals, ::esp_hal)
    };
    ($peripherals:expr, $hal:path) => {{
        let (rx, tx) = $crate::__usb_serial_jtag!($peripherals, $hal).split();
        $crate::__log_on_usb_serial_jtag!(tx, $hal);
        rx
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! __usb_serial_jtag {
    ($peripherals:expr, $hal:path) => {{
        use $hal as _hal;
        _hal::usb::usb_serial_jtag::UsbSerialJtag::new(
            $peripherals.USB_DEVICE,
            $crate::__usb_pin!(dp, $peripherals),
            $crate::__usb_pin!(dm, $peripherals),
            ::core::default::Default::default(),
        )
        .unwrap()
    }};
}

// The static lives at the call site, because the writer type comes from the application's esp-hal.

#[doc(hidden)]
#[macro_export]
macro_rules! __log_on_uart {
    ($tx:expr, $hal:path) => {{
        use $hal as _hal;
        static SLOT: $crate::__private::Slot<_hal::uart::UartTx<'static, _hal::Blocking>> =
            $crate::__private::Slot::new();
        SLOT.install($tx);
    }};
}

#[doc(hidden)]
#[macro_export]
macro_rules! __log_on_usb_serial_jtag {
    ($tx:expr, $hal:path) => {{
        use $hal as _hal;
        static SLOT: $crate::__private::Slot<
            _hal::usb::usb_serial_jtag::UsbSerialJtagTx<'static, _hal::Blocking>,
        > = $crate::__private::Slot::new();
        SLOT.install($tx);
    }};
}

// The default UART0 pins are the ones whose IO MUX function 0 is `U0TXD` and `U0RXD`. The ESP32-P4
// metadata does not list them; they are `U0TXD_GPIO_NUM` and `U0RXD_GPIO_NUM` of ESP-IDF.

#[cfg(feature = "esp32")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO1
    };
    (rx, $p:expr) => {
        $p.GPIO3
    };
}

#[cfg(feature = "esp32c2")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO20
    };
    (rx, $p:expr) => {
        $p.GPIO19
    };
}

#[cfg(feature = "esp32c3")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO21
    };
    (rx, $p:expr) => {
        $p.GPIO20
    };
}

#[cfg(feature = "esp32c5")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO11
    };
    (rx, $p:expr) => {
        $p.GPIO12
    };
}

#[cfg(feature = "esp32c6")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO16
    };
    (rx, $p:expr) => {
        $p.GPIO17
    };
}

#[cfg(feature = "esp32c61")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO11
    };
    (rx, $p:expr) => {
        $p.GPIO10
    };
}

#[cfg(any(feature = "esp32h2", feature = "esp32h4"))]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO24
    };
    (rx, $p:expr) => {
        $p.GPIO23
    };
}

#[cfg(feature = "esp32p4")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO37
    };
    (rx, $p:expr) => {
        $p.GPIO38
    };
}

#[cfg(any(feature = "esp32s2", feature = "esp32s3"))]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO43
    };
    (rx, $p:expr) => {
        $p.GPIO44
    };
}

#[cfg(feature = "esp32s31")]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    (tx, $p:expr) => {
        $p.GPIO58
    };
    (rx, $p:expr) => {
        $p.GPIO59
    };
}

#[cfg(not(any(
    feature = "esp32",
    feature = "esp32c2",
    feature = "esp32c3",
    feature = "esp32c5",
    feature = "esp32c6",
    feature = "esp32c61",
    feature = "esp32h2",
    feature = "esp32h4",
    feature = "esp32p4",
    feature = "esp32s2",
    feature = "esp32s3",
    feature = "esp32s31",
)))]
#[doc(hidden)]
#[macro_export]
macro_rules! __uart0_pin {
    ($pin:ident, $p:expr) => {
        ::core::compile_error!("enable the chip feature of esp-log")
    };
}

// The D+ and D- pins of the default USB PHY.

#[cfg(feature = "esp32c3")]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    (dp, $p:expr) => {
        $p.GPIO19
    };
    (dm, $p:expr) => {
        $p.GPIO18
    };
}

#[cfg(any(feature = "esp32c5", feature = "esp32h4"))]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    (dp, $p:expr) => {
        $p.GPIO14
    };
    (dm, $p:expr) => {
        $p.GPIO13
    };
}

#[cfg(any(feature = "esp32c6", feature = "esp32c61"))]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    (dp, $p:expr) => {
        $p.GPIO13
    };
    (dm, $p:expr) => {
        $p.GPIO12
    };
}

#[cfg(feature = "esp32h2")]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    (dp, $p:expr) => {
        $p.GPIO27
    };
    (dm, $p:expr) => {
        $p.GPIO26
    };
}

#[cfg(feature = "esp32p4")]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    (dp, $p:expr) => {
        $p.GPIO25
    };
    (dm, $p:expr) => {
        $p.GPIO24
    };
}

#[cfg(feature = "esp32s3")]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    (dp, $p:expr) => {
        $p.GPIO20
    };
    (dm, $p:expr) => {
        $p.GPIO19
    };
}

#[cfg(feature = "esp32s31")]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    (dp, $p:expr) => {
        $p.GPIO34
    };
    (dm, $p:expr) => {
        $p.GPIO33
    };
}

#[cfg(not(any(
    feature = "esp32c3",
    feature = "esp32c5",
    feature = "esp32c6",
    feature = "esp32c61",
    feature = "esp32h2",
    feature = "esp32h4",
    feature = "esp32p4",
    feature = "esp32s3",
    feature = "esp32s31",
)))]
#[doc(hidden)]
#[macro_export]
macro_rules! __usb_pin {
    ($pin:ident, $p:expr) => {
        ::core::compile_error!(
            "the chip has no USB-Serial-JTAG, or the chip feature of esp-log is not enabled"
        )
    };
}
