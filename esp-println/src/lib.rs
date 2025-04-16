#![doc = include_str!("../README.md")]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(rustdoc::bare_urls)]
#![no_std]

#[cfg(feature = "defmt-espflash")]
pub mod defmt;
#[cfg(feature = "log")]
pub mod logger;

macro_rules! log_format {
    ($value:expr_2021) => {
        #[unsafe(link_section = concat!(".espressif.metadata"))]
        #[used]
        #[unsafe(export_name = concat!("espflash.LOG_FORMAT"))]
        static LOG_FORMAT: [u8; $value.len()] = const {
            let val_bytes = $value.as_bytes();
            let mut val_bytes_array = [0; $value.len()];
            let mut i = 0;
            while i < val_bytes.len() {
                val_bytes_array[i] = val_bytes[i];
                i += 1;
            }
            val_bytes_array
        };
    };
}

#[cfg(feature = "defmt-espflash")]
log_format!("defmt-espflash");

#[cfg(not(feature = "defmt-espflash"))]
log_format!("serial");

/// Prints to the selected output, with a newline.
#[cfg(not(feature = "no-op"))]
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {{
        {
            use core::fmt::Write;
            writeln!($crate::Printer, $($arg)*).ok();
        }
    }};
}

/// Prints to the selected output.
#[cfg(not(feature = "no-op"))]
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        {
            use core::fmt::Write;
            write!($crate::Printer, $($arg)*).ok();
        }
    }};
}

/// Prints to the configured output, with a newline.
#[cfg(feature = "no-op")]
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {{}};
}

/// Prints to the configured output.
#[cfg(feature = "no-op")]
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{}};
}

/// Prints and returns the value of a given expression for quick and dirty
/// debugging.
// implementation adapted from `std::dbg`
#[macro_export]
macro_rules! dbg {
    // NOTE: We cannot use `concat!` to make a static string as a format argument
    // of `eprintln!` because `file!` could contain a `{` or
    // `$val` expression could be a block (`{ .. }`), in which case the `println!`
    // will be malformed.
    () => {
        $crate::println!("[{}:{}]", ::core::file!(), ::core::line!())
    };
    ($val:expr_2021 $(,)?) => {
        // Use of `match` here is intentional because it affects the lifetimes
        // of temporaries - https://stackoverflow.com/a/48732525/1063961
        match $val {
            tmp => {
                $crate::println!("[{}:{}] {} = {:#?}",
                    ::core::file!(), ::core::line!(), ::core::stringify!($val), &tmp);
                tmp
            }
        }
    };
    ($($val:expr_2021),+ $(,)?) => {
        ($($crate::dbg!($val)),+,)
    };
}

/// The printer that is used by the `print!` and `println!` macros.
pub struct Printer;

impl core::fmt::Write for Printer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Printer::write_bytes(s.as_bytes());
        Ok(())
    }
}

impl Printer {
    /// Writes a byte slice to the configured output.
    pub fn write_bytes(bytes: &[u8]) {
        with(|token| {
            PrinterImpl::write_bytes_in_cs(bytes, token);
            PrinterImpl::flush(token);
        })
    }
}

#[cfg(feature = "jtag-serial")]
type PrinterImpl = serial_jtag_printer::Printer;

#[cfg(feature = "uart")]
type PrinterImpl = uart_printer::Printer;

#[cfg(feature = "auto")]
type PrinterImpl = auto_printer::Printer;

#[cfg(all(
    feature = "auto",
    any(
        feature = "esp32c3",
        feature = "esp32c6",
        feature = "esp32h2",
        feature = "esp32p4",    // as mentioned in 'build.rs'
        feature = "esp32s3"
    )
))]
mod auto_printer {
    use crate::{
        LockToken,
        serial_jtag_printer::Printer as PrinterSerialJtag,
        uart_printer::Printer as PrinterUart,
    };

    pub struct Printer;
    impl Printer {
        fn use_jtag() -> bool {
            // Decide if serial-jtag is used by checking SOF interrupt flag.
            // SOF packet is sent by the HOST every 1ms on a full speed bus.
            // Between two consecutive ticks, there will be at least 1ms (selectable tick
            // rate range is 1 - 1000Hz).
            // We don't reset the flag - if it was ever connected we assume serial-jtag is
            // used
            #[cfg(feature = "esp32c3")]
            const USB_DEVICE_INT_RAW: *const u32 = 0x60043008 as *const u32;
            #[cfg(feature = "esp32c6")]
            const USB_DEVICE_INT_RAW: *const u32 = 0x6000f008 as *const u32;
            #[cfg(feature = "esp32h2")]
            const USB_DEVICE_INT_RAW: *const u32 = 0x6000f008 as *const u32;
            #[cfg(feature = "esp32p4")]
            const USB_DEVICE_INT_RAW: *const u32 = unimplemented!();
            #[cfg(feature = "esp32s3")]
            const USB_DEVICE_INT_RAW: *const u32 = 0x60038000 as *const u32;

            const SOF_INT_MASK: u32 = 0b10;

            unsafe { (USB_DEVICE_INT_RAW.read_volatile() & SOF_INT_MASK) != 0 }
        }

        pub fn write_bytes_in_cs(bytes: &[u8], token: LockToken<'_>) {
            if Self::use_jtag() {
                PrinterSerialJtag::write_bytes_in_cs(bytes, token);
            } else {
                PrinterUart::write_bytes_in_cs(bytes, token);
            }
        }

        pub fn flush(token: LockToken<'_>) {
            if Self::use_jtag() {
                PrinterSerialJtag::flush(token);
            } else {
                PrinterUart::flush(token);
            }
        }
    }
}

#[cfg(all(
    feature = "auto",
    not(any(
        feature = "esp32c3",
        feature = "esp32c6",
        feature = "esp32h2",
        feature = "esp32p4",
        feature = "esp32s3"
    ))
))]
mod auto_printer {
    // models that only have UART
    pub type Printer = crate::uart_printer::Printer;
}

#[cfg(all(
    any(feature = "jtag-serial", feature = "auto"),
    any(
        feature = "esp32c3",
        feature = "esp32c6",
        feature = "esp32h2",
        feature = "esp32p4",
        feature = "esp32s3"
    )
))]
mod serial_jtag_printer {
    use portable_atomic::{AtomicBool, Ordering};

    use super::LockToken;
    pub struct Printer;

    #[cfg(feature = "esp32c3")]
    const SERIAL_JTAG_FIFO_REG: usize = 0x6004_3000;
    #[cfg(feature = "esp32c3")]
    const SERIAL_JTAG_CONF_REG: usize = 0x6004_3004;

    #[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
    const SERIAL_JTAG_FIFO_REG: usize = 0x6000_F000;
    #[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
    const SERIAL_JTAG_CONF_REG: usize = 0x6000_F004;

    #[cfg(feature = "esp32p4")]
    const SERIAL_JTAG_FIFO_REG: usize = 0x500D_2000;
    #[cfg(feature = "esp32p4")]
    const SERIAL_JTAG_CONF_REG: usize = 0x500D_2004;

    #[cfg(feature = "esp32s3")]
    const SERIAL_JTAG_FIFO_REG: usize = 0x6003_8000;
    #[cfg(feature = "esp32s3")]
    const SERIAL_JTAG_CONF_REG: usize = 0x6003_8004;

    /// A previous wait has timed out. We use this flag to avoid blocking
    /// forever if there is no host attached.
    static TIMED_OUT: AtomicBool = AtomicBool::new(false);

    fn fifo_flush() {
        let conf = SERIAL_JTAG_CONF_REG as *mut u32;
        unsafe { conf.write_volatile(0b001) };
    }

    fn fifo_full() -> bool {
        let conf = SERIAL_JTAG_CONF_REG as *mut u32;
        unsafe { conf.read_volatile() & 0b010 == 0b000 }
    }

    fn fifo_write(byte: u8) {
        let fifo = SERIAL_JTAG_FIFO_REG as *mut u32;
        unsafe { fifo.write_volatile(byte as u32) }
    }

    fn wait_for_flush() -> bool {
        const TIMEOUT_ITERATIONS: usize = 50_000;

        // Wait for some time for the FIFO to clear.
        let mut timeout = TIMEOUT_ITERATIONS;
        while fifo_full() {
            if timeout == 0 {
                TIMED_OUT.store(true, Ordering::Relaxed);
                return false;
            }
            timeout -= 1;
        }

        true
    }

    impl Printer {
        pub fn write_bytes_in_cs(bytes: &[u8], _token: LockToken<'_>) {
            if fifo_full() {
                // The FIFO is full. Let's see if we can progress.

                if TIMED_OUT.load(Ordering::Relaxed) {
                    // Still wasn't able to drain the FIFO. Let's assume we won't be able to, and
                    // don't queue up more data.
                    // This is important so we don't block forever if there is no host attached.
                    return;
                }

                // Give the fifo some time to drain.
                if !wait_for_flush() {
                    return;
                }
            } else {
                // Reset the flag - we managed to clear our FIFO.
                TIMED_OUT.store(false, Ordering::Relaxed);
            }

            for &b in bytes {
                if fifo_full() {
                    fifo_flush();

                    // Wait for the FIFO to clear, we have more data to shift out.
                    if !wait_for_flush() {
                        return;
                    }
                }
                fifo_write(b);
            }
        }

        pub fn flush(_token: LockToken<'_>) {
            fifo_flush();
        }
    }
}

#[cfg(all(any(feature = "uart", feature = "auto"), feature = "esp32"))]
mod uart_printer {
    use super::LockToken;
    const UART_TX_ONE_CHAR: usize = 0x4000_9200;

    pub struct Printer;
    impl Printer {
        pub fn write_bytes_in_cs(bytes: &[u8], _token: LockToken<'_>) {
            for &b in bytes {
                unsafe {
                    let uart_tx_one_char: unsafe extern "C" fn(u8) -> i32 =
                        core::mem::transmute(UART_TX_ONE_CHAR);
                    uart_tx_one_char(b)
                };
            }
        }

        pub fn flush(_token: LockToken<'_>) {}
    }
}

#[cfg(all(any(feature = "uart", feature = "auto"), feature = "esp32s2"))]
mod uart_printer {
    use super::LockToken;
    pub struct Printer;
    impl Printer {
        pub fn write_bytes_in_cs(bytes: &[u8], _token: LockToken<'_>) {
            // On ESP32-S2 the UART_TX_ONE_CHAR ROM-function seems to have some issues.
            for chunk in bytes.chunks(64) {
                for &b in chunk {
                    unsafe {
                        // write FIFO
                        (0x3f400000 as *mut u32).write_volatile(b as u32);
                    };
                }

                // wait for TX_DONE
                while unsafe { (0x3f400004 as *const u32).read_volatile() } & (1 << 14) == 0 {}
                unsafe {
                    // reset TX_DONE
                    (0x3f400010 as *mut u32).write_volatile(1 << 14);
                }
            }
        }

        pub fn flush(_token: LockToken<'_>) {}
    }
}

#[cfg(all(
    any(feature = "uart", feature = "auto"),
    not(any(feature = "esp32", feature = "esp32s2"))
))]
mod uart_printer {
    use super::LockToken;
    trait Functions {
        const TX_ONE_CHAR: usize;
        const CHUNK_SIZE: usize = 32;

        fn tx_byte(b: u8) {
            unsafe {
                let tx_one_char: unsafe extern "C" fn(u8) -> i32 =
                    core::mem::transmute(Self::TX_ONE_CHAR);
                tx_one_char(b);
            }
        }

        fn flush();
    }

    struct Device;

    #[cfg(feature = "esp32c2")]
    impl Functions for Device {
        const TX_ONE_CHAR: usize = 0x4000_005C;

        fn flush() {
            // tx_one_char waits for empty
        }
    }

    #[cfg(feature = "esp32c3")]
    impl Functions for Device {
        const TX_ONE_CHAR: usize = 0x4000_0068;

        fn flush() {
            unsafe {
                const TX_FLUSH: usize = 0x4000_0080;
                const GET_CHANNEL: usize = 0x4000_058C;
                let tx_flush: unsafe extern "C" fn(u8) = core::mem::transmute(TX_FLUSH);
                let get_channel: unsafe extern "C" fn() -> u8 = core::mem::transmute(GET_CHANNEL);

                const G_USB_PRINT_ADDR: usize = 0x3FCD_FFD0;
                let g_usb_print = G_USB_PRINT_ADDR as *mut bool;

                let channel = if *g_usb_print {
                    // Flush USB-JTAG
                    3
                } else {
                    get_channel()
                };
                tx_flush(channel);
            }
        }
    }

    #[cfg(feature = "esp32s3")]
    impl Functions for Device {
        const TX_ONE_CHAR: usize = 0x4000_0648;

        fn flush() {
            unsafe {
                const TX_FLUSH: usize = 0x4000_0690;
                const GET_CHANNEL: usize = 0x4000_1A58;
                let tx_flush: unsafe extern "C" fn(u8) = core::mem::transmute(TX_FLUSH);
                let get_channel: unsafe extern "C" fn() -> u8 = core::mem::transmute(GET_CHANNEL);

                const G_USB_PRINT_ADDR: usize = 0x3FCE_FFB8;
                let g_usb_print = G_USB_PRINT_ADDR as *mut bool;

                let channel = if *g_usb_print {
                    // Flush USB-JTAG
                    4
                } else {
                    get_channel()
                };
                tx_flush(channel);
            }
        }
    }

    #[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
    impl Functions for Device {
        const TX_ONE_CHAR: usize = 0x4000_0058;

        fn flush() {
            unsafe {
                const TX_FLUSH: usize = 0x4000_0074;
                const GET_CHANNEL: usize = 0x4000_003C;

                let tx_flush: unsafe extern "C" fn(u8) = core::mem::transmute(TX_FLUSH);
                let get_channel: unsafe extern "C" fn() -> u8 = core::mem::transmute(GET_CHANNEL);

                tx_flush(get_channel());
            }
        }
    }

    #[cfg(feature = "esp32p4")]
    impl Functions for Device {
        const TX_ONE_CHAR: usize = 0x4FC0_0054;

        fn flush() {
            unsafe {
                const TX_FLUSH: usize = 0x4FC0_0074;
                const GET_CHANNEL: usize = 0x4FC0_0038;

                let tx_flush: unsafe extern "C" fn(u8) = core::mem::transmute(TX_FLUSH);
                let get_channel: unsafe extern "C" fn() -> u8 = core::mem::transmute(GET_CHANNEL);

                tx_flush(get_channel());
            }
        }
    }

    pub struct Printer;
    impl Printer {
        pub fn write_bytes_in_cs(bytes: &[u8], _token: LockToken<'_>) {
            for chunk in bytes.chunks(Device::CHUNK_SIZE) {
                for &b in chunk {
                    Device::tx_byte(b);
                }

                Device::flush();
            }
        }

        pub fn flush(_token: LockToken<'_>) {}
    }
}

#[cfg(not(feature = "critical-section"))]
use core::marker::PhantomData;

#[cfg(not(feature = "critical-section"))]
type LockInner<'a> = PhantomData<&'a ()>;
#[cfg(feature = "critical-section")]
type LockInner<'a> = critical_section::CriticalSection<'a>;

#[derive(Clone, Copy)]
struct LockToken<'a>(LockInner<'a>);

impl LockToken<'_> {
    #[allow(unused)]
    unsafe fn conjure() -> Self {
        unsafe {
            #[cfg(feature = "critical-section")]
            let inner = critical_section::CriticalSection::new();
            #[cfg(not(feature = "critical-section"))]
            let inner = PhantomData;

            LockToken(inner)
        }
    }
}

/// Runs the callback in a critical section, if enabled.
#[inline]
fn with<R>(f: impl FnOnce(LockToken) -> R) -> R {
    #[cfg(feature = "critical-section")]
    return critical_section::with(|cs| f(LockToken(cs)));

    #[cfg(not(feature = "critical-section"))]
    f(unsafe { LockToken::conjure() })
}
