#![doc = include_str!("../README.md")]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(rustdoc::bare_urls)]
#![no_std]

#[cfg(feature = "defmt-espflash")]
pub mod defmt;
#[cfg(feature = "log")]
pub mod logger;

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

#[cfg(feature = "no-op")]
#[macro_export]
macro_rules! println {
    ($($arg:tt)*) => {{}};
}

#[cfg(feature = "no-op")]
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{}};
}

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
    ($val:expr $(,)?) => {
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
    ($($val:expr),+ $(,)?) => {
        ($($crate::dbg!($val)),+,)
    };
}

pub struct Printer;

impl core::fmt::Write for Printer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Printer.write_bytes(s.as_bytes());
        Ok(())
    }
}

impl Printer {
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        with(|| {
            self.write_bytes_assume_cs(bytes);
            self.flush();
        })
    }
}

#[cfg(all(
    feature = "jtag-serial",
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

    impl super::Printer {
        pub fn write_bytes_assume_cs(&mut self, bytes: &[u8]) {
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

        pub fn flush(&mut self) {
            fifo_flush();
        }
    }
}

#[cfg(all(feature = "uart", feature = "esp32"))]
mod uart_printer {
    const UART_TX_ONE_CHAR: usize = 0x4000_9200;
    impl super::Printer {
        pub fn write_bytes_assume_cs(&mut self, bytes: &[u8]) {
            for &b in bytes {
                unsafe {
                    let uart_tx_one_char: unsafe extern "C" fn(u8) -> i32 =
                        core::mem::transmute(UART_TX_ONE_CHAR);
                    uart_tx_one_char(b)
                };
            }
        }

        pub fn flush(&mut self) {}
    }
}

#[cfg(all(feature = "uart", feature = "esp32s2"))]
mod uart_printer {
    const UART_TX_ONE_CHAR: usize = 0x4000_9200;
    impl super::Printer {
        pub fn write_bytes_assume_cs(&mut self, bytes: &[u8]) {
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

        pub fn flush(&mut self) {}
    }
}

#[cfg(all(feature = "uart", not(any(feature = "esp32", feature = "esp32s2"))))]
mod uart_printer {
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

    impl super::Printer {
        pub fn write_bytes_assume_cs(&mut self, bytes: &[u8]) {
            for chunk in bytes.chunks(Device::CHUNK_SIZE) {
                for &b in chunk {
                    Device::tx_byte(b);
                }

                Device::flush();
            }
        }

        pub fn flush(&mut self) {}
    }
}

#[inline]
fn with<R>(f: impl FnOnce() -> R) -> R {
    #[cfg(feature = "critical-section")]
    return critical_section::with(|_| f());

    #[cfg(not(feature = "critical-section"))]
    f()
}
