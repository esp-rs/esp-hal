//! A `log` and `defmt` backend for Espressif chips.
//!
//! [`init_uart!`](crate::init_uart) and [`init_usb!`](crate::init_usb) install the logger on UART0
//! or USB-Serial-JTAG, with the default pins of the chip:
//!
//! ```rust,ignore
//! let peripherals = esp_hal::init(esp_hal::Config::default());
//! esp_log::init_uart!(peripherals);
//! log::info!("Hello");
//! ```
//!
//! [`init_uart_split!`](crate::init_uart_split) and [`init_usb_split!`](crate::init_usb_split) also
//! return the receiver of the port. The macros create the esp-hal drivers through the `esp_hal`
//! crate of the application. If the application renames that dependency, pass the name as the
//! second argument, for example `esp_log::init_uart!(peripherals, hal)`. The macros need the chip
//! feature of this crate, and esp-hal's `unstable` feature, because they use the unstable UART and
//! USB-Serial-JTAG drivers.
//!
//! [`init`] takes any other [`embedded_io::Write`], for example a UART on other pins:
//!
//! ```rust,ignore
//! let uart = UartTx::new(peripherals.UART1, Default::default())
//!     .unwrap()
//!     .with_tx(peripherals.GPIO5);
//! static WRITER: StaticCell<UartTx<'static, Blocking>> = StaticCell::new();
//! esp_log::init(WRITER.init(uart));
//! ```
//!
//! Output produced before the logger is installed is dropped.
//!
//! ## Filtering
//!
//! With the `log-04` feature, [`init`] filters records at build time from the `ESP_LOG`
//! environment variable, similar to `RUST_LOG` (no regular expressions), for example
//! `warn,my_crate::foo=info,my_crate::foo::bar=debug`. Without `ESP_LOG` nothing is logged.
//! [`init_with_level`] ignores `ESP_LOG` and uses a single maximum level instead.
//!
//! ## Panics
//!
//! Every write runs in a critical section. If a panic (or another fatal exception) interrupts a
//! write on the same core, the logger hands the writer to the panic report instead of blocking or
//! dropping the report, and keeps accepting output afterwards. A panic on another core waits until
//! the interrupted write finishes.
//!
//! ## Feature Flags
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
#![cfg_attr(not(test), no_std)]

use core::cell::UnsafeCell;

#[cfg(feature = "defmt")]
mod defmt;
#[cfg(any(feature = "defmt", test))]
mod frame;
#[cfg(feature = "log-04")]
mod logger;
mod macros;
#[cfg(test)]
mod tests;

#[cfg(feature = "log-04")]
pub use logger::{init, init_with_level};

/// Installs `writer` as the output of the `defmt` global logger.
///
/// Calling this again replaces the writer.
#[cfg(feature = "defmt")]
pub fn init<W>(writer: &'static mut W)
where
    W: embedded_io::Write + Send,
{
    set_writer(writer);
}

macro_rules! log_format {
    ($value:expr) => {
        #[cfg(target_os = "none")]
        #[unsafe(link_section = ".espressif.metadata")]
        #[used]
        #[unsafe(export_name = "espflash.LOG_FORMAT")]
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

#[cfg(feature = "defmt")]
log_format!("defmt-espflash");

#[cfg(not(feature = "defmt"))]
log_format!("serial");

/// An [`embedded_io::Write`] with the error type erased.
trait ErasedWrite {
    fn write_bytes(&mut self, bytes: &[u8]);
    fn flush_bytes(&mut self);
}

impl<W: embedded_io::Write> ErasedWrite for W {
    fn write_bytes(&mut self, bytes: &[u8]) {
        // There is nowhere to report a failed log write.
        _ = self.write_all(bytes);
    }

    fn flush_bytes(&mut self) {
        _ = self.flush();
    }
}

struct State {
    writer: Option<*mut dyn ErasedWrite>,
    /// A write is in progress.
    taken: bool,
    /// A write was interrupted on the same core, and the writer has been taken over.
    taken_over: bool,
}

struct GlobalState(UnsafeCell<State>);

// Safety: `State` is only accessed inside a critical section.
unsafe impl Sync for GlobalState {}

static STATE: GlobalState = GlobalState(UnsafeCell::new(State {
    writer: None,
    taken: false,
    taken_over: false,
}));

fn set_writer<W>(writer: &'static mut W)
where
    W: embedded_io::Write + Send,
{
    let writer: &'static mut dyn ErasedWrite = writer;
    critical_section::with(|_| {
        // Safety: we are in a critical section.
        unsafe { (*STATE.0.get()).writer = Some(writer) };
    });
}

/// The result of [`acquire`].
struct Access {
    writer: *mut dyn ErasedWrite,
    /// This access interrupted another one on the same core.
    reentered: bool,
    /// This is the first access after an interrupted one.
    takeover: bool,
}

/// Marks the writer as taken. Must be called in a critical section, paired with [`release`].
///
/// Interrupts are disabled in the critical section, so finding the writer already taken means a
/// panic or exception interrupted a write on this core. That write never resumes, so the writer is
/// handed over and stays taken from then on.
///
/// # Safety
///
/// Must be called in a critical section that lasts until the matching [`release`].
unsafe fn acquire() -> Option<Access> {
    // Safety: the caller holds a critical section.
    let state = unsafe { &mut *STATE.0.get() };
    let writer = state.writer?;

    let reentered = state.taken;
    let takeover = reentered && !state.taken_over;
    state.taken_over |= reentered;
    state.taken = true;

    Some(Access {
        writer,
        reentered,
        takeover,
    })
}

/// # Safety
///
/// Must be called in the critical section of the matching [`acquire`].
unsafe fn release(access: &Access) {
    if !access.reentered {
        // Safety: the caller holds a critical section.
        unsafe { (*STATE.0.get()).taken = false };
    }
}

/// Runs `f` with the writer in a critical section. Does nothing before the writer is installed.
///
/// `f` receives `true` as its second argument when it takes over an interrupted write.
#[cfg(any(feature = "log-04", test))]
fn with_writer(f: impl FnOnce(&mut dyn ErasedWrite, bool)) {
    critical_section::with(|_| {
        // Safety: we are in a critical section.
        let Some(access) = (unsafe { acquire() }) else {
            return;
        };
        // Safety: the writer is `'static`, and only used in the critical section.
        f(unsafe { &mut *access.writer }, access.takeover);
        // Safety: we are in the same critical section.
        unsafe { release(&access) };
    });
}

#[cfg(feature = "timestamp")]
fn timestamp() -> u64 {
    unsafe extern "Rust" {
        fn _esp_log_timestamp() -> u64;
    }
    // Safety: the `timestamp` feature requires the application to provide this function.
    unsafe { _esp_log_timestamp() }
}

#[doc(hidden)]
pub mod __private {
    use core::{cell::UnsafeCell, mem::MaybeUninit};

    /// Storage for the driver that the logger writes to. The macros declare one per call site.
    pub struct Slot<T> {
        value: UnsafeCell<MaybeUninit<T>>,
        used: UnsafeCell<bool>,
    }

    // Safety: `used` is only accessed in a critical section, and `value` is written once.
    unsafe impl<T> Sync for Slot<T> {}

    impl<T> Slot<T> {
        #[allow(clippy::new_without_default)]
        pub const fn new() -> Self {
            Self {
                value: UnsafeCell::new(MaybeUninit::uninit()),
                used: UnsafeCell::new(false),
            }
        }

        /// Stores `writer` and installs it as the output of the logger.
        pub fn install(&'static self, writer: T)
        where
            T: embedded_io::Write + Send,
        {
            let used = critical_section::with(|_| {
                // Safety: we are in a critical section.
                unsafe { self.used.get().replace(true) }
            });
            assert!(!used, "the logger is already installed by this call");
            // Safety: `used` makes this the only reference to `value`.
            crate::init(unsafe { (*self.value.get()).write(writer) });
        }
    }
}
