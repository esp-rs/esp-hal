//! A `log` and `defmt` backend that writes to any [`embedded_io::Write`].
//!
//! The crate does not know about any particular hardware. You construct the transport (for
//! example an esp-hal `Uart` or `UsbSerialJtag`) and hand it to [`init`]:
//!
//! ```rust,ignore
//! let peripherals = esp_hal::init(esp_hal::Config::default());
//! let usj = UsbSerialJtag::new(peripherals.USB_DEVICE, Default::default()).unwrap();
//! static WRITER: StaticCell<UsbSerialJtag<'static, Blocking>> = StaticCell::new();
//! embedded_log::init(WRITER.init(usj));
//! log::info!("Hello");
//! ```
//!
//! Output produced before [`init`] is dropped.
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
        fn _embedded_log_timestamp() -> u64;
    }
    // Safety: the `timestamp` feature requires the application to provide this function.
    unsafe { _embedded_log_timestamp() }
}
