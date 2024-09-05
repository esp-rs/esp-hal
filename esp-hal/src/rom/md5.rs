//! # MD5 Message-Digest Algorithm (MD5)
//!
//! ## ⚠️ Security Warning ⚠️
//!
//! MD5 is a **cryptographically broken** message digest. It should **not**
//! be used for data security, for example, to check if data has been
//! intentionally tampered with.
//!
//! However, it is still very useful for purposes of data **integrity**, for
//! example, to check if data has been **accidentally** tampered with, such as
//! detecting data corrupted during transmission in a stream or stored in
//! flash. This is especially important on microcontrollers where the
//! computational efficiency of MD5 is desired.
//!
//! ## Compatibility
//!
//! The public API exposed by this module tries to *mimic* the public API
//! offered by the [MD5][1] crate in an effort to act as a drop-in replacement.
//! The actual implementation, however, links to whatever is available in the
//! ROM of the particular ESP32 target. Each chip target may offer a different
//! underlying MD5 implementation with varying functionality.
//!
//! This module offers a least-common-denominator API to stay consistent with
//! all of them. Usage of this module may help make program binaries smaller
//! than it would be if you included an MD5 implementation in your project.
//!
//! ## Examples
//!
//! ### Compute a Full Digest From a Single Buffer
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::rom::md5;
//! # use esp_hal::uart::Uart;
//! # use esp_hal::gpio::Io;
//! # use core::writeln;
//! # use core::fmt::Write;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let mut uart0 = Uart::new(peripherals.UART0, io.pins.gpio1, io.pins.gpio2).unwrap();
//! # let data = "Dummy";
//! let d: md5::Digest = md5::compute(&data);
//! writeln!(uart0, "{}", d);
//! # }
//! ```
//! 
//! ### Compute a Digest Over Multiple Buffers
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::rom::md5;
//! # use esp_hal::uart::Uart;
//! # use esp_hal::gpio::Io;
//! # use core::writeln;
//! # use core::fmt::Write;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! # let mut uart0 = Uart::new(peripherals.UART0, io.pins.gpio1, io.pins.gpio2).unwrap();
//! # let data0 = "Dummy";
//! # let data1 = "Dummy";
//! #
//! let mut ctx = md5::Context::new();
//! ctx.consume(&data0);
//! ctx.consume(&data1);
//! let d: md5::Digest = ctx.compute();
//! writeln!(uart0, "{}", d);
//! # }
//! ```
//! 
//! [1]: <https://crates.io/crates/md5>

#[allow(unused)]
use core::ffi::{c_int, c_uchar, c_void};
use core::{
    fmt,
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
};

// If there is not exactly one of the MD5 variations defined in the device
// toml file then `InternalContext` will be either undefined or multiple
// defined and this module will fail to compile letting you know to fix it

#[cfg(rom_md5_bsd)]
#[derive(Clone)]
#[repr(C)]
struct InternalContext {
    buf: [u32; 4],
    bits: [u32; 2],
    _in: [u8; 64],
}

#[cfg(rom_md5_bsd)]
extern "C" {
    fn esp_rom_md5_init(context: *mut InternalContext);
    fn esp_rom_md5_update(context: *mut InternalContext, buf: *const c_void, len: u32);
    fn esp_rom_md5_final(digest: *mut u8, context: *mut InternalContext);
}

#[cfg(rom_md5_mbedtls)]
#[derive(Clone)]
#[repr(C)]
struct InternalContext {
    total: [u32; 2],
    state: [u32; 4],
    buffer: [c_uchar; 64],
}

#[cfg(rom_md5_mbedtls)]
extern "C" {
    fn esp_rom_mbedtls_md5_starts_ret(context: *mut InternalContext) -> c_int;
    fn esp_rom_mbedtls_md5_update_ret(
        context: *mut InternalContext,
        buf: *const c_void,
        len: u32,
    ) -> c_int;
    fn esp_rom_mbedtls_md5_finish_ret(context: *mut InternalContext, digest: *mut u8) -> c_int;
}

/// MD5 context for an ongoing computation
#[derive(Clone)]
pub struct Context(InternalContext);

impl Context {
    /// Create a new MD5 context
    #[inline]
    pub fn new() -> Self {
        let mut ctx = MaybeUninit::<InternalContext>::uninit();
        unsafe {
            #[cfg(rom_md5_bsd)]
            esp_rom_md5_init(ctx.as_mut_ptr());

            #[cfg(rom_md5_mbedtls)]
            let _ = esp_rom_mbedtls_md5_starts_ret(ctx.as_mut_ptr());

            Self(ctx.assume_init())
        }
    }

    /// Feed data to the hasher
    #[inline]
    pub fn consume<T: AsRef<[u8]>>(&mut self, data: T) {
        let data = data.as_ref();
        unsafe {
            #[cfg(rom_md5_bsd)]
            esp_rom_md5_update(
                &mut self.0 as *mut _,
                data.as_ptr() as *const c_void,
                data.len() as u32,
            );

            #[cfg(rom_md5_mbedtls)]
            let _ = esp_rom_mbedtls_md5_update_ret(
                &mut self.0 as *mut _,
                data.as_ptr() as *const c_void,
                data.len() as u32,
            );
        }
    }

    /// Finalize and return a digest
    #[inline]
    pub fn compute(mut self) -> Digest {
        let mut digest = MaybeUninit::<[u8; 16]>::uninit();
        unsafe {
            #[cfg(rom_md5_bsd)]
            esp_rom_md5_final(digest.as_mut_ptr() as *mut _, &mut self.0 as *mut _);

            #[cfg(rom_md5_mbedtls)]
            let _ = esp_rom_mbedtls_md5_finish_ret(
                &mut self.0 as *mut _,
                digest.as_mut_ptr() as *mut _,
            );

            Digest(digest.assume_init())
        }
    }
}

impl Default for Context {
    fn default() -> Self {
        Self::new()
    }
}

/// Compute a full digest from a single buffer
#[inline]
pub fn compute<T: AsRef<[u8]>>(data: T) -> Digest {
    let mut ctx = Context::new();
    ctx.consume(data);
    ctx.compute()
}

/// 16-byte message digest
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct Digest(pub [u8; 16]);

impl fmt::LowerHex for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for &byte in &self.0 {
            write!(f, "{:02x}", byte)?;
        }
        Ok(())
    }
}

impl fmt::UpperHex for Digest {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for &byte in &self.0 {
            write!(f, "{:02X}", byte)?;
        }
        Ok(())
    }
}

impl fmt::Display for Digest {
    #[inline]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        <Digest as fmt::LowerHex>::fmt(self, f)
    }
}

impl fmt::Debug for Digest {
    #[inline]
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        <Digest as fmt::LowerHex>::fmt(self, f)
    }
}

impl From<Digest> for [u8; 16] {
    #[inline]
    fn from(digest: Digest) -> Self {
        digest.0
    }
}

impl From<Context> for Digest {
    #[inline]
    fn from(context: Context) -> Digest {
        context.compute()
    }
}

impl Deref for Digest {
    type Target = [u8; 16];

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for Digest {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
