//! MD5 Message-Digest Algorithm 

use core::mem::MaybeUninit;

#[allow(unused)]
use core::ffi::{c_int, c_void};

// If there is not exactly one of the MD5 variations defined in the device
// toml file then `InternalContext` will be either undefined or multiple
// defined and this module will fail to compile letting you know to fix it

#[cfg(rom_md5_bsd)]
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
#[repr(C)]
struct InternalContext {
    total: [u32; 2],
    state: [u32; 4],
    buffer: [core::ffi::c_uchar; 64],
}

#[cfg(rom_md5_mbedtls)]
extern "C" {
    fn esp_rom_mbedtls_md5_starts_ret(context: *mut InternalContext) -> c_int;
    fn esp_rom_mbedtls_md5_update_ret(context: *mut InternalContext, buf: *const c_void, len: u32);
    fn esp_rom_mbedtls_md5_finish_ret(context: *mut InternalContext, digest: *mut u8);
}

pub struct Context(InternalContext);

pub struct Digest(pub [u8; 16]);

impl Context {

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

    #[inline]
    pub fn consume<T: AsRef<[u8]>>(&mut self, data: T) {
        let data = data.as_ref();
        unsafe {
            #[cfg(rom_md5_bsd)]
            esp_rom_md5_update(&mut self.0 as *mut _, data.as_ptr() as *const c_void, data.len() as u32);

            #[cfg(rom_md5_mbedtls)]
            let _ = esp_rom_mbedtls_md5_update_ret(&mut self.0 as *mut _, data.as_ptr() as *const c_void, data.len() as u32);
        }
    }

    #[inline]
    pub fn compute(mut self) -> Digest {
        let mut digest = MaybeUninit::<[u8; 16]>::uninit();

        unsafe {
            #[cfg(rom_md5_bsd)]
            esp_rom_md5_final(digest.as_mut_ptr() as *mut _, &mut self.0 as *mut _);

            #[cfg(rom_md5_mbedtls)]
            let _ = esp_rom_mbedtls_md5_finish_ret(&mut self.0 as *mut _, digest.as_mut_ptr() as *mut _);

            Digest(digest.assume_init())
        }
    }
}
