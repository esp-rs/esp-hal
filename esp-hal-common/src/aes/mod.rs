//! Advanced Encryption Standard (AES) support.
//!
//! ## Overview
//! The AES module provides an interface to interact with the AES peripheral, provides encryption 
//! and decryption capabilities for ESP chips using the AES algorithm. We currently support following
//! AES encryption modes:
//! * AES-128
//! * AES-192
//! * AES-256
//!
//! ## Example
//! ### Initialisation
//! ```no_run
//! let mut aes = Aes::new(peripherals.AES, &mut system.peripheral_clock_control);
//! ```
//! 
//! ### Creating key and block Buffer
//! ```no_run
//! let keytext = "SUp4SeCp@sSw0rd".as_bytes();
//! let plaintext = "message".as_bytes();
//!
//! // create an array with aes128 key size
//! let mut keybuf = [0_u8; 16];
//! keybuf[..keytext.len()].copy_from_slice(keytext);
//!
//! // create an array with aes block size
//! let mut block_buf = [0_u8; 16];
//! block_buf[..plaintext.len()].copy_from_slice(plaintext);
//! ```
//! 
//! ### Encrypting and Decrypting (using hardware)
//! ```no_run
//! let key = Key::<Aes128>::from(&keybuf);
//!
//! let mut cipher = Cipher::new(&mut aes, &key);
//! let mut block = block_buf.clone();
//! cipher.encrypt_block(&mut block);
//! 
//! let hw_encrypted = block.clone();
//! cipher.decrypt_block(&mut block);
//! let hw_decrypted = block;
//! ```
//!
//! ### Encrypting and Decrypting (using software)
//! ```no_run
//! let key = GenericArray::from(keybuf);
//! 
//! let mut block = GenericArray::from(block_buf);
//! let cipher = Aes128SW::new(&key);
//! cipher.encrypt_block(&mut block);
//!
//! let sw_encrypted = block.clone();
//! cipher.decrypt_block(&mut block);
//!
//! let sw_decrypted = block;
//! ```
//! 
//! ### Implementation State
//! * DMA mode is currently not supported. ⚠️

use core::marker::PhantomData;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{
        generic::{Readable, Reg, RegisterSpec, Resettable, Writable},
        AES,
    },
    system::PeripheralClockControl,
};

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32c3, path = "esp32cX.rs")]
#[cfg_attr(esp32c6, path = "esp32cX.rs")]
#[cfg_attr(esp32h2, path = "esp32cX.rs")]
mod aes_spec_impl;

const ALIGN_SIZE: usize = core::mem::size_of::<u32>();

/// AES peripheral container
pub struct Aes<'d> {
    aes: PeripheralRef<'d, AES>,
}

impl<'d> Aes<'d> {
    pub fn new(
        aes: impl Peripheral<P = AES> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(aes);
        let mut ret = Self { aes: aes };
        ret.init(peripheral_clock_control);
        ret
    }

    fn write_to_regset<T>(input: &[u8], n_offset: usize, reg_0: &mut Reg<T>)
    where
        T: RegisterSpec<Ux = u32> + Resettable + Writable,
    {
        let chunks = input.chunks_exact(ALIGN_SIZE);
        for (offset, chunk) in (0..n_offset).zip(chunks) {
            let to_write = u32::from_ne_bytes(chunk.try_into().unwrap());
            unsafe {
                let p = reg_0.as_ptr().add(offset);
                p.write_volatile(to_write);
            }
        }
    }

    fn read_from_regset<T>(out_buf: &mut [u8], n_offset: usize, reg_0: &Reg<T>)
    where
        T: RegisterSpec<Ux = u32> + Readable,
    {
        let chunks = out_buf.chunks_exact_mut(ALIGN_SIZE);
        for (offset, chunk) in (0..n_offset).zip(chunks) {
            unsafe {
                let p = reg_0.as_ptr().add(offset);
                let read_val: [u8; ALIGN_SIZE] = p.read_volatile().to_ne_bytes();
                chunk.copy_from_slice(&read_val);
            }
        }
    }

    fn write_to_register<T>(reg: &mut Reg<T>, data: u32)
    where
        T: RegisterSpec<Ux = u32> + Resettable + Writable,
    {
        reg.write(|w| unsafe { w.bits(data) });
    }
}

mod sealed {
    /// Specifications for AES flavours
    pub trait AesFlavour {
        type KeyType<'b>;
        const ENCRYPT_MODE: u32;
        const DECRYPT_MODE: u32;
    }
}

use sealed::AesFlavour;

/// Marker type for AES-128
pub struct Aes128;

/// Marker type for AES-192
#[cfg(any(esp32, esp32s2))]
pub struct Aes192;

/// Marker type for AES-256
pub struct Aes256;

/// Block cipher
pub struct Cipher<'a, 'd, T: AesFlavour> {
    aes: &'a mut Aes<'d>,
    phantom: PhantomData<T>,
}

impl<'a, 'd, T: AesFlavour> Cipher<'a, 'd, T> {
    /// Creates and returns a new cipher
    pub fn new(aes: &'a mut Aes<'d>, key: &Key<T>) -> Self {
        aes.write_key(key.key);
        Self {
            aes,
            phantom: PhantomData,
        }
    }
    /// Encrypts the given buffer
    pub fn encrypt_block(&mut self, block: &mut [u8; 16]) {
        self.set_mode(T::ENCRYPT_MODE);
        self.set_block(block);
        self.start();
        while !(self.is_idle()) {}
        self.get_block(block);
    }

    /// Decrypts the given buffer
    pub fn decrypt_block(&mut self, block: &mut [u8; 16]) {
        self.set_mode(T::DECRYPT_MODE);
        self.set_block(block);
        self.start();
        while !(self.is_idle()) {}
        self.get_block(block);
    }

    fn set_mode(&mut self, mode: u32) {
        self.aes.write_mode(mode);
    }

    fn is_idle(&mut self) -> bool {
        self.aes.read_idle()
    }

    fn set_block(&mut self, block: &[u8; 16]) {
        self.aes.write_block(block);
    }

    fn get_block(&self, block: &mut [u8; 16]) {
        self.aes.read_block(block);
    }

    fn start(&mut self) {
        self.aes.write_start();
    }
}

/// Aes cipher key
///
/// A `Key` can be initialized from an array of appropriate length:
///
/// ``` plain
/// let key = Key::<Aes128>::from(&[0_u8;16]);
/// let key = Key::<Aes192>::from(&[0_u8;24]);
/// let key = Key::<Aes256>::from(&[0_u8;32]);
/// ```
pub struct Key<'b, T: AesFlavour> {
    key: &'b [u8],
    phantom: PhantomData<T>,
}

impl<'b, T, const N: usize> From<&'b [u8; N]> for Key<'b, T>
where
    T: AesFlavour<KeyType<'b> = &'b [u8; N]>,
{
    fn from(value: T::KeyType<'b>) -> Self {
        Key {
            key: value,
            phantom: PhantomData,
        }
    }
}
/// State matrix endianness
#[cfg(any(esp32, esp32s2))]
pub enum Endianness {
    BigEndian    = 1,
    LittleEndian = 0,
}
