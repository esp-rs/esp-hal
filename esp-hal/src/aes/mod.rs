//! # Advanced Encryption Standard (AES) support.
//!
//! ## Overview
//!
//! The AES module provides an interface to interact with the AES peripheral,
//! provides encryption and decryption capabilities for ESP chips using the AES
//! algorithm. We currently support the following AES encryption modes:
//!
//! * AES-128
//! * AES-192
//! * AES-256
//!
//! ## Example
//!
//! ### Initialization
//!
//! ```no_run
//! let mut aes = Aes::new(peripherals.AES);
//! ```
//!
//! ### Creating key and block Buffer
//!
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
//!
//! ```no_run
//! let mut block = block_buf.clone();
//! aes.process(&mut block, Mode::Encryption128, keybuf);
//! let hw_encrypted = block.clone();
//!
//! aes.process(&mut block, Mode::Decryption128, keybuf);
//! let hw_decrypted = block;
//! ```
//!
//! ### Encrypting and Decrypting (using software)
//!
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
//!
//! * DMA mode is currently not supported on ESP32 and ESP32S2 ⚠️
//!
//! ## DMA-AES Mode
//!
//! Supports 6 block cipher modes including `ECB/CBC/OFB/CTR/CFB8/CFB128`.
//!
//! * Initialization vector (IV) is currently not supported ⚠️
//!
//! ## Example
//!
//! ### Initialization
//!
//! ```no_run
//! let dma = Gdma::new(peripherals.DMA);
//! let dma_channel = dma.channel0;
//!
//! let mut descriptors = [0u32; 8 * 3];
//! let mut rx_descriptors = [0u32; 8 * 3];
//!
//! let aes = Aes::new(peripherals.AES).with_dma(dma_channel.configure(
//!     false,
//!     &mut descriptors,
//!     &mut rx_descriptors,
//!     DmaPriority::Priority0,
//! ));
//! ```
//!
//! ### Operation
//!
//! ```no_run
//! let transfer = aes
//!     .process(
//!         plaintext,
//!         hw_encrypted,
//!         Mode::Encryption128,
//!         CipherMode::Ecb,
//!         keybuf,
//!     )
//!     .unwrap();
//! transfer.wait().unwrap();
//! ```

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::AES,
    reg_access::{AlignmentHelper, NativeEndianess},
};

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32c3, path = "esp32cX.rs")]
#[cfg_attr(esp32c6, path = "esp32cX.rs")]
#[cfg_attr(esp32h2, path = "esp32cX.rs")]
mod aes_spec_impl;

const ALIGN_SIZE: usize = core::mem::size_of::<u32>();

/// Represents the various key sizes allowed for AES encryption and decryption.
pub enum Key {
    /// 128-bit AES key
    Key16([u8; 16]),
    /// 192-bit AES key
    #[cfg(any(esp32, esp32s2))]
    Key24([u8; 24]),
    /// 256-bit AES key
    Key32([u8; 32]),
}

// Implementing From for easy conversion from array to Key enum.
impl From<[u8; 16]> for Key {
    fn from(key: [u8; 16]) -> Self {
        Key::Key16(key)
    }
}

#[cfg(any(esp32, esp32s2))]
impl From<[u8; 24]> for Key {
    fn from(key: [u8; 24]) -> Self {
        Key::Key24(key)
    }
}

impl From<[u8; 32]> for Key {
    fn from(key: [u8; 32]) -> Self {
        Key::Key32(key)
    }
}

impl Key {
    /// Returns a slice representation of the AES key.
    fn as_slice(&self) -> &[u8] {
        match self {
            Key::Key16(ref key) => key,
            #[cfg(any(esp32, esp32s2))]
            Key::Key24(ref key) => key,
            Key::Key32(ref key) => key,
        }
    }
}

/// Defines the operating modes for AES encryption and decryption.
pub enum Mode {
    /// Encryption mode with 128-bit key
    Encryption128 = 0,
    /// Encryption mode with 192-bit key
    #[cfg(any(esp32, esp32s2))]
    Encryption192 = 1,
    /// Encryption mode with 256-bit key
    Encryption256 = 2,
    /// Decryption mode with 128-bit key
    Decryption128 = 4,
    /// Decryption mode with 192-bit key
    #[cfg(any(esp32, esp32s2))]
    Decryption192 = 5,
    /// Decryption mode with 256-bit key
    Decryption256 = 6,
}

/// AES peripheral container
pub struct Aes<'d> {
    aes: PeripheralRef<'d, AES>,
    alignment_helper: AlignmentHelper<NativeEndianess>,
}

impl<'d> Aes<'d> {
    /// Constructs a new `Aes` instance.
    pub fn new(aes: impl Peripheral<P = AES> + 'd) -> Self {
        crate::into_ref!(aes);
        let mut ret = Self {
            aes,
            alignment_helper: AlignmentHelper::native_endianess(),
        };
        ret.init();

        ret
    }

    /// Encrypts/Decrypts the given buffer based on `mode` parameter
    pub fn process<K>(&mut self, block: &mut [u8; 16], mode: Mode, key: K)
    where
        K: Into<Key>,
    {
        // Convert from into Key enum
        self.write_key(key.into().as_slice());
        self.set_mode(mode as u8);
        self.set_block(block);
        self.start();
        while !(self.is_idle()) {}
        self.get_block(block);
    }

    fn set_mode(&mut self, mode: u8) {
        self.write_mode(mode as u32);
    }

    fn is_idle(&mut self) -> bool {
        self.read_idle()
    }

    fn set_block(&mut self, block: &[u8; 16]) {
        self.write_block(block);
    }

    fn get_block(&self, block: &mut [u8; 16]) {
        self.read_block(block);
    }

    fn start(&mut self) {
        self.write_start();
    }
}

/// Specifications for AES flavours
pub trait AesFlavour: crate::private::Sealed {
    type KeyType<'b>;
    const ENCRYPT_MODE: u32;
    const DECRYPT_MODE: u32;
}

/// Marker type for AES-128
pub struct Aes128;

/// Marker type for AES-192
#[cfg(any(esp32, esp32s2))]
pub struct Aes192;

/// Marker type for AES-256
pub struct Aes256;

impl crate::private::Sealed for Aes128 {}
#[cfg(any(esp32, esp32s2))]
impl crate::private::Sealed for Aes192 {}
impl crate::private::Sealed for Aes256 {}

/// State matrix endianness
#[cfg(any(esp32, esp32s2))]
pub enum Endianness {
    BigEndian    = 1,
    LittleEndian = 0,
}

/// Provides DMA (Direct Memory Access) support for AES operations.
///
/// This module enhances the AES capabilities by utilizing DMA to handle data
/// transfer, which can significantly speed up operations when dealing with
/// large data volumes. It supports various cipher modes such as ECB, CBC, OFB,
/// CTR, CFB8, and CFB128.
#[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
pub mod dma {
    use embedded_dma::{ReadBuffer, WriteBuffer};

    use crate::{
        aes::{Key, Mode},
        dma::{
            dma_private::{DmaSupport, DmaSupportRx, DmaSupportTx},
            AesPeripheral,
            Channel,
            ChannelTypes,
            DmaPeripheral,
            DmaTransferTxRx,
            RxPrivate,
            TxPrivate,
        },
    };

    const ALIGN_SIZE: usize = core::mem::size_of::<u32>();

    /// Specifies the block cipher modes available for AES operations.
    pub enum CipherMode {
        /// Electronic Codebook Mode
        Ecb = 0,
        /// Cipher Block Chaining Mode
        Cbc,
        /// Output Feedback Mode
        Ofb,
        /// Counter Mode.
        Ctr,
        /// Cipher Feedback Mode with 8-bit shifting.
        Cfb8,
        /// Cipher Feedback Mode with 128-bit shifting.
        Cfb128,
    }

    /// A DMA capable AES instance.
    pub struct AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        pub aes: super::Aes<'d>,

        pub(crate) channel: Channel<'d, C, crate::Blocking>,
    }

    pub trait WithDmaAes<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        fn with_dma(self, channel: Channel<'d, C, crate::Blocking>) -> AesDma<'d, C>;
    }

    impl<'d, C> WithDmaAes<'d, C> for crate::aes::Aes<'d>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        fn with_dma(self, mut channel: Channel<'d, C, crate::Blocking>) -> AesDma<'d, C> {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            AesDma { aes: self, channel }
        }
    }

    impl<'d, C> core::fmt::Debug for AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("AesDma").finish()
        }
    }

    impl<'d, C> DmaSupport for AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        fn peripheral_wait_dma(&mut self, _is_tx: bool, _is_rx: bool) {
            while self.aes.aes.state().read().state().bits() != 2 // DMA status DONE == 2
            && !self.channel.tx.is_done()
            {
                // wait until done
            }

            self.finish_transform();
        }

        fn peripheral_dma_stop(&mut self) {
            unreachable!("unsupported")
        }
    }

    impl<'d, C> DmaSupportTx for AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        type TX = C::Tx<'d>;

        fn tx(&mut self) -> &mut Self::TX {
            &mut self.channel.tx
        }
    }

    impl<'d, C> DmaSupportRx for AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        type RX = C::Rx<'d>;

        fn rx(&mut self) -> &mut Self::RX {
            &mut self.channel.rx
        }
    }

    impl<'d, C> AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        /// Writes the encryption key to the AES hardware, checking that its
        /// length matches expected constraints.
        pub fn write_key<K>(&mut self, key: K)
        where
            K: Into<Key>,
        {
            let key = key.into(); // Convert into Key enum
            debug_assert!(key.as_slice().len() <= 8 * ALIGN_SIZE);
            debug_assert_eq!(key.as_slice().len() % ALIGN_SIZE, 0);
            self.aes.write_key(key.as_slice());
        }

        /// Writes a block of data to the AES hardware, ensuring the block's
        /// length is properly aligned.
        pub fn write_block(&mut self, block: &[u8]) {
            debug_assert_eq!(block.len(), 4 * ALIGN_SIZE);
            self.aes.write_key(block);
        }

        /// Perform a DMA transfer.
        ///
        /// This will return a [AesDmaTransfer] owning the buffer(s) and the
        /// AES instance. The maximum amount of data to be sent/received
        /// is 32736 bytes.
        pub fn process<'t, K, TXBUF, RXBUF>(
            &'t mut self,
            words: &'t TXBUF,
            read_buffer: &'t mut RXBUF,
            mode: Mode,
            cipher_mode: CipherMode,
            key: K,
        ) -> Result<DmaTransferTxRx<Self>, crate::dma::DmaError>
        where
            K: Into<Key>,
            TXBUF: ReadBuffer<Word = u8>,
            RXBUF: WriteBuffer<Word = u8>,
        {
            let (write_ptr, write_len) = unsafe { words.read_buffer() };
            let (read_ptr, read_len) = unsafe { read_buffer.write_buffer() };

            self.start_transfer_dma(
                write_ptr,
                write_len,
                read_ptr,
                read_len,
                mode,
                cipher_mode,
                key.into(),
            )?;

            Ok(DmaTransferTxRx::new(self))
        }

        #[allow(clippy::too_many_arguments)]
        fn start_transfer_dma<K>(
            &mut self,
            write_buffer_ptr: *const u8,
            write_buffer_len: usize,
            read_buffer_ptr: *mut u8,
            read_buffer_len: usize,
            mode: Mode,
            cipher_mode: CipherMode,
            key: K,
        ) -> Result<(), crate::dma::DmaError>
        where
            K: Into<Key>,
        {
            // AES has to be restarted after each calculation
            self.reset_aes();

            self.channel.tx.is_done();
            self.channel.rx.is_done();

            self.channel
                .tx
                .prepare_transfer_without_start(
                    self.dma_peripheral(),
                    false,
                    write_buffer_ptr,
                    write_buffer_len,
                )
                .and_then(|_| self.channel.tx.start_transfer())?;
            unsafe {
                self.channel
                    .rx
                    .prepare_transfer_without_start(
                        false,
                        self.dma_peripheral(),
                        read_buffer_ptr,
                        read_buffer_len,
                    )
                    .and_then(|_| self.channel.rx.start_transfer())?;
            }
            self.enable_dma(true);
            self.enable_interrupt();
            self.set_mode(mode);
            self.set_cipher_mode(cipher_mode);
            self.write_key(key.into());

            // TODO: verify 16?
            self.set_num_block(16);

            self.start_transform();

            Ok(())
        }

        #[cfg(any(esp32c3, esp32s3))]
        fn reset_aes(&self) {
            unsafe {
                let s = crate::peripherals::SYSTEM::steal();
                s.perip_rst_en1()
                    .modify(|_, w| w.crypto_aes_rst().set_bit());
                s.perip_rst_en1()
                    .modify(|_, w| w.crypto_aes_rst().clear_bit());
            }
        }

        #[cfg(any(esp32c6, esp32h2))]
        fn reset_aes(&self) {
            unsafe {
                let s = crate::peripherals::PCR::steal();
                s.aes_conf().modify(|_, w| w.aes_rst_en().set_bit());
                s.aes_conf().modify(|_, w| w.aes_rst_en().clear_bit());
            }
        }

        fn dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::Aes
        }

        fn enable_dma(&self, enable: bool) {
            self.aes
                .aes
                .dma_enable()
                .write(|w| w.dma_enable().bit(enable));
        }

        fn enable_interrupt(&self) {
            self.aes.aes.int_ena().write(|w| w.int_ena().set_bit());
        }

        fn set_cipher_mode(&self, mode: CipherMode) {
            self.aes
                .aes
                .block_mode()
                .modify(|_, w| unsafe { w.bits(mode as u32) });

            if self.aes.aes.block_mode().read().block_mode().bits() == CipherMode::Ctr as u8 {
                self.aes
                    .aes
                    .inc_sel()
                    .modify(|_, w| w.inc_sel().clear_bit());
            }
        }

        fn set_mode(&self, mode: Mode) {
            self.aes
                .aes
                .mode()
                .modify(|_, w| unsafe { w.mode().bits(mode as u8) });
        }

        fn start_transform(&self) {
            self.aes.aes.trigger().write(|w| w.trigger().set_bit());
        }

        fn finish_transform(&self) {
            self.aes.aes.dma_exit().write(|w| w.dma_exit().set_bit());
            self.enable_dma(false);
            self.reset_aes();
        }

        fn set_num_block(&self, block: u32) {
            self.aes
                .aes
                .block_num()
                .modify(|_, w| unsafe { w.block_num().bits(block) });
        }
    }
}
