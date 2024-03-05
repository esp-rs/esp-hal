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
//! aes.process(&mut block, Mode::Encryption128, &keybuf);
//! let hw_encrypted = block.clone();
//!
//! aes.process(&mut block, Mode::Decryption128, &keybuf);
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
//! let (hw_encrypted, plaintext, aes) = transfer.wait().unwrap();
//! ```

#[cfg(esp32)]
use crate::peripherals::generic::{Readable, Reg, RegisterSpec};
#[cfg(not(esp32))]
use crate::reg_access::AlignmentHelper;
use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::AES,
};

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32c3, path = "esp32cX.rs")]
#[cfg_attr(esp32c6, path = "esp32cX.rs")]
#[cfg_attr(esp32h2, path = "esp32cX.rs")]
mod aes_spec_impl;

const ALIGN_SIZE: usize = core::mem::size_of::<u32>();

pub enum Mode {
    Encryption128 = 0,
    Encryption256 = 2,
    Decryption128 = 4,
    Decryption256 = 6,
}

/// AES peripheral container
pub struct Aes<'d> {
    aes: PeripheralRef<'d, AES>,
    #[cfg(not(esp32))]
    alignment_helper: AlignmentHelper,
}

impl<'d> Aes<'d> {
    pub fn new(aes: impl Peripheral<P = AES> + 'd) -> Self {
        crate::into_ref!(aes);
        let mut ret = Self {
            aes,
            #[cfg(not(esp32))]
            alignment_helper: AlignmentHelper::default(),
        };
        ret.init();

        ret
    }

    /// Encrypts/Decrypts the given buffer based on `mode` parameter
    pub fn process(&mut self, block: &mut [u8; 16], mode: Mode, key: &[u8; 16]) {
        self.write_key(key);
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

    // TODO: for some reason, the `volatile read/write` helpers from `reg_access`
    // don't work for ESP32
    #[cfg(esp32)]
    fn write_to_regset(input: &[u8], n_offset: usize, reg_0: *mut u32) {
        let chunks = input.chunks_exact(ALIGN_SIZE);
        for (offset, chunk) in (0..n_offset).zip(chunks) {
            let to_write = u32::from_ne_bytes(chunk.try_into().unwrap());
            unsafe {
                let p = reg_0.add(offset);
                p.write_volatile(to_write);
            }
        }
    }

    // TODO: for some reason, the `volatile read/write` helpers from `reg_access`
    // don't work for ESP32
    #[cfg(esp32)]
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

#[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
pub mod dma {
    use embedded_dma::{ReadBuffer, WriteBuffer};

    use crate::{
        aes::Mode,
        dma::{
            AesPeripheral,
            Channel,
            ChannelTypes,
            DmaError,
            DmaPeripheral,
            DmaTransferRxTx,
            RxPrivate,
            TxPrivate,
        },
    };

    const ALIGN_SIZE: usize = core::mem::size_of::<u32>();

    pub enum CipherMode {
        Ecb = 0,
        Cbc,
        Ofb,
        Ctr,
        Cfb8,
        Cfb128,
    }

    /// A DMA capable AES instance.
    pub struct AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        pub aes: super::Aes<'d>,

        pub(crate) channel: Channel<'d, C>,
    }

    pub trait WithDmaAes<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        fn with_dma(self, channel: Channel<'d, C>) -> AesDma<'d, C>;
    }

    impl<'d, C> WithDmaAes<'d, C> for crate::aes::Aes<'d>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        fn with_dma(self, mut channel: Channel<'d, C>) -> AesDma<'d, C> {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            AesDma { aes: self, channel }
        }
    }

    /// An in-progress DMA transfer
    #[must_use]
    pub struct AesDmaTransferRxTx<'t, 'd, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        aes_dma: &'t mut AesDma<'d, C>,
    }

    impl<'t, 'd, C> DmaTransferRxTx for AesDmaTransferRxTx<'t, 'd, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        /// Wait for the DMA transfer to complete
        fn wait(self) -> Result<(), DmaError> {
            // Waiting for the DMA transfer is not enough. We need to wait for the
            // peripheral to finish flushing its buffers, too.
            while self.aes_dma.aes.aes.state().read().state().bits() != 2 // DMA status DONE == 2
                && !self.aes_dma.channel.tx.is_done()
            {
                // wait until done
            }

            self.aes_dma.finish_transform();

            if self.aes_dma.channel.rx.has_error() || self.aes_dma.channel.tx.has_error() {
                Err(DmaError::DescriptorError)
            } else {
                Ok(())
            }
        }

        /// Check if the DMA transfer is complete
        fn is_done(&self) -> bool {
            let ch = &self.aes_dma.channel;
            ch.tx.is_done() && ch.rx.is_done()
        }
    }

    impl<'t, 'd, C> Drop for AesDmaTransferRxTx<'t, 'd, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        fn drop(&mut self) {
            self.aes_dma
                .aes
                .aes
                .dma_exit()
                .write(|w| w.dma_exit().set_bit());
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

    impl<'d, C> AesDma<'d, C>
    where
        C: ChannelTypes,
        C::P: AesPeripheral,
    {
        pub fn write_key(&mut self, key: &[u8]) {
            debug_assert!(key.len() <= 8 * ALIGN_SIZE);
            debug_assert_eq!(key.len() % ALIGN_SIZE, 0);
            self.aes.write_key(key);
        }

        pub fn write_block(&mut self, block: &[u8]) {
            debug_assert_eq!(block.len(), 4 * ALIGN_SIZE);
            self.aes.write_key(block);
        }

        /// Perform a DMA transfer.
        ///
        /// This will return a [AesDmaTransferRxTx] owning the buffer(s) and the
        /// AES instance. The maximum amount of data to be sent/received
        /// is 32736 bytes.
        pub fn process<'t, TXBUF, RXBUF>(
            &'t mut self,
            words: &'t TXBUF,
            read_buffer: &'t mut RXBUF,
            mode: Mode,
            cipher_mode: CipherMode,
            key: [u8; 16],
        ) -> Result<AesDmaTransferRxTx<'t, 'd, C>, crate::dma::DmaError>
        where
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
                key,
            )?;

            Ok(AesDmaTransferRxTx { aes_dma: self })
        }

        #[allow(clippy::too_many_arguments)]
        fn start_transfer_dma(
            &mut self,
            write_buffer_ptr: *const u8,
            write_buffer_len: usize,
            read_buffer_ptr: *mut u8,
            read_buffer_len: usize,
            mode: Mode,
            cipher_mode: CipherMode,
            key: [u8; 16],
        ) -> Result<(), crate::dma::DmaError> {
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
            self.channel
                .rx
                .prepare_transfer_without_start(
                    false,
                    self.dma_peripheral(),
                    read_buffer_ptr,
                    read_buffer_len,
                )
                .and_then(|_| self.channel.rx.start_transfer())?;
            self.enable_dma(true);
            self.enable_interrupt();
            self.set_mode(mode);
            self.set_cipher_mode(cipher_mode);
            self.write_key(&key);

            // TODO: verify 16?
            self.set_num_block(16);

            self.start_transform();

            Ok(())
        }

        #[cfg(any(esp32c3, esp32s3))]
        pub fn reset_aes(&self) {
            unsafe {
                let s = crate::peripherals::SYSTEM::steal();
                s.perip_rst_en1()
                    .modify(|_, w| w.crypto_aes_rst().set_bit());
                s.perip_rst_en1()
                    .modify(|_, w| w.crypto_aes_rst().clear_bit());
            }
        }

        #[cfg(any(esp32c6, esp32h2))]
        pub fn reset_aes(&self) {
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

        pub fn set_cipher_mode(&self, mode: CipherMode) {
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

        pub fn set_mode(&self, mode: Mode) {
            self.aes
                .aes
                .mode()
                .modify(|_, w| w.mode().variant(mode as u8));
        }

        fn start_transform(&self) {
            self.aes.aes.trigger().write(|w| w.trigger().set_bit());
        }

        pub fn finish_transform(&self) {
            self.aes.aes.dma_exit().write(|w| w.dma_exit().set_bit());
            self.enable_dma(false);
        }

        fn set_num_block(&self, block: u32) {
            self.aes
                .aes
                .block_num()
                .modify(|_, w| unsafe { w.block_num().bits(block) });
        }
    }
}
