//! # Advanced Encryption Standard (AES).
//!
//! ## Overview
//! The AES accelerator is a hardware device that speeds up computation
//! using AES algorithm significantly, compared to AES algorithms implemented
//! solely in software.  The AES accelerator has two working modes, which are
//! Typical AES and AES-DMA.
//!
//! ## Configuration
//! The AES peripheral can be configured to encrypt or decrypt data using
//! different encryption/decryption modes.
//!
//! When using AES-DMA, the peripheral can be configured to use different block
//! cipher modes such as ECB, CBC, OFB, CTR, CFB8, and CFB128.
//!
//! ## Examples
//! ### Encrypting and Decrypting a Message
//! Simple example of encrypting and decrypting a message using AES-128:
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::aes::{Aes, Mode};
//! # let keytext = "SUp4SeCp@sSw0rd".as_bytes();
//! # let plaintext = "message".as_bytes();
//! # let mut keybuf = [0_u8; 16];
//! # keybuf[..keytext.len()].copy_from_slice(keytext);
//! let mut block_buf = [0_u8; 16];
//! block_buf[..plaintext.len()].copy_from_slice(plaintext);
//! let mut block = block_buf.clone();
//!
//! let mut aes = Aes::new(peripherals.AES);
//! aes.process(&mut block, Mode::Encryption128, keybuf);
//! let hw_encrypted = block.clone();
//!
//! aes.process(&mut block, Mode::Decryption128, keybuf);
//! let hw_decrypted = block;
//! # }
//! ```
//! 
//! ### AES-DMA
//! Visit the [AES-DMA] test for a more advanced example of using AES-DMA
//! mode.
//!
//! [AES-DMA]: https://github.com/esp-rs/esp-hal/blob/main/hil-test/tests/aes_dma.rs
//!
//! ## Implementation State
//! * AES-DMA mode is currently not supported on ESP32 and ESP32S2
//! * AES-DMA Initialization Vector (IV) is currently not supported

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
    /// Type of the AES key, a fixed-size array of bytes
    ///
    /// The size of this type depends on various factors, such as the device
    /// being targeted and the desired key size.
    type KeyType<'b>;
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
    /// Big endian (most-significant byte at the smallest address)
    BigEndian    = 1,
    /// Little endian (least-significant byte at the smallest address)
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
    use crate::{
        aes::{Key, Mode},
        dma::{
            dma_private::{DmaSupport, DmaSupportRx, DmaSupportTx},
            AesPeripheral,
            Channel,
            ChannelRx,
            ChannelTx,
            DescriptorChain,
            DmaChannel,
            DmaDescriptor,
            DmaPeripheral,
            DmaTransferTxRx,
            ReadBuffer,
            RxPrivate,
            TxPrivate,
            WriteBuffer,
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
        C: DmaChannel,
        C::P: AesPeripheral,
    {
        /// The underlying [`Aes`](super::Aes) driver
        pub aes: super::Aes<'d>,

        pub(crate) channel: Channel<'d, C, crate::Blocking>,
        tx_chain: DescriptorChain,
        rx_chain: DescriptorChain,
    }

    /// Functionality for using AES with DMA.
    pub trait WithDmaAes<'d, C>
    where
        C: DmaChannel,
        C::P: AesPeripheral,
    {
        /// Enable DMA for the current instance of the AES driver
        fn with_dma(
            self,
            channel: Channel<'d, C, crate::Blocking>,
            tx_descriptors: &'static mut [DmaDescriptor],
            rx_descriptors: &'static mut [DmaDescriptor],
        ) -> AesDma<'d, C>;
    }

    impl<'d, C> WithDmaAes<'d, C> for crate::aes::Aes<'d>
    where
        C: DmaChannel,
        C::P: AesPeripheral,
    {
        fn with_dma(
            self,
            mut channel: Channel<'d, C, crate::Blocking>,
            tx_descriptors: &'static mut [DmaDescriptor],
            rx_descriptors: &'static mut [DmaDescriptor],
        ) -> AesDma<'d, C> {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            AesDma {
                aes: self,
                channel,
                tx_chain: DescriptorChain::new(tx_descriptors),
                rx_chain: DescriptorChain::new(rx_descriptors),
            }
        }
    }

    impl<'d, C> core::fmt::Debug for AesDma<'d, C>
    where
        C: DmaChannel,
        C::P: AesPeripheral,
    {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("AesDma").finish()
        }
    }

    impl<'d, C> DmaSupport for AesDma<'d, C>
    where
        C: DmaChannel,
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
        C: DmaChannel,
        C::P: AesPeripheral,
    {
        type TX = ChannelTx<'d, C>;

        fn tx(&mut self) -> &mut Self::TX {
            &mut self.channel.tx
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.tx_chain
        }
    }

    impl<'d, C> DmaSupportRx for AesDma<'d, C>
    where
        C: DmaChannel,
        C::P: AesPeripheral,
    {
        type RX = ChannelRx<'d, C>;

        fn rx(&mut self) -> &mut Self::RX {
            &mut self.channel.rx
        }

        fn chain(&mut self) -> &mut DescriptorChain {
            &mut self.rx_chain
        }
    }

    impl<'d, C> AesDma<'d, C>
    where
        C: DmaChannel,
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
        /// This will return a [DmaTransferTxRx]. The maximum amount of data to
        /// be sent/received is 32736 bytes.
        pub fn process<'t, K, TXBUF, RXBUF>(
            &'t mut self,
            words: &'t TXBUF,
            read_buffer: &'t mut RXBUF,
            mode: Mode,
            cipher_mode: CipherMode,
            key: K,
        ) -> Result<DmaTransferTxRx<'_, Self>, crate::dma::DmaError>
        where
            K: Into<Key>,
            TXBUF: ReadBuffer,
            RXBUF: WriteBuffer,
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

            unsafe {
                self.tx_chain
                    .fill_for_tx(false, write_buffer_ptr, write_buffer_len)?;
                self.channel
                    .tx
                    .prepare_transfer_without_start(self.dma_peripheral(), &self.tx_chain)
                    .and_then(|_| self.channel.tx.start_transfer())?;

                self.rx_chain
                    .fill_for_rx(false, read_buffer_ptr, read_buffer_len)?;
                self.channel
                    .rx
                    .prepare_transfer_without_start(self.dma_peripheral(), &self.rx_chain)
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
