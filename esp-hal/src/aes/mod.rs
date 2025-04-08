//! # Advanced Encryption Standard (AES).
//!
//! ## Overview
//!
//! The AES accelerator is a hardware device that speeds up computation
//! using AES algorithm significantly, compared to AES algorithms implemented
//! solely in software.  The AES accelerator has two working modes, which are
//! Typical AES and AES-DMA.
//!
//! ## Configuration
//!
//! The AES peripheral can be configured to encrypt or decrypt data using
//! different encryption/decryption modes.
//!
//! When using AES-DMA, the peripheral can be configured to use different block
//! cipher modes such as ECB, CBC, OFB, CTR, CFB8, and CFB128.
//!
//! ## Examples
//!
//! ### Encrypting and decrypting a message
//!
//! Simple example of encrypting and decrypting a message using AES-128:
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::aes::{Aes, Mode};
//! # let keytext = b"SUp4SeCp@sSw0rd";
//! # let plaintext = b"message";
//! # let mut keybuf = [0_u8; 16];
//! # keybuf[..keytext.len()].copy_from_slice(keytext);
//! #
//! let mut block = [0_u8; 16];
//! block[..plaintext.len()].copy_from_slice(plaintext);
//!
//! let mut aes = Aes::new(peripherals.AES);
//! aes.process(&mut block, Mode::Encryption128, keybuf);
//!
//! // The encryption happens in-place, so the ciphertext is in `block`
//!
//! aes.process(&mut block, Mode::Decryption128, keybuf);
//!
//! // The decryption happens in-place, so the plaintext is in `block`
//! # Ok(())
//! # }
//! ```
//! 
//! ### AES-DMA
//!
//! Visit the [AES-DMA] test for a more advanced example of using AES-DMA
//! mode.
//!
//! [AES-DMA]: https://github.com/esp-rs/esp-hal/blob/main/hil-test/tests/aes_dma.rs
//!
//! ## Implementation State
//!
//! * AES-DMA mode is currently not supported on ESP32
//! * AES-DMA Initialization Vector (IV) is currently not supported

use crate::{
    pac,
    peripherals::AES,
    reg_access::{AlignmentHelper, NativeEndianess},
    system::GenericPeripheralGuard,
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
    aes: AES<'d>,
    alignment_helper: AlignmentHelper<NativeEndianess>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Aes as u8 }>,
}

impl<'d> Aes<'d> {
    /// Constructs a new `Aes` instance.
    pub fn new(aes: AES<'d>) -> Self {
        let guard = GenericPeripheralGuard::new();

        let mut ret = Self {
            aes,
            alignment_helper: AlignmentHelper::native_endianess(),
            _guard: guard,
        };
        ret.init();

        ret
    }

    fn regs(&self) -> &pac::aes::RegisterBlock {
        self.aes.register_block()
    }

    /// Encrypts/Decrypts the given buffer based on `mode` parameter
    pub fn process<K>(&mut self, block: &mut [u8; 16], mode: Mode, key: K)
    where
        K: Into<Key>,
    {
        // Convert from into Key enum
        self.write_key(key.into().as_slice());
        self.write_mode(mode);
        self.set_block(block);
        self.start();
        while !(self.is_idle()) {}
        self.block(block);
    }

    fn is_idle(&mut self) -> bool {
        self.read_idle()
    }

    fn set_block(&mut self, block: &[u8; 16]) {
        self.write_block(block);
    }

    fn block(&self, block: &mut [u8; 16]) {
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
#[cfg(any(esp32c3, esp32c6, esp32h2, esp32s2, esp32s3))]
pub mod dma {
    use core::mem::ManuallyDrop;

    use crate::{
        aes::{Key, Mode},
        dma::{
            Channel,
            DmaChannelFor,
            DmaPeripheral,
            DmaRxBuffer,
            DmaTxBuffer,
            PeripheralDmaChannel,
        },
        peripherals::AES,
        Blocking,
    };

    const ALIGN_SIZE: usize = core::mem::size_of::<u32>();

    /// Specifies the block cipher modes available for AES operations.
    #[derive(Clone, Copy, PartialEq, Eq)]
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
    #[instability::unstable]
    pub struct AesDma<'d> {
        /// The underlying [`Aes`](super::Aes) driver
        pub aes: super::Aes<'d>,

        channel: Channel<Blocking, PeripheralDmaChannel<AES<'d>>>,
    }

    impl<'d> crate::aes::Aes<'d> {
        /// Enable DMA for the current instance of the AES driver
        pub fn with_dma(self, channel: impl DmaChannelFor<AES<'d>>) -> AesDma<'d> {
            let channel = Channel::new(channel.degrade());
            channel.runtime_ensure_compatible(&self.aes);
            AesDma { aes: self, channel }
        }
    }

    impl core::fmt::Debug for AesDma<'_> {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            f.debug_struct("AesDma").finish()
        }
    }

    impl<'d> AesDma<'d> {
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
        /// This will return a [AesTransfer]. The maximum amount of data to
        /// be sent/received is 32736 bytes.
        pub fn process<K, RXBUF, TXBUF>(
            mut self,
            number_of_blocks: usize,
            mut output: RXBUF,
            mut input: TXBUF,
            mode: Mode,
            cipher_mode: CipherMode,
            key: K,
        ) -> Result<AesTransfer<'d, RXBUF, TXBUF>, (crate::dma::DmaError, Self, RXBUF, TXBUF)>
        where
            K: Into<Key>,
            TXBUF: DmaTxBuffer,
            RXBUF: DmaRxBuffer,
        {
            // AES has to be restarted after each calculation
            self.reset_aes();

            let result = unsafe {
                self.channel
                    .tx
                    .prepare_transfer(self.dma_peripheral(), &mut input)
                    .and_then(|_| self.channel.tx.start_transfer())
            };
            if let Err(err) = result {
                return Err((err, self, output, input));
            }

            let result = unsafe {
                self.channel
                    .rx
                    .prepare_transfer(self.dma_peripheral(), &mut output)
                    .and_then(|_| self.channel.rx.start_transfer())
            };
            if let Err(err) = result {
                self.channel.tx.stop_transfer();

                return Err((err, self, output, input));
            }

            self.enable_dma(true);
            self.enable_interrupt();
            self.aes.write_mode(mode);
            self.set_cipher_mode(cipher_mode);
            self.write_key(key.into());

            self.set_num_block(number_of_blocks as u32);

            self.start_transform();

            Ok(AesTransfer {
                aes_dma: ManuallyDrop::new(self),
                rx_view: ManuallyDrop::new(output.into_view()),
                tx_view: ManuallyDrop::new(input.into_view()),
            })
        }

        #[cfg(any(esp32c3, esp32s2, esp32s3))]
        fn reset_aes(&self) {
            use crate::peripherals::SYSTEM;

            SYSTEM::regs()
                .perip_rst_en1()
                .modify(|_, w| w.crypto_aes_rst().set_bit());
            SYSTEM::regs()
                .perip_rst_en1()
                .modify(|_, w| w.crypto_aes_rst().clear_bit());
        }

        #[cfg(any(esp32c6, esp32h2))]
        fn reset_aes(&self) {
            use crate::peripherals::PCR;

            PCR::regs()
                .aes_conf()
                .modify(|_, w| w.aes_rst_en().set_bit());
            PCR::regs()
                .aes_conf()
                .modify(|_, w| w.aes_rst_en().clear_bit());
        }

        fn dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::Aes
        }

        fn enable_dma(&self, enable: bool) {
            self.aes
                .regs()
                .dma_enable()
                .write(|w| w.dma_enable().bit(enable));
        }

        fn enable_interrupt(&self) {
            self.aes.regs().int_ena().write(|w| w.int_ena().set_bit());
        }

        fn set_cipher_mode(&self, mode: CipherMode) {
            self.aes
                .regs()
                .block_mode()
                .modify(|_, w| unsafe { w.block_mode().bits(mode as u8) });

            if mode == CipherMode::Ctr {
                self.aes
                    .regs()
                    .inc_sel()
                    .modify(|_, w| w.inc_sel().clear_bit());
            }
        }

        fn start_transform(&self) {
            self.aes.write_start();
        }

        fn finish_transform(&self) {
            self.aes.regs().dma_exit().write(|w| w.dma_exit().set_bit());
            self.enable_dma(false);
            self.reset_aes();
        }

        fn set_num_block(&self, block: u32) {
            self.aes
                .regs()
                .block_num()
                .modify(|_, w| unsafe { w.block_num().bits(block) });
        }
    }

    /// Represents an ongoing (or potentially stopped) transfer with the Aes.
    #[instability::unstable]
    pub struct AesTransfer<'d, RX: DmaRxBuffer, TX: DmaTxBuffer> {
        aes_dma: ManuallyDrop<AesDma<'d>>,
        rx_view: ManuallyDrop<RX::View>,
        tx_view: ManuallyDrop<TX::View>,
    }

    impl<'d, RX: DmaRxBuffer, TX: DmaTxBuffer> AesTransfer<'d, RX, TX> {
        /// Returns true when [Self::wait] will not block.
        pub fn is_done(&self) -> bool {
            // DMA status DONE == 2
            self.aes_dma.aes.regs().state().read().state().bits() == 2
        }

        /// Waits for the transfer to finish and returns the peripheral and
        /// buffers.
        pub fn wait(mut self) -> (AesDma<'d>, RX, TX) {
            while !self.is_done() {}

            // Stop the DMA as it doesn't know that the aes has stopped.
            self.aes_dma.channel.rx.stop_transfer();
            self.aes_dma.channel.tx.stop_transfer();

            self.aes_dma.finish_transform();

            let (aes_dma, rx_view, tx_view) = unsafe {
                let aes_dma = ManuallyDrop::take(&mut self.aes_dma);
                let rx_view = ManuallyDrop::take(&mut self.rx_view);
                let tx_view = ManuallyDrop::take(&mut self.tx_view);
                core::mem::forget(self);
                (aes_dma, rx_view, tx_view)
            };

            (aes_dma, RX::from_view(rx_view), TX::from_view(tx_view))
        }

        /// Provides shared access to the DMA rx buffer view.
        pub fn rx_view(&self) -> &RX::View {
            &self.rx_view
        }

        /// Provides exclusive access to the DMA rx buffer view.
        pub fn rx_view_mut(&mut self) -> &mut RX::View {
            &mut self.rx_view
        }

        /// Provides shared access to the DMA tx buffer view.
        pub fn tx_view(&self) -> &TX::View {
            &self.tx_view
        }

        /// Provides exclusive access to the DMA tx buffer view.
        pub fn tx_view_mut(&mut self) -> &mut TX::View {
            &mut self.tx_view
        }
    }

    impl<RX: DmaRxBuffer, TX: DmaTxBuffer> Drop for AesTransfer<'_, RX, TX> {
        fn drop(&mut self) {
            // Stop the DMA to prevent further memory access.
            self.aes_dma.channel.rx.stop_transfer();
            self.aes_dma.channel.tx.stop_transfer();

            // SAFETY: This is Drop, we know that self.i8080 and self.buf_view
            // won't be touched again.
            unsafe {
                ManuallyDrop::drop(&mut self.aes_dma);
            }
            let rx_view = unsafe { ManuallyDrop::take(&mut self.rx_view) };
            let tx_view = unsafe { ManuallyDrop::take(&mut self.tx_view) };
            let _ = RX::from_view(rx_view);
            let _ = TX::from_view(tx_view);
        }
    }
}
