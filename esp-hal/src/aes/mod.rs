#![cfg_attr(docsrs, procmacros::doc_replace)]
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
//! # {before_snippet}
//! # use esp_hal::aes::{Aes, Operation};
//! # let keytext = b"SUp4SeCp@sSw0rd";
//! # let plaintext = b"message";
//! # let mut keybuf = [0_u8; 16];
//! # keybuf[..keytext.len()].copy_from_slice(keytext);
//! #
//! let mut block = [0_u8; 16];
//! block[..plaintext.len()].copy_from_slice(plaintext);
//!
//! let mut aes = Aes::new(peripherals.AES);
//! aes.encrypt(&mut block, keybuf);
//!
//! // The encryption happens in-place, so the ciphertext is in `block`
//!
//! aes.decrypt(&mut block, keybuf);
//!
//! // The decryption happens in-place, so the plaintext is in `block`
//! # {after_snippet}
//! ```
//!
//! ### AES-DMA
//!
//! Visit the [AES-DMA] test for a more advanced example of using AES-DMA
//! mode.
//!
//! [AES-DMA]: https://github.com/esp-rs/esp-hal/blob/main/hil-test/tests/aes_dma.rs

use crate::{pac, peripherals::AES, system::GenericPeripheralGuard};

for_each_aes_key_length! {
    ($len:literal) => {
        // Implementing From for easy conversion from array to Key enum.
        impl From<[u8; $len / 8]> for Key {
            fn from(key: [u8; $len / 8]) -> Self {
                paste::paste! {
                    Key::[<Key $len>](key)
                }
            }
        }

        paste::paste! {
            #[doc = concat!("Marker type for AES-", stringify!($len))]
            pub struct [<Aes $len>];

            impl crate::private::Sealed for [<Aes $len>] {}
            impl AesFlavour for [<Aes $len>] {
                type KeyType<'b> = &'b [u8; $len / 8];
            }
        }
    };

    (bits $( ($len:literal) ),*) => {
        paste::paste! {
            /// Represents the various key sizes allowed for AES encryption and decryption.
            pub enum Key {
                $(
                    #[doc = concat!(stringify!($len), "-bit AES key")]
                    [<Key $len>]([u8; $len / 8]),
                )*
            }

            impl Key {
                /// Returns a slice representation of the AES key.
                fn as_slice(&self) -> &[u8] {
                    match self {
                        $(
                            Self::[<Key $len>](key) => key.as_ref(),
                        )*
                    }
                }
            }
        }
    };

    (modes $(($bits:literal, $encrypt:literal, $decrypt:literal)),*) => {
        paste::paste! {
            /// Defines the operating modes for AES encryption and decryption.
            #[repr(C)]
            #[derive(Clone, Copy, PartialEq, Eq, Debug)]
            enum Mode {
                $(
                    [<Encryption $bits>] = $encrypt,
                    [<Decryption $bits>] = $decrypt,
                )*
            }

            impl Key {
                fn encrypt_mode(&self) -> Mode {
                    match self {
                        $(Self::[<Key $bits>](_) => Mode::[<Encryption $bits>],)*
                    }
                }

                fn decrypt_mode(&self) -> Mode {
                    match self {
                        $(Self::[<Key $bits>](_) => Mode::[<Decryption $bits>],)*
                    }
                }
            }
        }
    };
}

/// Defines the operating modes for AES encryption and decryption.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Operation {
    /// Produce ciphertext from plaintext
    Encrypt,

    /// Produce plaintext from ciphertext
    Decrypt,
}

/// AES peripheral container
pub struct Aes<'d> {
    aes: AES<'d>,
    _guard: GenericPeripheralGuard<{ crate::system::Peripheral::Aes as u8 }>,
}

impl<'d> Aes<'d> {
    /// Constructs a new `Aes` instance.
    pub fn new(aes: AES<'d>) -> Self {
        let guard = GenericPeripheralGuard::new();

        #[cfg_attr(not(aes_dma), expect(unused_mut))]
        let mut this = Self { aes, _guard: guard };

        #[cfg(aes_dma)]
        this.write_dma(false);

        this
    }

    fn regs(&self) -> &pac::aes::RegisterBlock {
        self.aes.register_block()
    }

    /// Configures how the state matrix would be laid out
    #[cfg(aes_endianness_configurable)]
    pub fn write_endianness(
        &mut self,
        input_text_word_endianess: Endianness,
        input_text_byte_endianess: Endianness,
        output_text_word_endianess: Endianness,
        output_text_byte_endianess: Endianness,
        key_word_endianess: Endianness,
        key_byte_endianess: Endianness,
    ) {
        let mut to_write = 0_u32;
        to_write |= key_byte_endianess as u32;
        to_write |= (key_word_endianess as u32) << 1;
        to_write |= (input_text_byte_endianess as u32) << 2;
        to_write |= (input_text_word_endianess as u32) << 3;
        to_write |= (output_text_byte_endianess as u32) << 4;
        to_write |= (output_text_word_endianess as u32) << 5;
        self.regs().endian().write(|w| unsafe { w.bits(to_write) });
    }

    fn start(&self) {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().start().write(|w| w.start().set_bit());
            } else {
                self.regs().trigger().write(|w| w.trigger().set_bit());
            }
        }
    }

    fn is_idle(&mut self) -> bool {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                self.regs().idle().read().idle().bit_is_set()
            } else {
                self.regs().state().read().state().bits() == 0
            }
        }
    }

    fn write_key(&mut self, input: &[u8]) {
        for (i, word) in read_words(input).enumerate() {
            self.regs().key(i).write(|w| unsafe { w.bits(word) });
        }
    }

    fn write_block(&mut self, block: &[u8]) {
        for (i, word) in read_words(block).enumerate() {
            cfg_if::cfg_if! {
                if #[cfg(aes_has_split_text_registers)] {
                    self.regs().text_in(i).write(|w| unsafe { w.bits(word) });
                } else {
                    self.regs().text(i).write(|w| unsafe { w.bits(word) });
                }
            }
        }
    }

    fn read_block(&self, block: &mut [u8]) {
        cfg_if::cfg_if! {
            if #[cfg(aes_has_split_text_registers)] {
                write_words(block, |i| self.regs().text_out(i).read().bits());
            } else {
                write_words(block, |i| self.regs().text(i).read().bits());
            }
        }
    }

    fn write_mode(&self, mode: Mode) {
        self.regs().mode().write(|w| unsafe { w.bits(mode as _) });
    }

    #[cfg(aes_dma)]
    fn write_dma(&mut self, enable_dma: bool) {
        self.regs()
            .dma_enable()
            .write(|w| w.dma_enable().bit(enable_dma));
    }

    fn process(&mut self, block: &mut [u8; 16], mode: Mode, key: Key) {
        self.write_key(key.as_slice());
        self.write_mode(mode);
        self.write_block(block);
        self.start();
        while !(self.is_idle()) {}
        self.read_block(block);
    }

    /// Encrypts the given buffer with the given key.
    pub fn encrypt(&mut self, block: &mut [u8; 16], key: impl Into<Key>) {
        let key = key.into();
        let mode = key.encrypt_mode();
        self.process(block, mode, key)
    }

    /// Decrypts the given buffer with the given key.
    pub fn decrypt(&mut self, block: &mut [u8; 16], key: impl Into<Key>) {
        let key = key.into();
        let mode = key.decrypt_mode();
        self.process(block, mode, key)
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
#[cfg(aes_dma)]
pub mod dma {
    use core::mem::ManuallyDrop;

    use crate::{
        Blocking,
        aes::{Key, Operation},
        dma::{
            Channel,
            DmaChannelFor,
            DmaPeripheral,
            DmaRxBuffer,
            DmaTxBuffer,
            PeripheralDmaChannel,
        },
        peripherals::AES,
        system::{Peripheral, PeripheralClockControl},
    };

    const ALIGN_SIZE: usize = core::mem::size_of::<u32>();

    /// Specifies the block cipher modes available for AES operations.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum CipherMode {
        /// Electronic Codebook Mode
        #[cfg(aes_dma_mode_ecb)]
        Ecb = 0,
        /// Cipher Block Chaining Mode
        Cbc,
        /// Output Feedback Mode
        #[cfg(aes_dma_mode_ofb)]
        Ofb,
        /// Counter Mode.
        #[cfg(aes_dma_mode_ctr)]
        Ctr,
        /// Cipher Feedback Mode with 8-bit shifting.
        #[cfg(aes_dma_mode_cfb8)]
        Cfb8,
        /// Cipher Feedback Mode with 128-bit shifting.
        #[cfg(aes_dma_mode_cfb128)]
        Cfb128,
        // TODO: GCM needs different handling, not supported yet
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
        pub fn write_key(&mut self, key: impl Into<Key>) {
            let key = key.into(); // Convert into Key enum
            let key = key.as_slice();
            debug_assert!(key.len() <= 8 * ALIGN_SIZE);
            debug_assert_eq!(key.len() % ALIGN_SIZE, 0);
            self.aes.write_key(key);
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
            mode: Operation,
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

            let key = key.into();
            self.enable_dma(true);
            self.enable_interrupt();
            self.aes.write_mode(if mode == Operation::Encrypt {
                key.encrypt_mode()
            } else {
                key.decrypt_mode()
            });
            self.set_cipher_mode(cipher_mode);
            self.write_key(key);

            self.set_num_block(number_of_blocks as u32);

            self.start_transform();

            Ok(AesTransfer {
                aes_dma: ManuallyDrop::new(self),
                rx_view: ManuallyDrop::new(output.into_view()),
                tx_view: ManuallyDrop::new(input.into_view()),
            })
        }

        fn reset_aes(&self) {
            PeripheralClockControl::reset(Peripheral::Aes);
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
            self.aes.start();
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

            // SAFETY: This is Drop, we know that self.aes_dma and self.buf_view
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

// Utilities

fn read_words(slice: &[u8]) -> impl Iterator<Item = u32> {
    fn bytes<const N: usize>(slice: &[u8]) -> impl Iterator<Item = [u8; N]> {
        slice.chunks(N).map(|c| {
            let mut bytes = [0; N];
            bytes[0..c.len()].copy_from_slice(c);
            bytes
        })
    }

    bytes::<4>(slice).map(u32::from_le_bytes)
}

fn write_words(slice: &mut [u8], next: impl Fn(usize) -> u32) {
    for (i, chunk) in slice.chunks_mut(4).enumerate() {
        let bytes = next(i).to_le_bytes();
        chunk.copy_from_slice(&bytes[0..chunk.len()]);
    }
}
