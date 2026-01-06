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

use core::ptr::NonNull;

use crate::{
    aes::cipher_modes::{CryptoBuffers, UnsafeCryptoBuffers},
    pac,
    peripherals::AES,
    system::GenericPeripheralGuard,
};

pub mod cipher_modes;

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

                fn copy(&self) -> Self {
                    match self {
                        $(
                            Self::[<Key $len>](key) => Self::[<Key $len>](*key),
                        )*
                    }
                }
            }

            impl Drop for Key {
                fn drop(&mut self) {
                    use core::mem::MaybeUninit;
                    unsafe { (self as *mut Self).cast::<MaybeUninit<Self>>().write_volatile(MaybeUninit::zeroed()) };
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

/// The possible AES operations.
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

    /// Encrypts/Decrypts the given buffer based on `mode` parameter
    fn process_work_item(&mut self, work_item: &mut AesOperation) {
        // Note that we can't just create slices out of the input and output buffers, because they
        // may alias (when encrypting/decrypting data in place).

        let slice = work_item.key.as_slice();
        self.write_key(slice);
        self.write_mode(unsafe {
            work_item
                .cipher_mode
                .as_ref()
                .software_operating_mode(work_item.mode, &work_item.key)
        });

        let process_block = |input: NonNull<[u8]>, mut output: NonNull<[u8]>| {
            unsafe { self.write_block(input.as_ref()) };
            self.start();
            while !self.is_idle() {}
            unsafe { self.read_block(output.as_mut()) };
        };

        // Safety: the reference to the algorithm state is only held for the duration of the
        // operation.
        match unsafe { work_item.cipher_mode.as_mut() } {
            CipherState::Ecb(algo) => algo.encrypt_decrypt(work_item.buffers, process_block),
            CipherState::Cbc(algo) => {
                if work_item.mode == Operation::Encrypt {
                    algo.encrypt(work_item.buffers, process_block);
                } else {
                    algo.decrypt(work_item.buffers, process_block);
                }
            }
            CipherState::Ofb(algo) => algo.encrypt_decrypt(work_item.buffers, process_block),
            CipherState::Ctr(algo) => algo.encrypt_decrypt(work_item.buffers, process_block),
            CipherState::Cfb8(algo) => {
                if work_item.mode == Operation::Encrypt {
                    algo.encrypt(work_item.buffers, process_block)
                } else {
                    algo.decrypt(work_item.buffers, process_block)
                }
            }
            CipherState::Cfb128(algo) => {
                if work_item.mode == Operation::Encrypt {
                    algo.encrypt(work_item.buffers, process_block)
                } else {
                    algo.decrypt(work_item.buffers, process_block)
                }
            }
        }
    }
}

/// Data endianness
#[cfg(aes_endianness_configurable)]
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
    use core::{mem::ManuallyDrop, ptr::NonNull};

    use procmacros::{handler, ram};

    #[cfg(psram_dma)]
    use crate::dma::ManualWritebackBuffer;
    use crate::{
        Blocking,
        aes::{
            AES_WORK_QUEUE,
            AesOperation,
            BLOCK_SIZE,
            CipherState,
            Key,
            Mode,
            Operation,
            UnsafeCryptoBuffers,
            cipher_modes,
            read_words,
        },
        dma::{
            Channel,
            DmaChannelFor,
            DmaDescriptor,
            DmaError,
            DmaPeripheral,
            DmaRxBuffer,
            DmaTxBuffer,
            NoBuffer,
            PeripheralDmaChannel,
            prepare_for_rx,
            prepare_for_tx,
        },
        peripherals::AES,
        system::{Peripheral, PeripheralClockControl},
        work_queue::{Poll, Status, VTable, WorkQueueDriver},
    };

    /// Specifies the block cipher modes available for AES operations.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub enum CipherMode {
        /// Electronic Codebook Mode
        #[cfg(aes_dma_mode_ecb)]
        Ecb    = 0,
        /// Cipher Block Chaining Mode
        #[cfg(aes_dma_mode_cbc)]
        Cbc    = 1,
        /// Output Feedback Mode
        #[cfg(aes_dma_mode_ofb)]
        Ofb    = 2,
        /// Counter Mode.
        #[cfg(aes_dma_mode_ctr)]
        Ctr    = 3,
        /// Cipher Feedback Mode with 8-bit shifting.
        #[cfg(aes_dma_mode_cfb8)]
        Cfb8   = 4,
        /// Cipher Feedback Mode with 128-bit shifting.
        #[cfg(aes_dma_mode_cfb128)]
        Cfb128 = 5,
        // TODO: GCM needs different handling, not supported yet
    }

    // If we can process from PSRAM, we need 2 extra descriptors. One will store the unaligned head,
    // one will store the unaligned tail.
    const OUT_DESCR_COUNT: usize = 1 + 2 * cfg!(psram_dma) as usize;

    /// A DMA capable AES instance.
    #[instability::unstable]
    pub struct AesDma<'d> {
        /// The underlying [`Aes`](super::Aes) driver
        pub aes: super::Aes<'d>,

        channel: Channel<Blocking, PeripheralDmaChannel<AES<'d>>>,
    }

    impl<'d> super::Aes<'d> {
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
        fn write_key(&mut self, key: &Key) {
            let key = key.as_slice();
            self.aes.write_key(key);
        }

        fn write_iv(&mut self, iv: &[u8; BLOCK_SIZE]) {
            for (word, reg) in read_words(iv).zip(self.aes.regs().iv_mem_iter()) {
                reg.write(|w| unsafe { w.bits(word) });
            }
        }

        fn read_iv(&self, iv: &mut [u8; BLOCK_SIZE]) {
            iv[0..4].copy_from_slice(&self.aes.regs().iv_mem(0).read().bits().to_le_bytes());
            iv[4..8].copy_from_slice(&self.aes.regs().iv_mem(1).read().bits().to_le_bytes());
            iv[8..12].copy_from_slice(&self.aes.regs().iv_mem(2).read().bits().to_le_bytes());
            iv[12..16].copy_from_slice(&self.aes.regs().iv_mem(3).read().bits().to_le_bytes());
        }

        fn start_dma_transfer<RXBUF, TXBUF>(
            mut self,
            number_of_blocks: usize,
            mut output: RXBUF,
            mut input: TXBUF,
            mode: Mode,
            cipher_mode: CipherMode,
            key: &Key,
        ) -> Result<AesTransfer<'d, RXBUF, TXBUF>, (DmaError, Self, RXBUF, TXBUF)>
        where
            TXBUF: DmaTxBuffer,
            RXBUF: DmaRxBuffer,
        {
            let peri = self.dma_peripheral();

            if let Err(error) = unsafe { self.channel.tx.prepare_transfer(peri, &mut input) }
                .and_then(|_| unsafe { self.channel.rx.prepare_transfer(peri, &mut output) })
                // Start them together, to avoid the latter prepare discarding data from FIFOs.
                .and_then(|_| self.channel.tx.start_transfer())
                .and_then(|_| self.channel.rx.start_transfer())
            {
                return Err((error, self, output, input));
            }

            self.enable_dma(true);
            self.listen();

            self.aes.write_mode(mode);
            self.write_key(key);

            self.set_cipher_mode(cipher_mode);
            self.set_num_block(number_of_blocks as u32);

            self.start_transform();

            Ok(AesTransfer {
                aes_dma: ManuallyDrop::new(self),
                rx_view: ManuallyDrop::new(output.into_view()),
                tx_view: ManuallyDrop::new(input.into_view()),
            })
        }

        /// Perform a DMA transfer.
        ///
        /// This will return a [AesTransfer].
        pub fn process<K, RXBUF, TXBUF>(
            mut self,
            number_of_blocks: usize,
            output: RXBUF,
            input: TXBUF,
            mode: Operation,
            cipher_state: &DmaCipherState,
            key: K,
        ) -> Result<AesTransfer<'d, RXBUF, TXBUF>, (DmaError, Self, RXBUF, TXBUF)>
        where
            K: Into<Key>,
            TXBUF: DmaTxBuffer,
            RXBUF: DmaRxBuffer,
        {
            // AES has to be restarted after each calculation
            self.reset_aes();

            // This unwrap is fine, the cipher state can only be constructed from supported
            // modes of operation.
            let cipher_mode = unwrap!(cipher_state.state.hardware_cipher_mode());

            let key = key.into();
            cipher_state.state.write_state(&mut self);

            let mode = cipher_state.state.hardware_operating_mode(mode, &key);

            self.start_dma_transfer(number_of_blocks, output, input, mode, cipher_mode, &key)
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

        fn listen(&self) {
            self.aes.regs().int_ena().write(|w| w.int_ena().set_bit());
        }

        fn clear_interrupt(&self) {
            self.aes.regs().int_clr().write(|w| w.int_clr().set_bit());
        }

        fn set_cipher_mode(&self, mode: CipherMode) {
            self.aes
                .regs()
                .block_mode()
                .modify(|_, w| unsafe { w.block_mode().bits(mode as u8) });

            // FIXME
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

        fn is_done(&self) -> bool {
            const DMA_STATUS_DONE: u8 = 2;
            // TODO: PAC should provide the variants
            self.aes.regs().state().read().state().bits() == DMA_STATUS_DONE
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
            self.aes_dma.is_done()
        }

        /// Waits for the transfer to finish and returns the peripheral and
        /// buffers.
        pub fn wait(mut self) -> (AesDma<'d>, RX::Final, TX::Final) {
            while !self.is_done() {}
            while !self.aes_dma.channel.rx.is_done() {}

            // Stop the DMA as it doesn't know that the aes has stopped.
            self.aes_dma.channel.rx.stop_transfer();
            self.aes_dma.channel.tx.stop_transfer();

            self.aes_dma.finish_transform();

            unsafe {
                let aes_dma = ManuallyDrop::take(&mut self.aes_dma);
                let rx_view = ManuallyDrop::take(&mut self.rx_view);
                let tx_view = ManuallyDrop::take(&mut self.tx_view);

                core::mem::forget(self);

                (aes_dma, RX::from_view(rx_view), TX::from_view(tx_view))
            }
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

    const AES_DMA_VTABLE: VTable<super::AesOperation> = VTable {
        post: |driver, item| {
            let driver = unsafe { AesDmaBackend::from_raw(driver) };
            driver.start_processing(item);
            Some(Poll::Pending(false))
        },
        poll: |driver, item| {
            let driver = unsafe { AesDmaBackend::from_raw(driver) };
            driver.poll_status(item)
        },
        cancel: |_driver, _item| {
            // We can't (shouldn't) abort an in-progress computation, or we may corrupt the block
            // cipher's state.
        },
        stop: |driver| {
            // Drop the AES driver to conserve power when there is nothig to do (or when the driver
            // was stopped).
            let driver = unsafe { AesDmaBackend::from_raw(driver) };
            driver.deinitialize()
        },
    };

    enum DriverState<'d> {
        None,
        Idle(AesDma<'d>),
        WaitingForDma {
            transfer: AesTransfer<'d, NoBuffer, NoBuffer>,
            remaining_bytes: usize,
        },
    }

    #[procmacros::doc_replace(
        "dma_channel" => {
            cfg(esp32s2) => "let dma_channel = peripherals.DMA_CRYPTO;",
            _ => "let dma_channel = peripherals.DMA_CH0;"
        }
    )]
    /// DMA-enabled AES processing backend.
    ///
    /// The backend will try its best to use hardware acceleration as much as possible. It will
    /// fall back to CPU-driven processing (equivalent to [`AesBackend`](super::AesBackend)) in some
    /// cases, including:
    ///
    /// - When the block cipher is not implemented in hardware.
    /// - When the data is not correctly aligned to the needs of the hardware (e.g. when using a
    ///   stream cipher mode, the data length is not an integer multiple of 16 bytes).
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::aes::{AesContext, Operation, cipher_modes::Ecb, dma::AesDmaBackend};
    /// #
    /// # {dma_channel}
    /// let mut aes = AesDmaBackend::new(peripherals.AES, dma_channel);
    /// // Start the backend, which allows processing AES operations.
    /// let _backend = aes.start();
    ///
    /// // Create a new context with a 128-bit key. The context will use the ECB block cipher mode.
    /// // The key length must be supported by the hardware.
    /// let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, *b"SUp4SeCp@sSw0rd\0");
    ///
    /// // Process a block of data in this context. The ECB mode of operation requires that
    /// // the length of the data is a multiple of the block (16 bytes) size.
    /// let input_buffer: [u8; 16] = *b"message\0\0\0\0\0\0\0\0\0";
    /// let mut output_buffer: [u8; 16] = [0; 16];
    ///
    /// let operation_handle = ecb_encrypt
    ///     .process(&input_buffer, &mut output_buffer)
    ///     .unwrap();
    /// operation_handle.wait_blocking();
    ///
    /// // output_buffer now contains the ciphertext
    /// assert_eq!(
    ///     output_buffer,
    ///     [
    ///         0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
    ///         0x31, 0x96,
    ///     ]
    /// );
    /// #
    /// # {after_snippet}
    /// ```
    pub struct AesDmaBackend<'d> {
        peri: AES<'d>,
        dma: PeripheralDmaChannel<AES<'d>>,
        state: DriverState<'d>,

        #[cfg(psram_dma)]
        unaligned_data_buffers: [Option<ManualWritebackBuffer>; 2],
        input_descriptors: [DmaDescriptor; 1],
        output_descriptors: [DmaDescriptor; OUT_DESCR_COUNT],
    }

    // The DMA descriptors prevent auto-implementing Sync and Send, but they can be treated as Send
    // and Sync because we only access them in a critical section (around the work queue
    // operations), and only when the DMA is not actively using them. `unaligned_data_buffers`
    // contain pointers to AesContext data, and are only accessed when the associated data is
    // handled by the work queue. Other parts of the backend are Send and Sync automatically.
    unsafe impl Sync for AesDmaBackend<'_> {}
    unsafe impl Send for AesDmaBackend<'_> {}

    impl<'d> AesDmaBackend<'d> {
        #[procmacros::doc_replace(
            "dma_channel" => {
                cfg(esp32s2) => "let dma_channel = peripherals.DMA_CRYPTO;",
                _ => "let dma_channel = peripherals.DMA_CH0;"
            }
        )]
        /// Creates a new DMA-enabled AES backend.
        ///
        /// The backend needs to be [`start`][Self::start]ed before it can execute AES operations.
        ///
        /// ## Example
        ///
        /// ```rust, no_run
        /// # {before_snippet}
        /// use esp_hal::aes::dma::AesDmaBackend;
        ///
        /// # {dma_channel}
        /// let mut aes = AesDmaBackend::new(peripherals.AES, dma_channel);
        /// # {after_snippet}
        /// ```
        pub fn new(aes: AES<'d>, dma: impl DmaChannelFor<AES<'d>>) -> Self {
            Self {
                peri: aes,
                dma: dma.degrade(),
                state: DriverState::None,

                #[cfg(psram_dma)]
                unaligned_data_buffers: [const { None }; 2],
                input_descriptors: [DmaDescriptor::EMPTY; 1],
                output_descriptors: [DmaDescriptor::EMPTY; OUT_DESCR_COUNT],
            }
        }

        #[procmacros::doc_replace(
            "dma_channel" => {
                cfg(esp32s2) => "let dma_channel = peripherals.DMA_CRYPTO;",
                _ => "let dma_channel = peripherals.DMA_CH0;"
            }
        )]
        /// Registers the DMA-driven AES driver to process AES operations.
        ///
        /// The driver stops operating when the returned object is dropped.
        ///
        /// ## Example
        ///
        /// ```rust, no_run
        /// # {before_snippet}
        /// use esp_hal::aes::dma::AesDmaBackend;
        ///
        /// # {dma_channel}
        /// let mut aes = AesDmaBackend::new(peripherals.AES, dma_channel);
        /// let _handle = aes.start();
        /// # {after_snippet}
        /// ```
        pub fn start(&mut self) -> AesDmaWorkQueueDriver<'_, 'd> {
            AesDmaWorkQueueDriver {
                _inner: WorkQueueDriver::new(self, AES_DMA_VTABLE, &super::AES_WORK_QUEUE),
            }
        }

        // WorkQueue callbacks. They may run in any context.

        unsafe fn from_raw<'any>(ptr: NonNull<()>) -> &'any mut Self {
            unsafe { ptr.cast::<AesDmaBackend<'_>>().as_mut() }
        }

        fn start_processing(&mut self, item: &mut AesOperation) -> Poll {
            let driver = match core::mem::replace(&mut self.state, DriverState::None) {
                DriverState::None => {
                    let peri = unsafe { self.peri.clone_unchecked() };
                    let dma = unsafe { self.dma.clone_unchecked() };
                    let driver = super::Aes::new(peri).with_dma(dma);

                    driver
                        .aes
                        .aes
                        .bind_peri_interrupt(interrupt_handler.handler());
                    driver
                        .aes
                        .aes
                        .enable_peri_interrupt(interrupt_handler.priority());

                    driver
                }
                DriverState::Idle(aes_dma) => aes_dma,
                _ => unreachable!(),
            };

            // There are some constraints that make us (partially) fall back to CPU-driven mode:
            // - The algo isn't implemented in hardware. Not much to do here, we process the whole
            //   data using the CPU.
            // - There is data stuck in the cipher state.
            //   - In this case we need to flush the data before we can start using the DMA.
            // - The data alignment isn't appropriate to its location (PSRAM)
            //   - The DMA can read (transmit) any number of bytes from any address (in theory), but
            //     it can't write arbitrarily to PSRAM. We should be able to split the write
            //     transfer into two parts, and write the unaligned bytes into internal memory using
            //     the DMA, then copy the data out to PSRAM using the CPU at the end.
            // - The data length (after all the above) is not a multiple of the block length
            //   - We process as many blocks as we can using the DMA, then use the CPU for the
            //     remainder.

            if unsafe { !item.cipher_mode.as_ref().implemented_in_hardware() } {
                // Algo is either not implemented in hardware, or the data is not accessible by DMA.
                self.process_with_cpu(driver, item);
                return Poll::Ready(Status::Completed);
            }

            if !crate::soc::is_valid_memory_address(item.buffers.input.addr().get()) {
                unsafe {
                    // Safety:
                    // We've verified when constructing the buffer that input and output are the
                    // same length. The CryptoBuffers constructor also ensures that output is
                    // mutable and therefore is in RAM. If input is not, the pointers
                    // don't overlap.
                    // A buffer created via `CryptoBuffers::new_in_place` will not reach here
                    // because it takes a `&mut` buffer, which is always in RAM.
                    item.buffers.output.cast::<u8>().copy_from_nonoverlapping(
                        item.buffers.input.cast::<u8>(),
                        item.buffers.input.len(),
                    );
                }
                // We've copied the data, now overwrite the pointer to make this an in-place
                // operation.
                item.buffers.input = item.buffers.output;
            }

            // Flush available bytes:
            let flushed = unsafe { item.cipher_mode.as_mut().flush(item.buffers, item.mode) };
            if flushed == item.buffers.input.len() {
                // No more data to process
                self.state = DriverState::Idle(driver);
                return Poll::Ready(Status::Completed);
            }

            // Process the remaining data with DMA:
            let mut new_item = AesOperation {
                mode: item.mode,
                cipher_mode: item.cipher_mode,
                buffers: unsafe { item.buffers.byte_add(flushed) },
                key: item.key.copy(),
            };

            // If there is enough data for the DMA, set it up:
            if let Err(driver) = self.process_with_dma(driver, &mut new_item) {
                // Otherwise, process the remaining data with CPU.
                self.process_with_cpu(driver, &mut new_item);
                return Poll::Ready(Status::Completed);
            }

            Poll::Pending(false)
        }

        fn poll_status(&mut self, item: &mut AesOperation) -> Poll {
            match &self.state {
                DriverState::WaitingForDma { transfer, .. } if !transfer.is_done() => {
                    Poll::Pending(false)
                }

                DriverState::WaitingForDma { .. } => {
                    let DriverState::WaitingForDma {
                        transfer,
                        remaining_bytes,
                    } = core::mem::replace(&mut self.state, DriverState::None)
                    else {
                        unreachable!()
                    };

                    let (driver, _, _) = transfer.wait();
                    unsafe { item.cipher_mode.as_mut() }.read_state(&driver);

                    driver.clear_interrupt();

                    // Write out PSRAM data if needed:
                    #[cfg(psram_dma)]
                    for buffer in self.unaligned_data_buffers.iter_mut() {
                        // Avoid copying the write_back buffer
                        if let Some(buffer) = buffer.as_ref() {
                            buffer.write_back();
                        }
                        *buffer = None;
                    }
                    #[cfg(psram_dma)]
                    if crate::psram::psram_range().contains(&item.buffers.output.addr().get()) {
                        unsafe {
                            crate::soc::cache_writeback_addr(
                                item.buffers.output.addr().get() as u32,
                                item.buffers.output.len() as u32,
                            );
                        }
                    }

                    // Now process the remainder:
                    if remaining_bytes > 0 {
                        let mut temp_item = AesOperation {
                            mode: item.mode,
                            cipher_mode: item.cipher_mode,
                            buffers: unsafe {
                                item.buffers
                                    .byte_add(item.buffers.input.len() - remaining_bytes)
                            },
                            key: item.key.copy(),
                        };

                        // If there is enough data for the DMA, set it up again:
                        if let Err(driver) = self.process_with_dma(driver, &mut temp_item) {
                            // Otherwise, process the remaining data with CPU.
                            self.process_with_cpu(driver, &mut temp_item);
                        } else {
                            return Poll::Pending(false);
                        }
                    }

                    Poll::Ready(Status::Completed)
                }
                _ => unreachable!(),
            }
        }

        fn process_with_cpu(&mut self, mut driver: AesDma<'d>, work_item: &mut AesOperation) {
            driver.aes.process_work_item(work_item);
            self.state = DriverState::Idle(driver);
        }

        fn process_with_dma(
            &mut self,
            mut driver: AesDma<'d>,
            work_item: &mut AesOperation,
        ) -> Result<(), AesDma<'d>> {
            let input_len = work_item.buffers.input.len();
            let (input_buffer, data_len) = unsafe {
                // This unwrap is infallible as AES-DMA devices don't have TX DMA alignment
                // requirements.
                unwrap!(prepare_for_tx(
                    &mut self.input_descriptors,
                    work_item.buffers.input,
                    BLOCK_SIZE,
                ))
            };

            let number_of_blocks = data_len / BLOCK_SIZE;
            if number_of_blocks == 0 {
                // DMA can't do anything.
                return Err(driver);
            }

            let (output_buffer, rx_data_len) = unsafe {
                prepare_for_rx(
                    &mut self.output_descriptors,
                    #[cfg(psram_dma)]
                    &mut self.unaligned_data_buffers,
                    // Truncate data based on how much the TX buffer can read.
                    NonNull::slice_from_raw_parts(work_item.buffers.output.cast::<u8>(), data_len),
                )
            };

            debug_assert_eq!(rx_data_len, data_len);

            let cipher_state = unsafe { work_item.cipher_mode.as_ref() };
            cipher_state.write_state(&mut driver);

            let mode = cipher_state.hardware_operating_mode(work_item.mode, &work_item.key);
            // This unwrap is fine, this function is not called for modes of operation not supported
            // by the hardware.
            let cipher_mode = unwrap!(cipher_state.hardware_cipher_mode());

            let Ok(transfer) = driver.start_dma_transfer(
                number_of_blocks,
                output_buffer,
                input_buffer,
                mode,
                cipher_mode,
                &work_item.key,
            ) else {
                panic!()
            };

            self.state = DriverState::WaitingForDma {
                transfer,
                remaining_bytes: input_len - data_len,
            };

            Ok(())
        }

        fn deinitialize(&mut self) {
            self.state = DriverState::None;
        }
    }

    #[handler]
    #[ram]
    fn interrupt_handler() {
        AES_WORK_QUEUE.process();
    }

    /// An active work queue driver.
    ///
    /// This object must be kept around, otherwise AES operations will never complete.
    pub struct AesDmaWorkQueueDriver<'t, 'd> {
        _inner: WorkQueueDriver<'t, AesDmaBackend<'d>, AesOperation>,
    }

    /// The state of block ciphers that the AES hardware implements.
    #[derive(Clone)]
    pub struct DmaCipherState {
        state: CipherState,
    }

    impl DmaCipherState {
        /// Saves the block cipher state in memory so that it can be restored by a later operation.
        pub fn save_state(&mut self, aes_dma: &AesDma<'_>) {
            self.state.read_state(aes_dma);
        }
    }

    #[cfg(aes_dma_mode_ecb)]
    impl From<cipher_modes::Ecb> for DmaCipherState {
        fn from(value: cipher_modes::Ecb) -> Self {
            Self {
                state: CipherState::Ecb(value),
            }
        }
    }

    #[cfg(aes_dma_mode_cbc)]
    impl From<cipher_modes::Cbc> for DmaCipherState {
        fn from(value: cipher_modes::Cbc) -> Self {
            Self {
                state: CipherState::Cbc(value),
            }
        }
    }

    #[cfg(aes_dma_mode_ofb)]
    impl From<cipher_modes::Ofb> for DmaCipherState {
        fn from(value: cipher_modes::Ofb) -> Self {
            Self {
                state: CipherState::Ofb(value),
            }
        }
    }

    #[cfg(aes_dma_mode_ctr)]
    impl From<cipher_modes::Ctr> for DmaCipherState {
        fn from(value: cipher_modes::Ctr) -> Self {
            Self {
                state: CipherState::Ctr(value),
            }
        }
    }

    #[cfg(aes_dma_mode_cfb8)]
    impl From<cipher_modes::Cfb8> for DmaCipherState {
        fn from(value: cipher_modes::Cfb8) -> Self {
            Self {
                state: CipherState::Cfb8(value),
            }
        }
    }

    #[cfg(aes_dma_mode_cfb128)]
    impl From<cipher_modes::Cfb128> for DmaCipherState {
        fn from(value: cipher_modes::Cfb128) -> Self {
            Self {
                state: CipherState::Cfb128(value),
            }
        }
    }

    impl CipherState {
        fn hardware_cipher_mode(&self) -> Option<CipherMode> {
            #[allow(unreachable_patterns)]
            match self {
                #[cfg(aes_dma_mode_ecb)]
                Self::Ecb(_) => Some(CipherMode::Ecb),
                #[cfg(aes_dma_mode_cbc)]
                Self::Cbc(_) => Some(CipherMode::Cbc),
                #[cfg(aes_dma_mode_ofb)]
                Self::Ofb(_) => Some(CipherMode::Ofb),
                #[cfg(aes_dma_mode_ctr)]
                Self::Ctr(_) => Some(CipherMode::Ctr),
                #[cfg(aes_dma_mode_cfb8)]
                Self::Cfb8(_) => Some(CipherMode::Cfb8),
                #[cfg(aes_dma_mode_cfb128)]
                Self::Cfb128(_) => Some(CipherMode::Cfb128),
                // TODO: GCM
                _ => None,
            }
        }

        fn implemented_in_hardware(&self) -> bool {
            self.hardware_cipher_mode().is_some()
        }

        fn hardware_operating_mode(&self, operation: Operation, key: &Key) -> Mode {
            if operation == Operation::Encrypt {
                key.encrypt_mode()
            } else {
                key.decrypt_mode()
            }
        }

        fn flush(&mut self, buffers: UnsafeCryptoBuffers, mode: Operation) -> usize {
            match self {
                // These operate on complete blocks, nothing to flush:
                Self::Ecb(_) | Self::Cbc(_) => 0,

                // CFB8 shifts bytes but has no internal state other than IV:
                Self::Cfb8(_) => 0,

                // These modes may have bytes to output:
                Self::Ofb(ofb) => ofb.flush(buffers),
                Self::Ctr(ctr) => ctr.flush(buffers),
                Self::Cfb128(cfb128) => {
                    if mode == Operation::Encrypt {
                        cfb128.flush_encrypt(buffers)
                    } else {
                        cfb128.flush_decrypt(buffers)
                    }
                }
            }
        }

        fn write_state(&self, aes: &mut AesDma<'_>) {
            match self {
                CipherState::Ecb(_ecb) => {}
                CipherState::Cbc(cbc) => aes.write_iv(&cbc.iv),
                CipherState::Ofb(ofb) => aes.write_iv(&ofb.iv),
                CipherState::Ctr(ctr) => aes.write_iv(&ctr.nonce),
                CipherState::Cfb8(cfb8) => aes.write_iv(&cfb8.iv),
                CipherState::Cfb128(cfb128) => aes.write_iv(&cfb128.iv),
            }
        }

        fn read_state(&mut self, aes: &AesDma<'_>) {
            match self {
                CipherState::Ecb(_ecb) => {}
                CipherState::Cbc(cbc) => aes.read_iv(&mut cbc.iv),
                CipherState::Ofb(ofb) => aes.read_iv(&mut ofb.iv),
                CipherState::Ctr(ctr) => aes.read_iv(&mut ctr.nonce),
                CipherState::Cfb8(cfb8) => aes.read_iv(&mut cfb8.iv),
                CipherState::Cfb128(cfb128) => aes.read_iv(&mut cfb128.iv),
            }
        }
    }
}

use crate::work_queue::{
    Handle,
    Poll,
    Status,
    VTable,
    WorkQueue,
    WorkQueueDriver,
    WorkQueueFrontend,
};

/// The stored state of various block cipher modes.
#[derive(Clone)]
#[non_exhaustive]
pub enum CipherState {
    /// Electronic Codebook Mode
    Ecb(cipher_modes::Ecb),
    /// Cipher Block Chaining Mode
    Cbc(cipher_modes::Cbc),
    /// Output Feedback Mode
    Ofb(cipher_modes::Ofb),
    /// Counter Mode
    Ctr(cipher_modes::Ctr),
    /// Cipher Feedback Mode with 8-bit shifting.
    Cfb8(cipher_modes::Cfb8),
    /// Cipher Feedback Mode with 128-bit shifting.
    Cfb128(cipher_modes::Cfb128),
    // Galois Counter Mode
    // Gcm(*mut Gcm), // TODO: this is more involved
}

impl From<cipher_modes::Ecb> for CipherState {
    fn from(value: cipher_modes::Ecb) -> Self {
        Self::Ecb(value)
    }
}

impl From<cipher_modes::Cbc> for CipherState {
    fn from(value: cipher_modes::Cbc) -> Self {
        Self::Cbc(value)
    }
}

impl From<cipher_modes::Ofb> for CipherState {
    fn from(value: cipher_modes::Ofb) -> Self {
        Self::Ofb(value)
    }
}

impl From<cipher_modes::Ctr> for CipherState {
    fn from(value: cipher_modes::Ctr) -> Self {
        Self::Ctr(value)
    }
}

impl From<cipher_modes::Cfb8> for CipherState {
    fn from(value: cipher_modes::Cfb8) -> Self {
        Self::Cfb8(value)
    }
}

impl From<cipher_modes::Cfb128> for CipherState {
    fn from(value: cipher_modes::Cfb128) -> Self {
        Self::Cfb128(value)
    }
}

impl CipherState {
    fn software_operating_mode(&self, operation: Operation, key: &Key) -> Mode {
        match self {
            CipherState::Ecb(_) | CipherState::Cbc(_) => {
                if operation == Operation::Encrypt {
                    key.encrypt_mode()
                } else {
                    key.decrypt_mode()
                }
            }
            // For these, decryption is handled in software using the hardware in ecryption mode to
            // produce intermediate results.
            CipherState::Ofb(_)
            | CipherState::Ctr(_)
            | CipherState::Cfb8(_)
            | CipherState::Cfb128(_) => key.encrypt_mode(),
        }
    }

    /// Returns whether the mode of operation requires complete 16-byte blocks.
    fn requires_blocks(&self) -> bool {
        matches!(
            self,
            CipherState::Ecb(_) | CipherState::Ofb(_) | CipherState::Cbc(_)
        )
    }
}

const BLOCK_SIZE: usize = 16;

struct AesOperation {
    mode: Operation,
    // The block cipher mode of operation.
    cipher_mode: NonNull<CipherState>,
    // The length of the key is determined by the mode of operation. Note that the pointers may
    // change during AES operation.
    buffers: UnsafeCryptoBuffers,
    key: Key,
}

impl Clone for AesOperation {
    fn clone(&self) -> Self {
        Self {
            mode: self.mode,
            cipher_mode: self.cipher_mode,
            buffers: self.buffers,
            key: self.key.copy(),
        }
    }
}

// Safety: AesOperation is safe to share between threads, in the context of a WorkQueue. The
// WorkQueue ensures that only a single location can access the data. All the internals, except
// for the pointers, are Sync. The pointers are safe to share because they point at data that the
// AES driver ensures can be accessed safely and soundly.
unsafe impl Sync for AesOperation {}
// Safety: we will not hold on to the pointers when the work item leaves the queue.
unsafe impl Send for AesOperation {}

static AES_WORK_QUEUE: WorkQueue<AesOperation> = WorkQueue::new();
const BLOCKING_AES_VTABLE: VTable<AesOperation> = VTable {
    post: |driver, item| {
        // Fully process the work item. A single CPU-driven AES operation would complete
        // faster than we could poll the queue with all the locking around it.
        let driver = unsafe { AesBackend::from_raw(driver) };
        Some(driver.process(item))
    },
    poll: |_driver, _item| {
        // We've processed the item completely when we received it in `post`.
        unreachable!()
    },
    cancel: |_driver, _item| {
        // To achieve a decent performance in Typical AES mode, we run the operations in a blocking
        // manner and so they can't be cancelled.
    },
    stop: |driver| {
        // Drop the AES driver to conserve power when there is nothig to do (or when the driver was
        // stopped).
        let driver = unsafe { AesBackend::from_raw(driver) };
        driver.deinitialize()
    },
};

#[procmacros::doc_replace]
/// CPU-driven AES processing backend.
///
/// ## Example
///
/// ```rust, no_run
/// # {before_snippet}
/// use esp_hal::aes::{AesBackend, AesContext, Operation, cipher_modes::Ecb};
/// #
/// let mut aes = AesBackend::new(peripherals.AES);
/// // Start the backend, which allows processing AES operations.
/// let _backend = aes.start();
///
/// // Create a new context with a 128-bit key. The context will use the ECB block cipher mode.
/// // The key length must be supported by the hardware.
/// let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, *b"SUp4SeCp@sSw0rd\0");
///
/// // Process a block of data in this context. The ECB mode of operation requires that
/// // the length of the data is a multiple of the block (16 bytes) size.
/// let input_buffer: [u8; 16] = *b"message\0\0\0\0\0\0\0\0\0";
/// let mut output_buffer: [u8; 16] = [0; 16];
///
/// let operation_handle = ecb_encrypt
///     .process(&input_buffer, &mut output_buffer)
///     .unwrap();
/// operation_handle.wait_blocking();
///
/// // output_buffer now contains the ciphertext
/// assert_eq!(
///     output_buffer,
///     [
///         0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
///         0x31, 0x96,
///     ]
/// );
/// #
/// # {after_snippet}
/// ```
pub struct AesBackend<'d> {
    peri: AES<'d>,
    driver: Option<Aes<'d>>,
}

impl<'d> AesBackend<'d> {
    #[procmacros::doc_replace]
    /// Creates a new AES backend.
    ///
    /// The backend needs to be [`start`][Self::start]ed before it can execute AES operations.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::aes::AesBackend;
    /// #
    /// let mut aes = AesBackend::new(peripherals.AES);
    /// # {after_snippet}
    /// ```
    pub fn new(aes: AES<'d>) -> Self {
        Self {
            peri: aes,
            driver: None,
        }
    }

    #[procmacros::doc_replace]
    /// Registers the CPU-driven AES driver to process AES operations.
    ///
    /// The driver stops operating when the returned object is dropped.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::aes::AesBackend;
    /// #
    /// let mut aes = AesBackend::new(peripherals.AES);
    /// // Start the backend, which allows processing AES operations.
    /// let _backend = aes.start();
    /// # {after_snippet}
    /// ```
    pub fn start(&mut self) -> AesWorkQueueDriver<'_, 'd> {
        AesWorkQueueDriver {
            inner: WorkQueueDriver::new(self, BLOCKING_AES_VTABLE, &AES_WORK_QUEUE),
        }
    }

    // WorkQueue callbacks. They may run in any context.

    unsafe fn from_raw<'any>(ptr: NonNull<()>) -> &'any mut Self {
        unsafe { ptr.cast::<AesBackend<'_>>().as_mut() }
    }

    fn process(&mut self, item: &mut AesOperation) -> Poll {
        let driver = self.driver.get_or_insert_with(|| {
            let peri = unsafe { self.peri.clone_unchecked() };
            Aes::new(peri)
        });

        driver.process_work_item(item);

        Poll::Ready(Status::Completed)
    }

    fn deinitialize(&mut self) {
        self.driver = None;
    }
}

/// An error related to an AES operation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The input and output buffers have different lengths.
    BuffersNotEqual,

    /// The buffer length is not appropriate for the current cipher mode.
    IncorrectBufferLength,
}

/// An active work queue driver.
///
/// This object must be kept around, otherwise AES operations will never complete.
pub struct AesWorkQueueDriver<'t, 'd> {
    inner: WorkQueueDriver<'t, AesBackend<'d>, AesOperation>,
}

impl<'t, 'd> AesWorkQueueDriver<'t, 'd> {
    /// Finishes processing the current work queue item, then stops the driver.
    pub fn stop(self) -> impl Future<Output = ()> {
        self.inner.stop()
    }
}

/// An AES work queue user.
#[derive(Clone)]
pub struct AesContext {
    cipher_mode: CipherState,
    frontend: WorkQueueFrontend<AesOperation>,
}

impl AesContext {
    /// Creates a new context to encrypt or decrypt data with the given block cipher mode of
    /// operation.
    pub fn new(
        cipher_mode: impl Into<CipherState>,
        operation: Operation,
        key: impl Into<Key>,
    ) -> Self {
        Self {
            cipher_mode: cipher_mode.into(),
            frontend: WorkQueueFrontend::new(AesOperation {
                mode: operation,
                cipher_mode: NonNull::dangling(),
                buffers: unsafe { CryptoBuffers::new_in_place(&mut []).into_inner() },
                key: key.into(),
            }),
        }
    }

    fn post(&mut self) -> AesHandle<'_> {
        AesHandle(self.frontend.post(&AES_WORK_QUEUE))
    }

    fn validate(&self, buffer: &[u8]) -> Result<(), Error> {
        if self.cipher_mode.requires_blocks() && !buffer.len().is_multiple_of(BLOCK_SIZE) {
            return Err(Error::IncorrectBufferLength);
        }

        Ok(())
    }

    /// Starts transforming the input buffer, and writes the result into the output buffer.
    ///
    /// - For encryption the input is the plaintext, the output is the ciphertext.
    /// - For decryption the input is the ciphertext, the output is the plaintext.
    ///
    /// The returned Handle must be polled until it returns `true`. Dropping the handle
    /// before the operation finishes will cancel the operation.
    ///
    /// For an example, see the documentation of [`AesBackend`].
    ///
    /// ## Errors
    ///
    /// - If the lengths of the input and output buffers don't match, an error is returned.
    /// - The ECB and OFB cipher modes require the data length to be a multiple of the block size
    ///   (16), otherwise an error is returned.
    pub fn process<'t>(
        &'t mut self,
        input: &'t [u8],
        output: &'t mut [u8],
    ) -> Result<AesHandle<'t>, Error> {
        self.validate(input)?;
        self.validate(output)?;

        let data = self.frontend.data_mut();
        data.cipher_mode = NonNull::from(&mut self.cipher_mode);
        data.buffers = unsafe { CryptoBuffers::new(input, output)?.into_inner() };

        Ok(self.post())
    }

    #[procmacros::doc_replace]
    /// Starts transforming the buffer.
    ///
    /// The processed data will be written back to the `buffer`.
    ///
    /// The returned Handle must be polled until it returns `true`. Dropping the handle
    /// before the operation finishes will cancel the operation.
    ///
    /// This function operates similar to [`AesContext::process`], but it overwrites the data buffer
    /// with the result of the transformation.
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::aes::{AesBackend, AesContext, Operation, cipher_modes::Ecb};
    /// #
    /// let mut aes = AesBackend::new(peripherals.AES);
    /// // Start the backend, which pins it in place and allows processing AES operations.
    /// let _backend = aes.start();
    ///
    /// // Create a new context with a 128-bit key. The context will use the ECB block cipher mode.
    /// // The key length must be supported by the hardware.
    /// let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, *b"SUp4SeCp@sSw0rd\0");
    ///
    /// // Process a block of data in this context, in place. The ECB mode of operation requires that
    /// // the length of the data is a multiple of the block (16 bytes) size.
    /// let mut buffer: [u8; 16] = *b"message\0\0\0\0\0\0\0\0\0";
    ///
    /// let operation_handle = ecb_encrypt.process_in_place(&mut buffer).unwrap();
    /// operation_handle.wait_blocking();
    ///
    /// // Instead of the plaintext message, buffer now contains the ciphertext.
    /// assert_eq!(
    ///     buffer,
    ///     [
    ///         0xb3, 0xc8, 0xd2, 0x3b, 0xa7, 0x36, 0x5f, 0x18, 0x61, 0x70, 0x0, 0x3e, 0xd9, 0x3a,
    ///         0x31, 0x96,
    ///     ]
    /// );
    /// #
    /// # {after_snippet}
    /// ```
    ///
    /// ## Errors
    ///
    /// The ECB and OFB cipher modes require the data length to be a multiple of the block size
    /// (16), otherwise an error is returned.
    pub fn process_in_place<'t>(
        &'t mut self,
        buffer: &'t mut [u8],
    ) -> Result<AesHandle<'t>, Error> {
        self.validate(buffer)?;

        let data = self.frontend.data_mut();

        data.cipher_mode = NonNull::from(&self.cipher_mode);
        data.buffers = unsafe { CryptoBuffers::new_in_place(buffer).into_inner() };

        Ok(self.post())
    }
}

/// The handle to the pending AES operation.
///
/// This object is returned by [`AesContext::process`] and [`AesContext::process_in_place`].
///
/// Dropping this handle before the operation finishes will cancel the operation.
///
/// For an example, see the documentation of [`AesBackend`].
pub struct AesHandle<'t>(Handle<'t, AesOperation>);

impl AesHandle<'_> {
    /// Polls the status of the work item.
    ///
    /// This function returns `true` if the item has been processed.
    #[inline]
    pub fn poll(&mut self) -> bool {
        self.0.poll()
    }

    /// Polls the work item to completion, by busy-looping.
    ///
    /// This function returns immediately if `poll` returns `true`.
    #[inline]
    pub fn wait_blocking(self) -> Status {
        self.0.wait_blocking()
    }

    /// Waits until the work item is completed.
    #[inline]
    pub fn wait(&mut self) -> impl Future<Output = Status> {
        self.0.wait()
    }

    /// Cancels the work item and asynchronously waits until it is removed from the work queue.
    #[inline]
    pub fn cancel(&mut self) -> impl Future<Output = ()> {
        self.0.cancel()
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
