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
            CipherModeState::Ecb(algo) => algo.encrypt_decrypt(work_item.buffers, process_block),
            CipherModeState::Cbc(algo) => {
                if work_item.mode == Operation::Encrypt {
                    algo.encrypt(work_item.buffers, process_block);
                } else {
                    algo.decrypt(work_item.buffers, process_block);
                }
            }
            CipherModeState::Ofb(algo) => algo.encrypt_decrypt(work_item.buffers, process_block),
            CipherModeState::Ctr(algo) => algo.encrypt_decrypt(work_item.buffers, process_block),
            CipherModeState::Cfb8(algo) => {
                if work_item.mode == Operation::Encrypt {
                    algo.encrypt(work_item.buffers, process_block)
                } else {
                    algo.decrypt(work_item.buffers, process_block)
                }
            }
            CipherModeState::Cfb128(algo) => {
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
        fn write_key(&mut self, key: impl Into<Key>) {
            let key = key.into();
            let key = key.as_slice();
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
pub enum CipherModeState {
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

impl From<cipher_modes::Ecb> for CipherModeState {
    fn from(value: cipher_modes::Ecb) -> Self {
        Self::Ecb(value)
    }
}

impl From<cipher_modes::Cbc> for CipherModeState {
    fn from(value: cipher_modes::Cbc) -> Self {
        Self::Cbc(value)
    }
}

impl From<cipher_modes::Ofb> for CipherModeState {
    fn from(value: cipher_modes::Ofb) -> Self {
        Self::Ofb(value)
    }
}

impl From<cipher_modes::Ctr> for CipherModeState {
    fn from(value: cipher_modes::Ctr) -> Self {
        Self::Ctr(value)
    }
}

impl From<cipher_modes::Cfb8> for CipherModeState {
    fn from(value: cipher_modes::Cfb8) -> Self {
        Self::Cfb8(value)
    }
}

impl From<cipher_modes::Cfb128> for CipherModeState {
    fn from(value: cipher_modes::Cfb128) -> Self {
        Self::Cfb128(value)
    }
}

impl CipherModeState {
    fn software_operating_mode(&self, operation: Operation, key: &Key) -> Mode {
        match self {
            CipherModeState::Ecb(_) | CipherModeState::Cbc(_) => {
                if operation == Operation::Encrypt {
                    key.encrypt_mode()
                } else {
                    key.decrypt_mode()
                }
            }
            // For these, decryption is handled in software using the hardware in ecryption mode to
            // produce intermediate results.
            CipherModeState::Ofb(_)
            | CipherModeState::Ctr(_)
            | CipherModeState::Cfb8(_)
            | CipherModeState::Cfb128(_) => key.encrypt_mode(),
        }
    }

    fn requires_blocks(&self) -> bool {
        matches!(
            self,
            CipherModeState::Ecb(_) | CipherModeState::Ofb(_) | CipherModeState::Cbc(_)
        )
    }
}

const BLOCK_SIZE: usize = 16;

struct AesOperation {
    mode: Operation,
    // The block cipher operating mode.
    cipher_mode: NonNull<CipherModeState>,
    // The length of the key is determined by the operating mode.
    buffers: UnsafeCryptoBuffers,
    key: Key,
}

// Safety: AesOperation is safe to share between threads, in the context of a WorkQueue. The
// WorkQueue ensures that only a single location can access the data. All the internals, except
// for the pointers, are Sync. The pointers are safe to share because they point at data that the
// AES driver ensures can be accessed safely and soundly.
unsafe impl Sync for AesOperation {}

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
/// Due to how this backend works, it provides no `run` method(s). Posting work to this backend will
/// immediately be executed, in a blocking way. The backend only needs to be kept alive in order to
/// function.
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
/// let mut ecb_encrypt = AesContext::new(
///     Ecb,
///     Operation::Encrypt,
///     [
///         b'S', b'U', b'p', b'4', b'S', b'e', b'C', b'p', b'@', b's', b'S', b'w', b'0', b'r',
///         b'd', 0,
///     ],
/// );
///
/// // Process a block of data in this context. The ECB operating mode requires that
/// // the length of the data is a multiple of the block (16 bytes) size.
/// let input_buffer: [u8; 16] = [
///     b'm', b'e', b's', b's', b'a', b'g', b'e', 0, 0, 0, 0, 0, 0, 0, 0, 0,
/// ];
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
            _inner: WorkQueueDriver::new(self, BLOCKING_AES_VTABLE, &AES_WORK_QUEUE),
        }
    }

    // WorkQueue callbacks. They may run in any context.

    unsafe fn from_raw<'any>(ptr: *const ()) -> &'any mut Self {
        unsafe { unwrap!(ptr.cast_mut().cast::<AesBackend<'_>>().as_mut()) }
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
    _inner: WorkQueueDriver<'t, AesBackend<'d>, AesOperation>,
}

/// An AES work queue user.
pub struct AesContext {
    cipher_mode: CipherModeState,
    frontend: WorkQueueFrontend<AesOperation>,
}

impl AesContext {
    /// Creates a new context to encrypt or decrypt data with the given block cipher operating
    /// mode.
    pub fn new(
        cipher_mode: impl Into<CipherModeState>,
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
    /// let mut ecb_encrypt = AesContext::new(
    ///     Ecb,
    ///     Operation::Encrypt,
    ///     [
    ///         b'S', b'U', b'p', b'4', b'S', b'e', b'C', b'p', b'@', b's', b'S', b'w', b'0', b'r',
    ///         b'd', 0,
    ///     ],
    /// );
    ///
    /// // Process a block of data in this context, in place. The ECB operating mode requires that
    /// // the length of the data is a multiple of the block (16 bytes) size.
    /// let mut buffer: [u8; 16] = [
    ///     b'm', b'e', b's', b's', b'a', b'g', b'e', 0, 0, 0, 0, 0, 0, 0, 0, 0,
    /// ];
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
