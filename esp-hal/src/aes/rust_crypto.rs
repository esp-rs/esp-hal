use core::{cell::RefCell, fmt};

use cipher::{
    AlgorithmName,
    Block,
    BlockCipherDecBackend,
    BlockCipherDecClosure,
    BlockCipherDecrypt,
    BlockCipherEncBackend,
    BlockCipherEncClosure,
    BlockCipherEncrypt,
    BlockSizeUser,
    InOut,
    Key as CipherKey,
    KeyInit,
    KeySizeUser,
    ParBlocksSizeUser,
    consts::{U1, U16},
};

use super::{AesContext, Operation, cipher_modes::Ecb};

struct RustCryptoBackend<'a> {
    context: &'a RefCell<AesContext>,
    operation: Operation,
}

impl BlockSizeUser for RustCryptoBackend<'_> {
    type BlockSize = U16;
}

impl ParBlocksSizeUser for RustCryptoBackend<'_> {
    type ParBlocksSize = U1;
}

impl RustCryptoBackend<'_> {
    fn process_block(&self, block: InOut<'_, '_, Block<Self>>) {
        let output = block.into_out_with_copied_in();
        let mut context = self.context.borrow_mut();
        context.set_operation(self.operation);
        context
            .process_in_place(output.as_mut())
            .expect("AES blocks are always the required size")
            .wait_blocking();
    }
}

impl BlockCipherEncBackend for RustCryptoBackend<'_> {
    fn encrypt_block(&self, block: InOut<'_, '_, Block<Self>>) {
        debug_assert_eq!(self.operation, Operation::Encrypt);
        self.process_block(block);
    }
}

impl BlockCipherDecBackend for RustCryptoBackend<'_> {
    fn decrypt_block(&self, block: InOut<'_, '_, Block<Self>>) {
        debug_assert_eq!(self.operation, Operation::Decrypt);
        self.process_block(block);
    }
}

macro_rules! impl_aes {
    ($name:ident, $bits:literal, $key_size:ty, $key_bytes:literal) => {
        #[doc = concat!("Hardware-accelerated AES-", stringify!($bits), " block cipher.")]
        ///
        /// Operations are submitted to the global AES work queue. An [`AesBackend`](super::AesBackend)
        /// or, on supported chips, an `AesDmaBackend` must be started before calling the blocking
        /// [`cipher`] trait methods.
        ///
        /// This type implements the low-level AES block primitive. Use a RustCrypto mode or AEAD
        /// crate when encrypting application data.
        #[derive(Clone)]
        pub struct $name {
            context: RefCell<AesContext>,
        }

        impl KeySizeUser for $name {
            type KeySize = $key_size;
        }

        impl KeyInit for $name {
            fn new(key: &CipherKey<Self>) -> Self {
                let mut key_bytes = [0; $key_bytes];
                key_bytes.copy_from_slice(key.as_ref());

                Self {
                    context: RefCell::new(AesContext::new(Ecb, Operation::Encrypt, key_bytes)),
                }
            }
        }

        impl BlockSizeUser for $name {
            type BlockSize = U16;
        }

        impl BlockCipherEncrypt for $name {
            fn encrypt_with_backend(
                &self,
                f: impl BlockCipherEncClosure<BlockSize = Self::BlockSize>,
            ) {
                f.call(&RustCryptoBackend {
                    context: &self.context,
                    operation: Operation::Encrypt,
                });
            }
        }

        impl BlockCipherDecrypt for $name {
            fn decrypt_with_backend(
                &self,
                f: impl BlockCipherDecClosure<BlockSize = Self::BlockSize>,
            ) {
                f.call(&RustCryptoBackend {
                    context: &self.context,
                    operation: Operation::Decrypt,
                });
            }
        }

        impl AlgorithmName for $name {
            fn write_alg_name(f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(stringify!($name))
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.write_str(concat!(stringify!($name), " { ... }"))
            }
        }
    };
}

for_each_aes_key_length! {
    (128) => {
        impl_aes!(Aes128, 128, cipher::consts::U16, 16);
    };
    (192) => {
        impl_aes!(Aes192, 192, cipher::consts::U24, 24);
    };
    (256) => {
        impl_aes!(Aes256, 256, cipher::consts::U32, 32);
    };
}
