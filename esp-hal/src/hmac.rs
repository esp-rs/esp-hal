//! # Hash-based Message Authentication Code (HMAC) Accelerator
//!
//! ## Overview
//! HMAC is a secure authentication technique that verifies the authenticity and
//! integrity of a message with a pre-shared key. This module provides hardware
//! acceleration for SHA256-HMAC generation using a key burned into an eFuse
//! block.
//!
//! Main features:
//!
//! - Standard HMAC-SHA-256 algorithm.
//! - Hash result only accessible by configurable hardware peripheral (in
//!   downstream mode).
//! - Compatible to challenge-response authentication algorithm.
//! - Generates required keys for the Digital Signature (DS) peripheral (in
//!   downstream mode).
//! - Re-enables soft-disabled JTAG (in downstream mode).
//!
//! ## Configuration
//! The HMAC module can be used in two modes - in ”upstream” mode the HMAC
//! message is supplied by the user and the calculation result is read back by
//! the user. In ”downstream” mode the HMAC module is used as a Key Derivation
//! Function (KDF) for other internal hardwares.
//!
//! ### HMAC padding
//!
//! The HMAC padding is handled by the driver. In downstream mode, users do not
//! need to input any message or apply padding. The HMAC module uses a default
//! 32-byte pattern of 0x00 for re-enabling JTAG and a 32-byte pattern of 0xff
//! for deriving the AES key for the DS module.
//!
//! ## Examples
//! Visit the [HMAC] example to learn how to use the HMAC accelerator
//!
//! [HMAC]: https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/hmac.rs

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::HMAC,
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::{GenericPeripheralGuard, Peripheral as PeripheralEnable},
};

/// Provides an interface for interacting with the HMAC hardware peripheral.
/// It allows users to compute HMACs for cryptographic purposes, ensuring data
/// integrity and authenticity.
pub struct Hmac<'d> {
    hmac: PeripheralRef<'d, HMAC>,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    byte_written: usize,
    next_command: NextCommand,
    _guard: GenericPeripheralGuard<{ PeripheralEnable::Hmac as u8 }>,
}

/// HMAC interface error
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// It means the purpose of the selected block does not match the
    /// configured key purpose and the calculation will not proceed.
    KeyPurposeMismatch,
}

/// The peripheral can be configured to deliver its output directly to the
/// user. It can also deliver to other peripherals.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "peripheral is unstable")]
pub enum HmacPurpose {
    /// HMAC is used to re-enable JTAG after soft-disabling it.
    ToJtag     = 6,
    /// HMAC is provided to the digital signature peripheral to decrypt the
    /// private key.
    ToDs       = 7,
    /// Let the user provide a message and read the result.
    ToUser     = 8,
    /// HMAC is used for both the digital signature or JTAG.
    ToDsOrJtag = 5,
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Represents the key identifiers for the HMAC peripheral.
pub enum KeyId {
    /// Key 0.
    Key0 = 0,
    /// Key 1.
    Key1 = 1,
    /// Key 2.
    Key2 = 2,
    /// Key 3.
    Key3 = 3,
    /// Key 4.
    Key4 = 4,
    /// Key 5.
    Key5 = 5,
}

enum NextCommand {
    None,
    MessageIng,
    MessagePad,
}

impl<'d> Hmac<'d> {
    /// Creates a new instance of the HMAC peripheral.
    pub fn new(hmac: impl Peripheral<P = HMAC> + 'd) -> Self {
        crate::into_ref!(hmac);

        let guard = GenericPeripheralGuard::new();

        Self {
            hmac,
            alignment_helper: AlignmentHelper::default(),
            byte_written: 64,
            next_command: NextCommand::None,
            _guard: guard,
        }
    }

    /// Step 1. Enable HMAC module.
    ///
    /// Before these steps, the user shall set the peripheral clocks bits for
    /// HMAC and SHA peripherals and clear the corresponding peripheral
    /// reset bits.
    pub fn init(&mut self) {
        self.hmac.set_start().write(|w| w.set_start().set_bit());
        self.alignment_helper.reset();
        self.byte_written = 64;
        self.next_command = NextCommand::None;
    }

    /// Step 2. Configure HMAC keys and key purposes.
    pub fn configure(&mut self, m: HmacPurpose, key_id: KeyId) -> Result<(), Error> {
        self.hmac
            .set_para_purpose()
            .write(|w| unsafe { w.purpose_set().bits(m as u8) });
        self.hmac
            .set_para_key()
            .write(|w| unsafe { w.key_set().bits(key_id as u8) });
        self.hmac
            .set_para_finish()
            .write(|w| w.set_para_end().set_bit());

        if self.hmac.query_error().read().query_check().bit_is_set() {
            return Err(Error::KeyPurposeMismatch);
        }

        Ok(())
    }

    /// Process the msg block after block
    pub fn update<'a>(&mut self, msg: &'a [u8]) -> &'a [u8] {
        while (&mut *self).is_busy() {}

        self.next_command();

        let remaining = self.write_data(msg);

        remaining
    }

    /// Finalizes the HMAC computation and retrieves the resulting hash output.
    pub fn finalize(&mut self, output: &mut [u8]) {
        while (&mut *self).is_busy() {}

        self.next_command();

        let msg_len = self.byte_written as u64;

        self.write_data(&[0x80]);
        self.flush_data();
        self.next_command();
        debug_assert!(self.byte_written % 4 == 0);

        self.padding(msg_len);

        // Checking if the message is one block including padding
        if msg_len < 64 + 56 {
            self.hmac.one_block().write(|w| w.set_one_block().set_bit());

            while self.is_busy() {}
        }

        self.alignment_helper.volatile_read_regset(
            #[cfg(esp32s2)]
            self.hmac.rd_result_(0).as_ptr(),
            #[cfg(not(esp32s2))]
            self.hmac.rd_result_mem(0).as_ptr(),
            output,
            core::cmp::min(output.len(), 32) / self.alignment_helper.align_size(),
        );

        self.hmac
            .set_result_finish()
            .write(|w| w.set_result_end().set_bit());
        self.byte_written = 64;
        self.next_command = NextCommand::None;
    }

    fn is_busy(&mut self) -> bool {
        self.hmac.query_busy().read().busy_state().bit_is_set()
    }

    fn next_command(&mut self) {
        match self.next_command {
            NextCommand::MessageIng => {
                self.hmac
                    .set_message_ing()
                    .write(|w| w.set_text_ing().set_bit());
            }
            NextCommand::MessagePad => {
                self.hmac
                    .set_message_pad()
                    .write(|w| w.set_text_pad().set_bit());
            }
            NextCommand::None => {}
        }
        self.next_command = NextCommand::None;
    }

    fn write_data<'a>(&mut self, incoming: &'a [u8]) -> &'a [u8] {
        let mod_length = self.byte_written % 64;

        let (remaining, bound_reached) = self.alignment_helper.aligned_volatile_copy(
            #[cfg(esp32s2)]
            self.hmac.wr_message_(0).as_ptr(),
            #[cfg(not(esp32s2))]
            self.hmac.wr_message_mem(0).as_ptr(),
            incoming,
            64 / self.alignment_helper.align_size(),
            mod_length / self.alignment_helper.align_size(),
        );

        self.byte_written = self
            .byte_written
            .wrapping_add(incoming.len() - remaining.len());

        if bound_reached {
            self.hmac
                .set_message_one()
                .write(|w| w.set_text_one().set_bit());

            if remaining.len() >= 56 {
                self.next_command = NextCommand::MessageIng;
            } else {
                self.next_command = NextCommand::MessagePad;
            }
        }

        remaining
    }

    fn flush_data(&mut self) {
        while self.is_busy() {}

        let flushed = self.alignment_helper.flush_to(
            #[cfg(esp32s2)]
            self.hmac.wr_message_(0).as_ptr(),
            #[cfg(not(esp32s2))]
            self.hmac.wr_message_mem(0).as_ptr(),
            (self.byte_written % 64) / self.alignment_helper.align_size(),
        );

        self.byte_written = self.byte_written.wrapping_add(flushed);
        if flushed > 0 && self.byte_written % 64 == 0 {
            self.hmac
                .set_message_one()
                .write(|w| w.set_text_one().set_bit());
            while self.is_busy() {}
            self.next_command = NextCommand::MessagePad;
        }
    }

    fn padding(&mut self, msg_len: u64) {
        let mod_cursor = self.byte_written % 64;

        // The padding will be spanned over 2 blocks
        if mod_cursor > 56 {
            let pad_len = 64 - mod_cursor;
            self.alignment_helper.volatile_write_bytes(
                #[cfg(esp32s2)]
                self.hmac.wr_message_(0).as_ptr(),
                #[cfg(not(esp32s2))]
                self.hmac.wr_message_mem(0).as_ptr(),
                0_u8,
                pad_len / self.alignment_helper.align_size(),
                mod_cursor / self.alignment_helper.align_size(),
            );
            self.hmac
                .set_message_one()
                .write(|w| w.set_text_one().set_bit());
            self.byte_written = self.byte_written.wrapping_add(pad_len);
            debug_assert!(self.byte_written % 64 == 0);
            while self.is_busy() {}
            self.next_command = NextCommand::MessagePad;
            self.next_command();
        }

        let mod_cursor = self.byte_written % 64;
        let pad_len = 64 - mod_cursor - core::mem::size_of::<u64>();

        self.alignment_helper.volatile_write_bytes(
            #[cfg(esp32s2)]
            self.hmac.wr_message_(0).as_ptr(),
            #[cfg(not(esp32s2))]
            self.hmac.wr_message_mem(0).as_ptr(),
            0_u8,
            pad_len / self.alignment_helper.align_size(),
            mod_cursor / self.alignment_helper.align_size(),
        );

        self.byte_written = self.byte_written.wrapping_add(pad_len);

        assert_eq!(self.byte_written % 64, 64 - core::mem::size_of::<u64>());

        // Add padded key
        let len_mem = (msg_len * 8).to_be_bytes();

        self.alignment_helper.aligned_volatile_copy(
            #[cfg(esp32s2)]
            self.hmac.wr_message_(0).as_ptr(),
            #[cfg(not(esp32s2))]
            self.hmac.wr_message_mem(0).as_ptr(),
            &len_mem,
            64 / self.alignment_helper.align_size(),
            (64 - core::mem::size_of::<u64>()) / self.alignment_helper.align_size(),
        );

        self.hmac
            .set_message_one()
            .write(|w| w.set_text_one().set_bit());

        while self.is_busy() {}
    }
}
