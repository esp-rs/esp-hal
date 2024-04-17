//! HMAC Accelerator
//!
//! # Overview
//!
//! The Hash-based Message Authentication Code (HMAC) module computes Message
//! Authentication Codes (MACs) using Hash algorithm and keys as described in
//! RFC 2104. The hash algorithm is SHA-256, the 256-bit HMAC key is stored in
//! an eFuse key block and can be set as read-protected, i. e., the key is not
//! accessible from outside the HMAC accelerator itself.
//!
//! The HMAC module can be used in two modes - in ”upstream” mode the HMAC
//! message is supplied by the user and the calculation result is read back by
//! the user. In ”downstream” mode the HMAC module is used as a Key Derivation
//! Function (KDF) for other internal hardwares.
//!
//! # Main features
//!
//! - Standard HMAC-SHA-256 algorithm.
//! - Hash result only accessible by configurable hardware peripheral (in
//!   downstream mode).
//! - Compatible to challenge-response authentication algorithm.
//! - Generates required keys for the Digital Signature (DS) peripheral (in
//!   downstream mode).
//! - Re-enables soft-disabled JTAG (in downstream mode).
//!
//! # Availability on ESP32 family
//!
//! The accelerator is available on ESP32-S2, ESP32-S3, ESP32-C3 and ESP32-C6.
//!
//! # HMAC padding
//!
//! The HMAC padding is handled by the driver. In
//! downstream mode, users do not need to input any message or apply padding.
//! The HMAC module uses a default 32-byte pattern of 0x00 for re-enabling JTAG
//! and a 32-byte pattern of 0xff for deriving the AES key for the DS module.

use core::convert::Infallible;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::HMAC,
    reg_access::{AlignmentHelper, SocDependentEndianess},
    system::{Peripheral as PeripheralEnable, PeripheralClockControl},
};

pub struct Hmac<'d> {
    hmac: PeripheralRef<'d, HMAC>,
    alignment_helper: AlignmentHelper<SocDependentEndianess>,
    byte_written: usize,
    next_command: NextCommand,
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
pub enum KeyId {
    Key0 = 0,
    Key1 = 1,
    Key2 = 2,
    Key3 = 3,
    Key4 = 4,
    Key5 = 5,
}

enum NextCommand {
    None,
    MessageIng,
    MessagePad,
}

impl<'d> Hmac<'d> {
    pub fn new(hmac: impl Peripheral<P = HMAC> + 'd) -> Self {
        crate::into_ref!(hmac);

        PeripheralClockControl::enable(PeripheralEnable::Sha);
        PeripheralClockControl::enable(PeripheralEnable::Hmac);

        Self {
            hmac,
            alignment_helper: AlignmentHelper::default(),
            byte_written: 64,
            next_command: NextCommand::None,
        }
    }

    pub fn free(self) -> PeripheralRef<'d, HMAC> {
        self.hmac
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
    pub fn configure(&mut self, m: HmacPurpose, key_id: KeyId) -> nb::Result<(), Error> {
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
            return Err(nb::Error::Other(Error::KeyPurposeMismatch));
        }

        Ok(())
    }

    /// Process the msg block after block
    ///
    /// Call this function as many times as necessary (msg.len() > 0)
    pub fn update<'a>(&mut self, msg: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        self.next_command();

        let remaining = self.write_data(msg).unwrap();

        Ok(remaining)
    }

    pub fn finalize(&mut self, output: &mut [u8]) -> nb::Result<(), Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

        self.next_command();

        let msg_len = self.byte_written as u64;

        nb::block!(self.write_data(&[0x80])).unwrap();
        nb::block!(self.flush_data()).unwrap();
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
        Ok(())
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

    fn write_data<'a>(&mut self, incoming: &'a [u8]) -> nb::Result<&'a [u8], Infallible> {
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

        Ok(remaining)
    }

    fn flush_data(&mut self) -> nb::Result<(), Infallible> {
        if self.is_busy() {
            return Err(nb::Error::WouldBlock);
        }

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

        Ok(())
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
