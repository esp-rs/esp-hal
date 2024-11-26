//! # Inter-Integrated Circuit (I2C)

#![allow(unused)] // TODO: Remove me when `embedded_hal::i2c::I2c` is implemented

use crate::pac::{lp_i2c0::COMD, LP_I2C0};

const LP_I2C_TRANS_COMPLETE_INT_ST_S: u32 = 7;
const LP_I2C_END_DETECT_INT_ST_S: u32 = 3;
const LP_I2C_NACK_INT_ST_S: u32 = 10;

const I2C_LL_INTR_MASK: u32 = (1 << LP_I2C_TRANS_COMPLETE_INT_ST_S)
    | (1 << LP_I2C_END_DETECT_INT_ST_S)
    | (1 << LP_I2C_NACK_INT_ST_S);

const LP_I2C_FIFO_LEN: u32 = 16;

#[doc(hidden)]
pub unsafe fn conjure() -> LpI2c {
    LpI2c {
        i2c: LP_I2C0::steal(),
    }
}

// TODO: Document enum variants
/// I2C-specific transmission errors
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    ExceedingFifo,
    AckCheckFailed,
    TimeOut,
    ArbitrationLost,
    ExecIncomplete,
    CommandNrExceeded,
    InvalidResponse,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OperationType {
    Write = 0,
    Read  = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Ack {
    Ack,
    Nack,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Opcode {
    RStart = 6,
    Write  = 1,
    Read   = 3,
    Stop   = 2,
    End    = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    Start,
    Stop,
    End,
    Write {
        /// This bit is to set an expected ACK value for the transmitter.
        ack_exp: Ack,
        /// Enables checking the ACK value received against the ack_exp value.
        ack_check_en: bool,
        /// Length of data (in bytes) to be written. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
    Read {
        /// Indicates whether the receiver will send an ACK after this byte has
        /// been received.
        ack_value: Ack,
        /// Length of data (in bytes) to be read. The maximum length is 255,
        /// while the minimum is 1.
        length: u8,
    },
}

impl From<Command> for u16 {
    fn from(c: Command) -> u16 {
        let opcode = match c {
            Command::Start => Opcode::RStart,
            Command::Stop => Opcode::Stop,
            Command::End => Opcode::End,
            Command::Write { .. } => Opcode::Write,
            Command::Read { .. } => Opcode::Read,
        };

        let length = match c {
            Command::Start | Command::Stop | Command::End => 0,
            Command::Write { length: l, .. } | Command::Read { length: l, .. } => l,
        };

        let ack_exp = match c {
            Command::Start | Command::Stop | Command::End | Command::Read { .. } => Ack::Nack,
            Command::Write { ack_exp: exp, .. } => exp,
        };

        let ack_check_en = match c {
            Command::Start | Command::Stop | Command::End | Command::Read { .. } => false,
            Command::Write {
                ack_check_en: en, ..
            } => en,
        };

        let ack_value = match c {
            Command::Start | Command::Stop | Command::End | Command::Write { .. } => Ack::Nack,
            Command::Read { ack_value: ack, .. } => ack,
        };

        let mut cmd: u16 = length.into();

        if ack_check_en {
            cmd |= 1 << 8;
        } else {
            cmd &= !(1 << 8);
        }

        if ack_exp == Ack::Nack {
            cmd |= 1 << 9;
        } else {
            cmd &= !(1 << 9);
        }

        if ack_value == Ack::Nack {
            cmd |= 1 << 10;
        } else {
            cmd &= !(1 << 10);
        }

        cmd |= (opcode as u16) << 11;

        cmd
    }
}

impl From<Command> for u32 {
    fn from(c: Command) -> u32 {
        u16::from(c) as u32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CommandRegister {
    COMD0,
    COMD1,
    COMD2,
    COMD3,
    COMD4,
    COMD5,
    COMD6,
    COMD7,
}

impl CommandRegister {
    fn advance(&mut self) {
        *self = match *self {
            CommandRegister::COMD0 => CommandRegister::COMD1,
            CommandRegister::COMD1 => CommandRegister::COMD2,
            CommandRegister::COMD2 => CommandRegister::COMD3,
            CommandRegister::COMD3 => CommandRegister::COMD4,
            CommandRegister::COMD4 => CommandRegister::COMD5,
            CommandRegister::COMD5 => CommandRegister::COMD6,
            CommandRegister::COMD6 => CommandRegister::COMD7,
            CommandRegister::COMD7 => panic!("Cannot advance beyond COMD7"),
        }
    }
}

// https://github.com/espressif/esp-idf/blob/master/components/ulp/lp_core/lp_core_i2c.c#L122
// TX/RX RAM size is 16*8 bit
// TX RX FIFO has 16 bit depth
// The clock source of APB_CLK in LP_I2C is CLK_AON_FAST.
// Configure LP_I2C_SCLK_SEL to select the clock source for I2C_SCLK.
// When LP_I2C_SCLK_SEL is 0, select CLK_ROOT_FAST as clock source,
// and when LP_I2C_SCLK_SEL is 1, select CLK _XTALD2 as the clock source.
// Configure LP_EXT_I2C_CK_EN high to enable the clock source of I2C_SCLK.
// Adjust the timing registers accordingly when the clock frequency changes.

/// LP-I2C driver
pub struct LpI2c {
    i2c: LP_I2C0,
}

impl LpI2c {
    /// Writes bytes to slave with address `addr`
    pub fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        let mut cmd_iterator = CommandRegister::COMD0;

        // If SCL is busy, reset the Master FSM
        if self.i2c.sr().read().bus_busy().bit_is_set() {
            self.i2c.ctr().modify(|_, w| w.fsm_rst().set_bit());
        }

        if bytes.len() > 255 {
            return Err(Error::ExceedingFifo);
        }

        // Reset FIFO and command list
        self.reset_fifo();

        self.add_cmd_lp(&mut cmd_iterator, Command::Start)?;

        // Load device address and R/W bit into FIFO
        self.write_fifo(addr << 1 | OperationType::Write as u8);

        self.add_cmd_lp(
            &mut cmd_iterator,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 1_u8,
            },
        )?;

        self.enable_interrupts(I2C_LL_INTR_MASK);

        let mut data_idx = 0;
        let mut remaining_bytes = bytes.len() as u32;

        let mut fifo_available = LP_I2C_FIFO_LEN - 1;

        while remaining_bytes > 0 {
            let fifo_size = if remaining_bytes < fifo_available {
                remaining_bytes
            } else {
                fifo_available
            };
            remaining_bytes -= fifo_size;

            // Write data to the FIFO
            for &byte in &bytes[data_idx as usize..(data_idx as usize) + fifo_size as usize] {
                self.write_fifo(byte);
            }

            // Add a Write command with the specified length
            self.add_cmd_lp(
                &mut cmd_iterator,
                Command::Write {
                    ack_exp: Ack::Ack,
                    ack_check_en: true,
                    length: fifo_size as u8,
                },
            )?;

            // Check if this is the last chunk
            let cmd = if remaining_bytes == 0 {
                Command::Stop
            } else {
                Command::End
            };

            // Add the Stop/End command
            self.add_cmd_lp(&mut cmd_iterator, cmd)?;

            // Start the I2C transaction
            self.lp_i2c_update();
            self.i2c.ctr().modify(|_, w| w.trans_start().set_bit());

            // Wait for the transaction to complete
            self.wait_for_completion()?;

            // Update the index for the next data chunk
            data_idx += fifo_size;

            fifo_available = LP_I2C_FIFO_LEN;
        }

        Ok(())
    }

    /// Reads enough bytes from slave with `addr` to fill `buffer`
    pub fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // Check size constraints
        if buffer.len() > 254 {
            return Err(Error::ExceedingFifo);
        }

        let mut cmd_iterator = CommandRegister::COMD0;

        self.add_cmd_lp(&mut cmd_iterator, Command::Start)?;

        // Load device address
        self.write_fifo(addr << 1 | OperationType::Read as u8);

        self.add_cmd_lp(
            &mut cmd_iterator,
            Command::Write {
                ack_exp: Ack::Ack,
                ack_check_en: true,
                length: 1_u8,
            },
        )?;

        self.enable_interrupts(
            1 << LP_I2C_TRANS_COMPLETE_INT_ST_S | 1 << LP_I2C_END_DETECT_INT_ST_S,
        );

        let mut remaining_bytes = buffer.len();

        while remaining_bytes > 0 {
            let fifo_size = if remaining_bytes < LP_I2C_FIFO_LEN as usize {
                remaining_bytes
            } else {
                LP_I2C_FIFO_LEN as usize
            };
            remaining_bytes -= fifo_size;

            if fifo_size == 1 {
                // Read one byte and send NACK
                self.add_cmd_lp(
                    &mut cmd_iterator,
                    Command::Read {
                        ack_value: Ack::Nack,
                        length: 1, // which is `fifo_size`
                    },
                )?;
                // Send STOP command after reading
                self.add_cmd_lp(&mut cmd_iterator, Command::Stop)?;
            } else if fifo_size > 1 && remaining_bytes == 0 {
                // This means it is the last transaction
                // Read all but the last byte and send ACKs
                self.add_cmd_lp(
                    &mut cmd_iterator,
                    Command::Read {
                        ack_value: Ack::Ack,
                        length: (fifo_size - 1) as u8,
                    },
                )?;
                // Read the last byte and send NACK
                self.add_cmd_lp(
                    &mut cmd_iterator,
                    Command::Read {
                        ack_value: Ack::Nack,
                        length: 1,
                    },
                )?;
                // Send STOP command after reading
                self.add_cmd_lp(&mut cmd_iterator, Command::Stop)?;
            } else {
                // This means we have to read data more than we can fit into the Rx FIFO
                // Read fifo_size bytes and send ACKs
                self.add_cmd_lp(
                    &mut cmd_iterator,
                    Command::Read {
                        ack_value: Ack::Ack,
                        length: fifo_size as u8,
                    },
                )?;
                // Send END command signaling more data to come
                self.add_cmd_lp(&mut cmd_iterator, Command::End)?;
            }

            self.lp_i2c_update();

            // Initiate I2C transfer
            self.i2c.ctr().modify(|_, w| w.trans_start().set_bit());

            // Await for completion (This function or mechanism should handle waiting for
            // the I2C transfer to complete)
            self.wait_for_completion()?;

            // Read from FIFO into the current chunk
            for byte in buffer.iter_mut() {
                *byte = self.read_fifo();
            }
        }
        Ok(())
    }

    /// Writes bytes to slave with address `addr` and then reads enough bytes
    /// to fill `buffer` *in a single transaction*
    pub fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        // It would be possible to combine the write and read in one transaction, but
        // filling the tx fifo with the current code is somewhat slow even in release
        // mode which can cause issues.
        self.write(addr, bytes)?;
        self.read(addr, buffer)?;

        Ok(())
    }

    fn lp_i2c_update(&self) {
        self.i2c.ctr().modify(|_, w| w.conf_upgate().set_bit());
    }

    fn reset_fifo(&self) {
        self.i2c
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().set_bit());

        self.i2c
            .fifo_conf()
            .modify(|_, w| w.tx_fifo_rst().clear_bit());

        self.i2c
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().set_bit());

        self.i2c
            .fifo_conf()
            .modify(|_, w| w.rx_fifo_rst().clear_bit());
    }

    fn wait_for_completion(&self) -> Result<(), Error> {
        loop {
            let interrupts = self.i2c.int_st().read();

            // Handle completion cases
            // A full transmission was completed
            if interrupts.nack().bit_is_set() {
                self.i2c
                    .int_clr()
                    .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
                return Err(Error::InvalidResponse);
            } else if interrupts.trans_complete().bit_is_set() {
                self.disable_interrupts();

                self.i2c
                    .int_clr()
                    .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
                break;
            } else if interrupts.end_detect().bit_is_set() {
                self.i2c
                    .int_clr()
                    .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
                break;
            }
        }

        Ok(())
    }

    fn enable_interrupts(&self, mask: u32) {
        self.i2c.int_ena().write(|w| unsafe { w.bits(mask) });
    }

    fn disable_interrupts(&self) {
        self.i2c.int_ena().write(|w| unsafe { w.bits(0) });
    }

    fn write_fifo(&self, data: u8) {
        self.i2c
            .data()
            .write(|w| unsafe { w.fifo_rdata().bits(data) });
    }

    fn read_fifo(&self) -> u8 {
        self.i2c.data().read().fifo_rdata().bits()
    }

    fn add_cmd_lp(
        &self,
        command_register: &mut CommandRegister,
        command: Command,
    ) -> Result<(), Error> {
        self.i2c
            .comd(*command_register as usize)
            .write(|w| unsafe { w.command().bits(command.into()) });

        command_register.advance();

        Ok(())
    }
}
