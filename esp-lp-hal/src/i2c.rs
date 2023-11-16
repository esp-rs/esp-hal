//! Low-power I2C driver

use esp32c6_lp::{LP_AON, LP_CLKRST, LP_I2C, LP_IO, LP_PERI};
use fugit::HertzU32;

use crate::{gpio::GpioPin, CPU_CLOCK};

const RTC_XTAL_FREQ_REG: u32 = 0x600B1000 + 0x10;
const LPPERI_CLK_EN_REG: u32 = 0x600B2800 + 0x0;
const LPPERI_RESET_EN_REG: u32 = 0x600B2800 + 0x4;
const LPPERI_LP_EXT_I2C_CK_EN: u32 = 1 << 28;
const LPPERI_LP_EXT_I2C_RESET_EN: u32 = 1 << 28;

const LP_I2C_FILTER_CYC_NUM_DEF: u8 = 7;

const LP_I2C_TRANS_COMPLETE_INT_ST_S: u32 = 7;
const LP_I2C_END_DETECT_INT_ST_S: u32 = 3;
const LP_I2C_NACK_INT_ST_S: u32 = 10;

const I2C_LL_INTR_MASK: u32 = (1 << LP_I2C_TRANS_COMPLETE_INT_ST_S)
    | (1 << LP_I2C_END_DETECT_INT_ST_S)
    | (1 << LP_I2C_NACK_INT_ST_S);

const LP_I2C_FIFO_LEN: u32 = 16; // TX RX FIFO depth on esp32c6

#[doc(hidden)]
pub unsafe fn conjour() -> Option<LpI2c> {
    Some(LpI2c {
        i2c: LP_I2C::steal(),
    })
}

/// I2C-specific transmission errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    ExceedingFifo,
    AckCheckFailed,
    TimeOut,
    ArbitrationLost,
    ExecIncomplete,
    CommandNrExceeded,
    InvalidResponse,
}

 
#[allow(unused)]
enum OperationType {
    Write = 0,
    Read  = 1,
}

#[allow(unused)]
#[derive(Eq, PartialEq, Copy, Clone)]
enum Ack {
    Ack,
    Nack,
}

#[allow(unused)]
enum Opcode {
    RStart = 6,
    Write  = 1,
    Read   = 3,
    Stop   = 2,
    End    = 4,
}

#[derive(PartialEq)]
#[allow(unused)]
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

        let mut cmd: u32 = length.into();

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

        cmd |= (opcode as u32) << 11;

        cmd
    }
}

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


pub struct LpI2c {
    i2c: LP_I2C,
}
 
impl<'d> LpI2c {
    /// Send data bytes from the `bytes` array to a target slave with the
    /// address `addr`
    fn master_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        let mut cmd_iterator = CommandRegister::COMD0;

        // If SCL is busy, reset the Master FSM
        if self.i2c.sr().read().bus_busy().bit_is_set() {
            self.i2c
                .ctr()
                .modify(|_, w| unsafe { w.fsm_rst().set_bit() });
        }

        if bytes.len() > 255 {
            return Err(Error::ExceedingFifo);
        }

        // Reset FIFO and command list
        self.reset_fifo();

        self.add_cmd_lp(&mut cmd_iterator, Command::Start)?;

        // Load device address and R/W bit into FIFO
        self.write_fifo(((addr & 0xFF) << 1) | ((OperationType::Write as u8) << 0));

        self.add_cmd_lp(
            &mut cmd_iterator,
            Command::Write {
                ack_exp: Ack::Ack, // TODO Nack???
                ack_check_en: true,
                length: 1 as u8,
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
            remaining_bytes -= fifo_size as u32;

            // Write data to the FIFO
            for &byte in &bytes[data_idx as usize..(data_idx as usize) + fifo_size as usize] {
                self.write_fifo(byte);
            }

            // Add a Write command with the specified length
            self.add_cmd_lp(
                &mut cmd_iterator,
                Command::Write {
                    ack_exp: Ack::Ack, //TODO Nack???
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

    #[allow(unused)]
    fn master_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // Check size constraints
        if buffer.len() > 254 {
            return Err(Error::ExceedingFifo);
        }

        let mut cmd_iterator = CommandRegister::COMD0;

        self.add_cmd_lp(&mut cmd_iterator, Command::Start)?;

        // Load device address
        // self.write_fifo(addr << 1 | OperationType::Read as u8);
        self.write_fifo(((addr & 0xFF) << 1) | ((OperationType::Read as u8) << 0));

        self.add_cmd_lp(
            &mut cmd_iterator,
            Command::Write {
                ack_exp: Ack::Ack, // TODO Nack???
                ack_check_en: true,
                length: 1 as u8,
            },
        )?;

        self.enable_interrupts(1 << 7 | 1 << 3);

        let mut fifo_size = 0;
        let mut data_idx: usize = 0;
        let mut remaining_bytes = buffer.len();

        // for chunk in buffer.chunks_mut(LP_I2C_FIFO_LEN as usize) {
            // let fifo_size = chunk.len();
        while remaining_bytes > 0 {
            let fifo_size = if remaining_bytes < LP_I2C_FIFO_LEN as usize{
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
            // for byte in chunk.iter_mut() {
            //     *byte = self.i2c.data().read().fifo_rdata().bits();
            // }
            for byte in buffer.iter_mut() {
                *byte = self.read_fifo();
            }

            data_idx += fifo_size;
        }
        Ok(())
    }

    fn master_write_read(
        &mut self,
        addr: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Error> {
        // it would be possible to combine the write and read
        // in one transaction but filling the tx fifo with
        // the current code is somewhat slow even in release mode
        // which can cause issues
        self.master_write(addr, bytes)?;
        self.master_read(addr, buffer)?;
        Ok(())
    }

    /// Update I2C configuration
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

        // self.i2c.int_clr().write(|w| {
        //     w.rxfifo_wm_int_clr()
        //         .set_bit()
        //         .txfifo_wm_int_clr()
        //         .set_bit()
        // });

        // self.lp_i2c_update();
    }

    fn wait_for_completion(&self) -> Result<(), Error> {
        loop {
            let interrupts = self.i2c.int_status().read();

            // Handle completion cases
            // A full transmission was completed
            if interrupts.nack_int_st().bit_is_set() {
                self.i2c
                    .int_clr()
                    .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
                return Err(Error::InvalidResponse);
            } else if interrupts.trans_complete_int_st().bit_is_set() {
                self.disable_interrupts();

                self.i2c
                    .int_clr()
                    .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
                break;
            } else if interrupts.end_detect_int_st().bit_is_set() {
                self.i2c
                    .int_clr()
                    .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });
                break;
            }
        }

        // if self.i2c.comd0.read().command0_done().bit_is_clear()
        //     || self.i2c.comd1.read().command1_done().bit_is_clear()
        //     || self.i2c.comd2.read().command2_done().bit_is_clear()
        //     || self.i2c.comd3.read().command3_done().bit_is_clear()
        //     || self.i2c.comd4.read().command4_done().bit_is_clear()
        //     || self.i2c.comd5.read().command5_done().bit_is_clear()
        //     || self.i2c.comd6.read().command6_done().bit_is_clear()
        //     || self.i2c.comd7.read().command7_done().bit_is_clear()
        // {
        //     return Err(Error::ExecIncomplete);
        // }

        Ok(())
    }

    fn check_errors(&self) -> Result<(), Error> {
        let interrupts = self.i2c.int_raw().read();
        // Handle error cases
        if interrupts.time_out_int_raw().bit_is_set() {
            self.reset();
            return Err(Error::TimeOut);
        } else if interrupts.nack_int_raw().bit_is_set() {
            self.reset();
            return Err(Error::AckCheckFailed);
        } else if interrupts.arbitration_lost_int_raw().bit_is_set() {
            self.reset();
            return Err(Error::ArbitrationLost);
        }

        Ok(())
    }

    /// Resets the I2C controller (FIFO + FSM + command list)
    fn reset(&self) {
        // Reset interrupts
        // Disable all I2C interrupts
        self.i2c.int_ena().write(|w| unsafe { w.bits(0) });
        // Clear all I2C interrupts
        self.i2c
            .int_clr()
            .write(|w| unsafe { w.bits(I2C_LL_INTR_MASK) });

        // Reset fifo
        self.reset_fifo();

        // Reset the command list
        self.reset_command_list();

        // Reset the FSM
        self.i2c.ctr().modify(|_, w| w.fsm_rst().set_bit());
    }

    fn enable_interrupts(&self, mask: u32) {
        self.i2c
            .int_ena()
            .write(|w| unsafe { w.bits(mask) });
    }

    fn disable_interrupts(&self) {
        self.i2c.int_ena().write(|w| unsafe { w.bits(0) });
    }

    fn reset_command_list(&self) {
        self.i2c.comd0().reset();
        self.i2c.comd1().reset();
        self.i2c.comd2().reset();
        self.i2c.comd3().reset();
        self.i2c.comd4().reset();
        self.i2c.comd5().reset();
        self.i2c.comd6().reset();
        self.i2c.comd7().reset();
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
        match *command_register {
            CommandRegister::COMD0 => {
                self.i2c
                    .comd0()
                    .write(|w| unsafe { w.command0().bits(command.into()) });
            }
            CommandRegister::COMD1 => {
                self.i2c
                    .comd1()
                    .write(|w| unsafe { w.command1().bits(command.into()) });
            }
            CommandRegister::COMD2 => {
                self.i2c
                    .comd2()
                    .write(|w| unsafe { w.command2().bits(command.into()) });
            }
            CommandRegister::COMD3 => {
                self.i2c
                    .comd3()
                    .write(|w| unsafe { w.command3().bits(command.into()) });
            }
            CommandRegister::COMD4 => {
                self.i2c
                    .comd4()
                    .write(|w| unsafe { w.command4().bits(command.into()) });
            }
            CommandRegister::COMD5 => {
                self.i2c
                    .comd5()
                    .write(|w| unsafe { w.command5().bits(command.into()) });
            }
            CommandRegister::COMD6 => {
                self.i2c
                    .comd6()
                    .write(|w| unsafe { w.command6().bits(command.into()) });
            }
            CommandRegister::COMD7 => {
                self.i2c
                    .comd7()
                    .write(|w| unsafe { w.command7().bits(command.into()) });
            }
        }
        command_register.advance();
        Ok(())
    }
}

impl embedded_hal::blocking::i2c::Read for LpI2c {
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.master_read(address, buffer)
    }
}

impl embedded_hal::blocking::i2c::Write for LpI2c {
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.master_write(addr, bytes)
    }
}

impl embedded_hal::blocking::i2c::WriteRead for LpI2c {
    type Error = Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.master_write_read(address, bytes, buffer)
    }
}

fn set_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() | mask);
    }
}
fn read_peri_reg(reg: u32) -> u32 {
    unsafe { (reg as *mut u32).read_volatile() }
}

fn clear_peri_reg_mask(reg: u32, mask: u32) {
    unsafe {
        (reg as *mut u32).write_volatile((reg as *mut u32).read_volatile() & !mask);
    }
}
