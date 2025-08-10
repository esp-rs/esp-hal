//! CMD53 `IO_RW_EXTENDED` SDIO command.

use super::{BlockMode, Crc, FunctionNumber, RwFlag};
use crate::sdio::{Direction, Error};

mod op_code;

pub use op_code::OpCode;

bitfield::bitfield! {
    /// Represents the SDIO CMD53 `IO_RW_EXTENDED` command.
    ///
    /// Can be used to transfer packets of an arbitrary length.
    // FIXME: this representation wastes 2 bytes
    // it is also wasteful using copies for type conversions
    // the macro branch for `struct Ty(MSB0 [ty])` is broken...
    #[repr(C)]
    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    pub struct Cmd53(u64);
    _s, _: 47;
    _d, _: 46;
    _cmd_index, _: 45, 40;
    _rw_flag, _set_rw_flag: 39;
    _fn_number, _set_fn_number: 38, 36;
    _block_mode, _set_block_mode: 35;
    _op_code, _set_op_code: 34;
    _reg_addr, _set_reg_addr: 33, 17;
    _count, _set_count: 16, 8;
    _crc, _set_crc: 7, 1;
    _e, _: 0;
}

impl Cmd53 {
    /// Represents the [Cmd53] command byte length.
    pub const LEN: usize = 6;
    /// Represents the [Cmd53] command default byte value.
    pub const DEFAULT_ARG: [u8; 4] = [0x0; 4];
    /// Represents the [Cmd53] command index.
    pub const COMMAND_INDEX: u8 = 53;

    /// Creates a new [Cmd53].
    pub const fn new() -> Self {
        Self(Self::default_u64())
    }

    #[inline]
    const fn start() -> u8 {
        0x40 | Self::COMMAND_INDEX
    }

    #[inline]
    const fn default_u64() -> u64 {
        let [a0, a1, a2, a3] = Self::DEFAULT_ARG;
        let raw = [Self::start(), a0, a1, a2, a3];
        let crc = Crc::calculate(&raw);
        let [r0, r1, r2, r3, r4] = raw;

        u64::from_be_bytes([0x0, 0x0, r0, r1, r2, r3, r4, (crc.into_u8() << 1) | 1])
    }

    /// Gets the direction.
    pub fn direction(&self) -> Direction {
        Direction::from_bool(self._d())
    }

    /// Gets the command index.
    pub fn command_index(&self) -> u8 {
        self._cmd_index() as u8
    }

    /// Gets the function number.
    pub fn function_number(&self) -> Result<FunctionNumber, Error> {
        FunctionNumber::try_from_u8(self._fn_number() as u8)
    }

    /// Sets the function number.
    pub fn set_function_number(&mut self, number: FunctionNumber) {
        self._set_fn_number(number.into_u8().into());
    }

    /// Gets the block mode.
    pub fn block_mode(&self) -> BlockMode {
        BlockMode::from_bool(self._block_mode())
    }

    /// Sets the block mode.
    pub fn set_block_mode(&mut self, mode: BlockMode) {
        self._set_block_mode(mode.into_bool());
    }

    /// Gets the read-write flag.
    pub fn read_write_flag(&self) -> RwFlag {
        RwFlag::from_bool(self._rw_flag())
    }

    /// Sets the read-write flag.
    pub fn set_read_write_flag(&mut self, flag: RwFlag) {
        self._set_rw_flag(flag.into_bool());
    }

    /// Gets the OP code.
    pub fn op_code(&self) -> OpCode {
        OpCode::from_bool(self._op_code())
    }

    /// Sets the OP code.
    pub fn set_op_code(&mut self, op_code: OpCode) {
        self._set_op_code(op_code.into_bool());
    }

    /// Gets the register address.
    pub fn register_address(&self) -> u32 {
        self._reg_addr() as u32
    }

    /// Sets the register address.
    pub fn set_register_address(&mut self, addr: u32) {
        self._set_reg_addr(addr.into());
    }

    /// Gets the block or bytes count depending on the [BlockMode] setting.
    pub fn count(&self) -> u16 {
        self._count() as u16
    }

    /// Sets the block or bytes count depending on the [BlockMode] setting.
    pub fn set_count(&mut self, count: u16) {
        self._set_count(count.into());
    }

    /// Gets the CRC.
    pub fn crc(&self) -> Crc {
        Crc::from_u8(self._crc() as u8)
    }

    /// Calculates and sets the CRC.
    pub fn set_crc(&mut self) {
        self._set_crc(self.calculate_crc().into_u8().into());
    }

    /// Calculates the CRC.
    pub const fn calculate_crc(&self) -> Crc {
        let [_, _, b0, b1, b2, b3, b4, _] = self.0.to_be_bytes();
        Crc::calculate(&[b0, b1, b2, b3, b4])
    }

    /// Verifies if the CRC matches the calculated value.
    pub fn verify_crc(&self) -> Result<(), Error> {
        match self.calculate_crc() == self.crc() {
            false => Err(Error::Crc),
            true => Ok(()),
        }
    }

    /// Attempts to convert a byte slice into a [Cmd52].
    pub fn try_from_bytes(val: &[u8]) -> Result<Self, Error> {
        match val.len() {
            len if len < Self::LEN => Err(Error::General),
            _ => match Self(u64::from_be_bytes([
                0, 0, val[0], val[1], val[2], val[3], val[4], val[5],
            ])) {
                cmd if cmd.command_index() != Self::COMMAND_INDEX => Err(Error::General),
                cmd if cmd._s() => Err(Error::General),
                cmd if !cmd._e() => Err(Error::General),
                cmd => cmd
                    .function_number()
                    .and_then(|_| cmd.verify_crc())
                    .map(|_| cmd),
            },
        }
    }

    /// Converts the [Cmd52] into a byte array.
    pub fn into_bytes(self) -> [u8; Self::LEN] {
        let [_, _, a0, a1, a2, a3, a4, a5] = self.0.to_be_bytes();
        [a0, a1, a2, a3, a4, a5]
    }
}

impl Default for Cmd53 {
    fn default() -> Self {
        Self::new()
    }
}

impl TryFrom<&[u8]> for Cmd53 {
    type Error = Error;

    fn try_from(val: &[u8]) -> Result<Self, Self::Error> {
        Self::try_from_bytes(val)
    }
}

impl From<Cmd53> for [u8; Cmd53::LEN] {
    fn from(val: Cmd53) -> Self {
        val.into_bytes()
    }
}
