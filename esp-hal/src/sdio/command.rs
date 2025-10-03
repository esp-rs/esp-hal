//! SDIO command types.

mod block_mode;
mod cmd0;
mod cmd5;
mod cmd52;
pub mod cmd53;
mod crc;
mod flag;
mod fn_number;
mod index;

pub use block_mode::BlockMode;
pub use cmd0::Cmd0;
pub use cmd5::Cmd5;
pub use cmd52::Cmd52;
pub use cmd53::Cmd53;
pub use crc::Crc;
pub use flag::{RawFlag, RwFlag};
pub use fn_number::FunctionNumber;
pub use index::CommandIndex;

/// Represents any SDIO command structure.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AnyCmd {
    /// Represents the `IO_RW_DIRECT` command.
    Cmd52(Cmd52),
    /// Represents the `IO_RW_EXTENDED` command.
    Cmd53(Cmd53),
}

impl AnyCmd {
    /// Represents the byte length of the command.
    pub const LEN: usize = 6;
}
