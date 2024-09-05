//! Exception handling
//!
//! Currently specialized for ESP32 (LX6) configuration: which extra registers
//! to store, how many interrupt levels etc.
//!
//! First level interrupts and exceptions save full processor state to the user
//! stack. This includes the coprocessor registers contrary to the esp-idf where
//! these are lazily saved. (Kernel mode option is currently not used.)
//!
//! WindowUnder/Overflow and AllocA use default Xtensa implementation.
//!
//! LoadStoreError and Unaligned are not (yet) implemented: so all accesses to
//! IRAM must be word sized and aligned.
//!
//! Syscall 0 is not (yet) implemented: it doesn't seem to be used in rust.
//!
//! Double Exceptions can only occur during the early setup of the exception
//! handler. Afterwards PS.EXCM is set to 0 to be able to handle
//! WindowUnderflow/Overflow and recursive exceptions will happen instead.
//!
//! In various places call0 are used as long jump: `j.l` syntax is not supported
//! and `call0` can always be expanded to `mov a0,label; call a0`. Care must be
//! taken since A0 is overwritten.

mod asm;
mod context;

pub use context::Context;

/// EXCCAUSE register values
///
/// General Exception Causes. (Values of EXCCAUSE special register set by
/// general exceptions, which vector to the user, kernel, or double-exception
/// vectors).
#[allow(unused)]
#[derive(Debug, PartialEq)]
#[repr(C)]
pub enum ExceptionCause {
    /// Illegal Instruction
    Illegal                        = 0,
    /// System Call (Syscall Instruction)
    Syscall                        = 1,
    /// Instruction Fetch Error
    InstrError                     = 2,
    /// Load Store Error
    LoadStoreError                 = 3,
    /// Level 1 Interrupt
    LevelOneInterrupt              = 4,
    /// Stack Extension Assist (movsp Instruction) For Alloca
    Alloca                         = 5,
    /// Integer Divide By Zero
    DivideByZero                   = 6,
    /// Use Of Failed Speculative Access (Not Implemented)
    NextPCValueIllegal             = 7,
    /// Privileged Instruction
    Privileged                     = 8,
    /// Unaligned Load Or Store
    Unaligned                      = 9,
    /// Reserved
    ExternalRegisterPrivilegeError = 10,
    /// Reserved
    ExclusiveError                 = 11,
    /// Pif Data Error On Instruction Fetch (Rb-200x And Later)
    InstrDataError                 = 12,
    /// Pif Data Error On Load Or Store (Rb-200x And Later)
    LoadStoreDataError             = 13,
    /// Pif Address Error On Instruction Fetch (Rb-200x And Later)
    InstrAddrError                 = 14,
    /// Pif Address Error On Load Or Store (Rb-200x And Later)
    LoadStoreAddrError             = 15,
    /// Itlb Miss (No Itlb Entry Matches, Hw Refill Also Missed)
    ItlbMiss                       = 16,
    /// Itlb Multihit (Multiple Itlb Entries Match)
    ItlbMultiHit                   = 17,
    /// Ring Privilege Violation On Instruction Fetch
    InstrRing                      = 18,
    /// Size Restriction On Ifetch (Not Implemented)
    Reserved19                     = 19,
    /// Cache Attribute Does Not Allow Instruction Fetch
    InstrProhibited                = 20,
    /// Reserved
    Reserved21                     = 21,
    /// Reserved
    Reserved22                     = 22,
    /// Reserved
    Reserved23                     = 23,
    /// Dtlb Miss (No Dtlb Entry Matches, Hw Refill Also Missed)
    DtlbMiss                       = 24,
    /// Dtlb Multihit (Multiple Dtlb Entries Match)
    DtlbMultiHit                   = 25,
    /// Ring Privilege Violation On Load Or Store
    LoadStoreRing                  = 26,
    /// Size Restriction On Load/Store (Not Implemented)
    Reserved27                     = 27,
    /// Cache Attribute Does Not Allow Load
    LoadProhibited                 = 28,
    /// Cache Attribute Does Not Allow Store
    StoreProhibited                = 29,
    /// Reserved
    Reserved30                     = 30,
    /// Reserved
    Reserved31                     = 31,
    /// Access To Coprocessor 0 When Disabled
    Cp0Disabled                    = 32,
    /// Access To Coprocessor 1 When Disabled
    Cp1Disabled                    = 33,
    /// Access To Coprocessor 2 When Disabled
    Cp2Disabled                    = 34,
    /// Access To Coprocessor 3 When Disabled
    Cp3Disabled                    = 35,
    /// Access To Coprocessor 4 When Disabled
    Cp4Disabled                    = 36,
    /// Access To Coprocessor 5 When Disabled
    Cp5Disabled                    = 37,
    /// Access To Coprocessor 6 When Disabled
    Cp6Disabled                    = 38,
    /// Access To Coprocessor 7 When Disabled
    Cp7Disabled                    = 39,

    None                           = 255,
}
