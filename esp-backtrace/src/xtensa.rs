use core::{arch::asm, fmt::Display};

// subtract 3 from the return address
// the return address is the address following the callxN
// we get better results (especially if the caller was the last function in the
// calling function) if we report the address of callxN itself
#[allow(unused)]
pub(super) const RA_OFFSET: usize = 3;

/// Exception Cause
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub enum ExceptionCause {
    /// Illegal Instruction
    IllegalInstruction             = 0,
    /// System Call (Syscall Instruction)
    Syscall                        = 1,
    /// Instruction Fetch Error
    InstrFetchError                = 2,
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
    PrivilegedInstruction          = 8,
    /// Unaligned Load Or Store
    UnalignedLoadOrStore           = 9,
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
    /// None
    None                           = 255,
}

impl Display for ExceptionCause {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if *self == Self::Cp0Disabled {
            write!(f, "Cp0Disabled (Access to the floating point coprocessor is not allowed. You may want to enable the `float-save-restore` feature of the `xtensa-lx-rt` crate.)")
        } else {
            write!(f, "{:?}", self)
        }
    }
}

#[doc(hidden)]
#[allow(non_snake_case)]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub struct Context {
    /// Program counter, stores the address of the next instruction to be
    /// executed.
    pub PC: u32,
    /// Processor status, holds various status flags for the CPU.
    pub PS: u32,
    /// General-purpose register A0 used for data storage and manipulation.
    pub A0: u32,
    /// General-purpose register A1 used for data storage and manipulation.
    pub A1: u32,
    /// General-purpose register A2 used for data storage and manipulation.
    pub A2: u32,
    /// General-purpose register A3 used for data storage and manipulation.
    pub A3: u32,
    /// General-purpose register A4 used for data storage and manipulation.
    pub A4: u32,
    /// General-purpose register A5 used for data storage and manipulation.
    pub A5: u32,
    /// General-purpose register A6 used for data storage and manipulation.
    pub A6: u32,
    /// General-purpose register A7 used for data storage and manipulation.
    pub A7: u32,
    /// General-purpose register A8 used for data storage and manipulation.
    pub A8: u32,
    /// General-purpose register A9 used for data storage and manipulation.
    pub A9: u32,
    /// General-purpose register A10 used for data storage and manipulation.
    pub A10: u32,
    /// General-purpose register A11 used for data storage and manipulation.
    pub A11: u32,
    /// General-purpose register A12 used for data storage and manipulation.
    pub A12: u32,
    /// General-purpose register A13 used for data storage and manipulation.
    pub A13: u32,
    /// General-purpose register A14 used for data storage and manipulation.
    pub A14: u32,
    /// General-purpose register A15 used for data storage and manipulation.
    pub A15: u32,
    /// Shift amount register, used for shift and rotate instructions.
    pub SAR: u32,
    /// Exception cause, indicates the reason for the last exception.
    pub EXCCAUSE: u32,
    /// Exception address, holds the address related to the exception.
    pub EXCVADDR: u32,
    /// Loop start address, used in loop instructions.
    pub LBEG: u32,
    /// Loop end address, used in loop instructions.
    pub LEND: u32,
    /// Loop counter, used to count iterations in loop instructions.
    pub LCOUNT: u32,
    /// Thread pointer, used for thread-local storage.
    pub THREADPTR: u32,
    /// Compare register, used for certain compare instructions.
    pub SCOMPARE1: u32,
    /// Break register, used for breakpoint-related operations.
    pub BR: u32,
    /// Accumulator low register, used for extended arithmetic operations.
    pub ACCLO: u32,
    /// Accumulator high register, used for extended arithmetic operations.
    pub ACCHI: u32,
    /// Additional register M0 used for special operations.
    pub M0: u32,
    /// Additional register M1 used for special operations.
    pub M1: u32,
    /// Additional register M2 used for special operations.
    pub M2: u32,
    /// Additional register M3 used for special operations.
    pub M3: u32,
    /// 64-bit floating-point register (low part), available if the
    /// `print-float-registers` feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F64R_LO: u32,
    /// 64-bit floating-point register (high part), available if the
    /// `print-float-registers` feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F64R_HI: u32,
    /// Floating-point status register, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F64S: u32,
    /// Floating-point control register, available if the
    /// `print-float-registers` feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub FCR: u32,
    /// Floating-point status register, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub FSR: u32,
    /// Floating-point register F0, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F0: u32,
    /// Floating-point register F1, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F1: u32,
    /// Floating-point register F2, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F2: u32,
    /// Floating-point register F3, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F3: u32,
    /// Floating-point register F4, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F4: u32,
    /// Floating-point register F5, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F5: u32,
    /// Floating-point register F6, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F6: u32,
    /// Floating-point register F7, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F7: u32,
    /// Floating-point register F8, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F8: u32,
    /// Floating-point register F9, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F9: u32,
    /// Floating-point register F10, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F10: u32,
    /// Floating-point register F11, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F11: u32,
    /// Floating-point register F12, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F12: u32,
    /// Floating-point register F13, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F13: u32,
    /// Floating-point register F14, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F14: u32,
    /// Floating-point register F15, available if the `print-float-registers`
    /// feature is enabled.
    #[cfg(feature = "print-float-registers")]
    pub F15: u32,
}

impl core::fmt::Debug for Context {
    fn fmt(&self, fmt: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        write!(
            fmt,
            "Context
PC=0x{:08x}       PS=0x{:08x}
A0=0x{:08x}       A1=0x{:08x}       A2=0x{:08x}       A3=0x{:08x}       A4=0x{:08x}
A5=0x{:08x}       A6=0x{:08x}       A7=0x{:08x}       A8=0x{:08x}       A9=0x{:08x}
A10=0x{:08x}      A11=0x{:08x}      A12=0x{:08x}      A13=0x{:08x}      A14=0x{:08x}
A15=0x{:08x}
SAR={:08x}
EXCCAUSE=0x{:08x} EXCVADDR=0x{:08x}
LBEG=0x{:08x}     LEND=0x{:08x}     LCOUNT=0x{:08x}
THREADPTR=0x{:08x}
SCOMPARE1=0x{:08x}
BR=0x{:08x}
ACCLO=0x{:08x}    ACCHI=0x{:08x}
M0=0x{:08x}       M1=0x{:08x}       M2=0x{:08x}       M3=0x{:08x}
",
            self.PC,
            self.PS,
            self.A0,
            self.A1,
            self.A2,
            self.A3,
            self.A4,
            self.A5,
            self.A6,
            self.A7,
            self.A8,
            self.A9,
            self.A10,
            self.A11,
            self.A12,
            self.A13,
            self.A14,
            self.A15,
            self.SAR,
            self.EXCCAUSE,
            self.EXCVADDR,
            self.LBEG,
            self.LEND,
            self.LCOUNT,
            self.THREADPTR,
            self.SCOMPARE1,
            self.BR,
            self.ACCLO,
            self.ACCHI,
            self.M0,
            self.M1,
            self.M2,
            self.M3,
        )?;
        #[cfg(feature = "print-float-registers")]
        write!(
            fmt,
            "F64R_LO=0x{:08x}  F64R_HI=0x{:08x}  F64S=0x{:08x}
FCR=0x{:08x}      FSR=0x{:08x}
F0=0x{:08x}       F1=0x{:08x}       F2=0x{:08x}       F3=0x{:08x}       F4=0x{:08x}
F5=0x{:08x}       F6=0x{:08x}       F7=0x{:08x}       F8=0x{:08x}       F9=0x{:08x}
F10=0x{:08x}      F11=0x{:08x}      F12=0x{:08x}      F13=0x{:08x}      F14=0x{:08x}
F15=0x{:08x}",
            self.F64R_LO,
            self.F64R_HI,
            self.F64S,
            self.FCR,
            self.FSR,
            self.F0,
            self.F1,
            self.F2,
            self.F3,
            self.F4,
            self.F5,
            self.F6,
            self.F7,
            self.F8,
            self.F9,
            self.F10,
            self.F11,
            self.F12,
            self.F13,
            self.F14,
            self.F15,
        )?;

        Ok(())
    }
}

/// This function returns the caller's frame pointer.
#[inline(never)]
#[cold]
fn sp() -> u32 {
    let mut sp: u32;
    unsafe {
        asm!(
            "mov {0}, a1", // current stack pointer
            // Spill registers, otherwise `sp - 12` will not contain the previous stack pointer
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 3",
            "add a12,a12,a12",
            "rotw 4",
            out(reg) sp
        );
    }

    // current frame pointer, caller's stack pointer
    unsafe { ((sp - 12) as *const u32).read_volatile() }
}

/// Get an array of backtrace addresses.
#[inline(never)]
#[cold]
pub fn backtrace() -> [Option<usize>; super::MAX_BACKTRACE_ADDRESSES] {
    let sp = sp();

    backtrace_internal(sp, 0)
}

pub(crate) fn sanitize_address(address: u32) -> u32 {
    (address & 0x3fff_ffff) | 0x4000_0000
}

pub(crate) fn backtrace_internal(
    sp: u32,
    suppress: u32,
) -> [Option<usize>; super::MAX_BACKTRACE_ADDRESSES] {
    let mut frames = [None; super::MAX_BACKTRACE_ADDRESSES];
    let mut index = 0;

    let mut fp = sp;
    let mut suppress = suppress;

    if !crate::is_valid_ram_address(fp) {
        println!("Current stack pointer (0x{:x}) is invalid", fp);
        return frames;
    }

    while index < frames.len() {
        unsafe {
            let return_addr = sanitize_address((fp as *const u32).offset(-4).read_volatile()); // RA/PC
            fp = (fp as *const u32).offset(-3).read_volatile(); // next FP

            // the return address is 0 but we sanitized the address - then 0 becomes
            // 0x40000000
            if return_addr == 0x40000000 {
                break;
            }

            if !crate::is_valid_ram_address(fp) {
                break;
            }

            if suppress == 0 {
                frames[index] = Some(return_addr as usize);
                index += 1;
            } else {
                suppress -= 1;
            }
        }
    }

    frames
}
