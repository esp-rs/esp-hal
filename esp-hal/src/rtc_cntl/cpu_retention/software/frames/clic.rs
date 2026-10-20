//! Sleep frames of CLIC RISC-V cores.
//!
//! The ESP32-C5 and the ESP32-C61 have byte-identical `rvsleep-frames.h`, so they share one layout.

pub(crate) use super::{CRITICAL_FRAME_SIZE, CriticalSleepFrame};
use crate::macros::{read_csr, write_csr};

/// Low two bits of `CriticalSleepFrame::pmufunc`: the CPU is about to sleep.
pub(crate) const PMUFUNC_GOING_TO_SLEEP: u32 = 1;
/// Low two bits of `CriticalSleepFrame::pmufunc`: the CPU has just woken.
pub(crate) const PMUFUNC_JUST_WOKE: u32 = 3;

const _: () = ::core::assert!(size_of::<CriticalSleepFrame>() == 40 * 4);

// The custom numbers come from `esp32c5/sleep_cpu.c:43-49`, and the PMA numbers from
// `components/riscv/include/riscv/csr.h`.
super::non_critical_frame! {
    mscratch = 0x340,
    misa = 0x301,
    tselect = 0x7a0,
    tdata1 = 0x7a1,
    tdata2 = 0x7a2,
    tcontrol = 0x7a5,
    pmpaddr0 = 0x3b0,
    pmpaddr1 = 0x3b1,
    pmpaddr2 = 0x3b2,
    pmpaddr3 = 0x3b3,
    pmpaddr4 = 0x3b4,
    pmpaddr5 = 0x3b5,
    pmpaddr6 = 0x3b6,
    pmpaddr7 = 0x3b7,
    pmpaddr8 = 0x3b8,
    pmpaddr9 = 0x3b9,
    pmpaddr10 = 0x3ba,
    pmpaddr11 = 0x3bb,
    pmpaddr12 = 0x3bc,
    pmpaddr13 = 0x3bd,
    pmpaddr14 = 0x3be,
    pmpaddr15 = 0x3bf,
    pmpcfg0 = 0x3a0,
    pmpcfg1 = 0x3a1,
    pmpcfg2 = 0x3a2,
    pmpcfg3 = 0x3a3,
    pmaaddr0 = 0xbd0,
    pmaaddr1 = 0xbd1,
    pmaaddr2 = 0xbd2,
    pmaaddr3 = 0xbd3,
    pmaaddr4 = 0xbd4,
    pmaaddr5 = 0xbd5,
    pmaaddr6 = 0xbd6,
    pmaaddr7 = 0xbd7,
    pmaaddr8 = 0xbd8,
    pmaaddr9 = 0xbd9,
    pmaaddr10 = 0xbda,
    pmaaddr11 = 0xbdb,
    pmaaddr12 = 0xbdc,
    pmaaddr13 = 0xbdd,
    pmaaddr14 = 0xbde,
    pmaaddr15 = 0xbdf,
    pmacfg0 = 0xbc0,
    pmacfg1 = 0xbc1,
    pmacfg2 = 0xbc2,
    pmacfg3 = 0xbc3,
    pmacfg4 = 0xbc4,
    pmacfg5 = 0xbc5,
    pmacfg6 = 0xbc6,
    pmacfg7 = 0xbc7,
    pmacfg8 = 0xbc8,
    pmacfg9 = 0xbc9,
    pmacfg10 = 0xbca,
    pmacfg11 = 0xbcb,
    pmacfg12 = 0xbcc,
    pmacfg13 = 0xbcd,
    pmacfg14 = 0xbce,
    pmacfg15 = 0xbcf,
    mcycle = 0xb00,
    mtvt = 0x307,
    mxstatus = 0x7c0,
    mhcr = 0x7c1,
    mhint = 0x7c5,
    mexstatus = 0x7e1,
    jvt = 0x017,
}

const _: () = ::core::assert!(size_of::<NonCriticalSleepFrame>() == 65 * 4);

pub(crate) const NON_CRITICAL_FRAME_SIZE: usize = size_of::<NonCriticalSleepFrame>();
