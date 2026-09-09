//! Sleep frames of CLIC RISC-V cores.
//!
//! The ESP32-C5 and the ESP32-C61 have byte-identical `rvsleep-frames.h`, so they share one layout.

use crate::macros::{read_csr, write_csr};

/// Low two bits of `CriticalSleepFrame::pmufunc`: the CPU is about to sleep.
pub(crate) const PMUFUNC_GOING_TO_SLEEP: u32 = 1;
/// Low two bits of `CriticalSleepFrame::pmufunc`: the CPU has just woken.
pub(crate) const PMUFUNC_JUST_WOKE: u32 = 3;

/// Registers that the assembly saves, because the CPU has no valid state at that point.
///
/// The field order is the order of `RvCoreCriticalSleepFrame` in
/// `components/esp_hw_support/lowpower/port/esp32c5/rvsleep-frames.h`. The assembly addresses the
/// fields through `offset_of!`, so the order must not change.
#[repr(C)]
pub(crate) struct CriticalSleepFrame {
    pub mepc: u32,
    pub ra: u32,
    pub sp: u32,
    pub gp: u32,
    pub tp: u32,
    pub t0: u32,
    pub t1: u32,
    pub t2: u32,
    pub s0: u32,
    pub s1: u32,
    pub a0: u32,
    pub a1: u32,
    pub a2: u32,
    pub a3: u32,
    pub a4: u32,
    pub a5: u32,
    pub a6: u32,
    pub a7: u32,
    pub s2: u32,
    pub s3: u32,
    pub s4: u32,
    pub s5: u32,
    pub s6: u32,
    pub s7: u32,
    pub s8: u32,
    pub s9: u32,
    pub s10: u32,
    pub s11: u32,
    pub t3: u32,
    pub t4: u32,
    pub t5: u32,
    pub t6: u32,
    pub mstatus: u32,
    pub mtvec: u32,
    pub mcause: u32,
    pub mtval: u32,
    pub mie: u32,
    pub mip: u32,
    pub mintthresh: u32,
    pub pmufunc: u32,
}

const _: () = ::core::assert!(size_of::<CriticalSleepFrame>() == 40 * 4);

/// `RV_SLEEP_CTX_FRMSZ`: the critical frame size, rounded up to 16 bytes.
pub(crate) const CRITICAL_FRAME_SIZE: usize = size_of::<CriticalSleepFrame>().next_multiple_of(16);

/// Declares the non-critical frame and the matched pair that moves it.
///
/// One list drives the struct, the save and the restore, so the three cannot drift apart.
macro_rules! non_critical_frame {
    ($($field:ident = $csr:expr),* $(,)?) => {
        /// Registers that Rust saves, because the CPU is usable while they move.
        ///
        /// The field order is the order of `RvCoreNonCriticalSleepFrame` in
        /// `components/esp_hw_support/lowpower/port/esp32c5/rvsleep-frames.h`, without
        /// `mintstatus`, because that CSR is read-only.
        #[repr(C)]
        pub(crate) struct NonCriticalSleepFrame {
            $($field: u32,)*
        }

        impl NonCriticalSleepFrame {
            /// Reads every CSR of the frame.
            ///
            /// # Safety
            ///
            /// The caller must own the CPU state that the frame describes.
            pub(crate) unsafe fn save(&mut self) {
                $(self.$field = unsafe { read_csr!($csr) };)*
            }

            /// Writes every CSR of the frame back.
            ///
            /// # Safety
            ///
            /// The frame must hold a state that [`Self::save`] read from this core.
            pub(crate) unsafe fn restore(&self) {
                $(unsafe { write_csr!($csr, self.$field) };)*
            }
        }
    };
}

// The custom numbers come from `esp32c5/sleep_cpu.c:43-49`, and the PMA numbers from
// `components/riscv/include/riscv/csr.h`.
non_critical_frame! {
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
