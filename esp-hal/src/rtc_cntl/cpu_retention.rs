//! CPU power-down retention during light sleep (RISC-V PMU chips).
//!
//! When `pd_cpu` powers down, the CPU loses all state and regDMA can't reach the
//! register file/CSRs, so they are saved/restored in software in three parts:
//! critical registers (GP + machine CSRs) in assembly via a setjmp/longjmp-style
//! trick, non-critical CSRs via `csrr`/`csrw`, and CPU-domain device registers
//! (`INTPRI`, `PLIC`/`CLINT`, cache). Backing RAM ([`CpuRetentionMemory`]) is
//! caller-owned, opt-in via
//! [`RtcSleepConfig::with_cpu_power_down`](crate::rtc_cntl::sleep::RtcSleepConfig::with_cpu_power_down).
//!
//! The logic is chip-agnostic - only the device-register base addresses are
//! per-chip data (from the `retention::chip` module). Everything runs from IRAM
//! (`.rwtext`): the ROM wakes with the flash cache lost, so no `#[ram]` function
//! may call flash-resident code until the cache config is restored.

// Mirrors ESP-IDF `v5.4` `esp_sleep_cpu_retention()` (`sleep_cpu.c`,
// `sleep_cpu_asm.S`, `rvsleep-frames.h`).

use core::sync::atomic::{AtomicU32, Ordering};

use procmacros::ram;

use crate::peripherals::{LP_AON, PMU};
/// Second buffer required by
/// [`RtcSleepConfig::with_top_power_down`](crate::rtc_cntl::sleep::RtcSleepConfig::with_top_power_down).
#[instability::unstable]
pub use crate::rtc_cntl::retention::SystemRetentionMemory;

/// Incremented only when the CPU domain actually lost and regained power.
static CPU_POWERDOWN_WAKES: AtomicU32 = AtomicU32::new(0);

/// How many times the CPU power domain was actually powered down and restored.
/// Diagnostic: rises across light sleeps only if the CPU genuinely lost power.
#[instability::unstable]
pub fn cpu_power_down_wake_count() -> u32 {
    CPU_POWERDOWN_WAKES.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// Critical register frame (RvCoreCriticalSleepFrame)
// ---------------------------------------------------------------------------

// Word buffer addressed by byte offset (`RV_SLP_CTX_*`) from the assembly below;
// layout matches ESP-IDF's `rvsleep-frames.h`:
//
//   0: mepc     1: ra       2: sp       3: gp       4: tp
//   5: t0       6: t1       7: t2       8: s0       9: s1
//  10: a0  ..  17: a7      18: s2  ..  27: s11     28: t3  ..  31: t6
//  32: mstatus 33: mtvec   34: mcause  35: mtval   36: mie  37: mip  38: pmufunc
const CRITICAL_FRAME_WORDS: usize = 39;

// `pmufunc` slot. `pmufunc & 0x3`: `1` = going to sleep, `3` = resumed via the
// wake stub.
const PMUFUNC_WORD: usize = 38;

/// Pointer the assembly reads to find the critical frame. Set before every sleep.
static mut RV_CORE_CRITICAL_REGS_FRAME: *mut u32 = core::ptr::null_mut();

unsafe extern "C" {
    /// Save the critical registers into `RV_CORE_CRITICAL_REGS_FRAME`, mark the
    /// frame "going to sleep", and return the frame pointer.
    fn rv_core_critical_regs_save() -> *mut u32;
    /// Restore the critical registers. Used as the ROM wake stub: returns as if
    /// [`rv_core_critical_regs_save`] had just returned.
    fn rv_core_critical_regs_restore() -> *mut u32;
}

// Ported from ESP-IDF's `rv_core_critical_regs_save`/`..._restore` in
// `sleep_cpu_asm.S`.
core::arch::global_asm!(
    r#"
    .set RV_SLP_CTX_MEPC,    0
    .set RV_SLP_CTX_RA,      4
    .set RV_SLP_CTX_SP,      8
    .set RV_SLP_CTX_GP,      12
    .set RV_SLP_CTX_TP,      16
    .set RV_SLP_CTX_T0,      20
    .set RV_SLP_CTX_T1,      24
    .set RV_SLP_CTX_T2,      28
    .set RV_SLP_CTX_S0,      32
    .set RV_SLP_CTX_S1,      36
    .set RV_SLP_CTX_A0,      40
    .set RV_SLP_CTX_A1,      44
    .set RV_SLP_CTX_A2,      48
    .set RV_SLP_CTX_A3,      52
    .set RV_SLP_CTX_A4,      56
    .set RV_SLP_CTX_A5,      60
    .set RV_SLP_CTX_A6,      64
    .set RV_SLP_CTX_A7,      68
    .set RV_SLP_CTX_S2,      72
    .set RV_SLP_CTX_S3,      76
    .set RV_SLP_CTX_S4,      80
    .set RV_SLP_CTX_S5,      84
    .set RV_SLP_CTX_S6,      88
    .set RV_SLP_CTX_S7,      92
    .set RV_SLP_CTX_S8,      96
    .set RV_SLP_CTX_S9,      100
    .set RV_SLP_CTX_S10,     104
    .set RV_SLP_CTX_S11,     108
    .set RV_SLP_CTX_T3,      112
    .set RV_SLP_CTX_T4,      116
    .set RV_SLP_CTX_T5,      120
    .set RV_SLP_CTX_T6,      124
    .set RV_SLP_CTX_MSTATUS, 128
    .set RV_SLP_CTX_MTVEC,   132
    .set RV_SLP_CTX_MCAUSE,  136
    .set RV_SLP_CTX_MTVAL,   140
    .set RV_SLP_CTX_MIE,     144
    .set RV_SLP_CTX_MIP,     148
    .set RV_SLP_CTX_PMUFUNC, 152

    .section .rwtext, "ax"
    .global rv_core_critical_regs_save
    .type   rv_core_critical_regs_save, @function
    .align  4
rv_core_critical_regs_save:
    csrw    mscratch, t0                # use mscratch as temp storage
    la      t0, {frame}
    lw      t0, 0(t0)                   # t0 = &CriticalFrame

    sw      ra,  RV_SLP_CTX_RA(t0)
    sw      sp,  RV_SLP_CTX_SP(t0)
    sw      gp,  RV_SLP_CTX_GP(t0)
    sw      tp,  RV_SLP_CTX_TP(t0)
    sw      t1,  RV_SLP_CTX_T1(t0)
    sw      t2,  RV_SLP_CTX_T2(t0)
    sw      s0,  RV_SLP_CTX_S0(t0)
    sw      s1,  RV_SLP_CTX_S1(t0)

    # a0 is caller saved but is also the return value (frame pointer).
    mv      a0, t0
    sw      a0,  RV_SLP_CTX_A0(t0)
    sw      a1,  RV_SLP_CTX_A1(t0)
    sw      a2,  RV_SLP_CTX_A2(t0)
    sw      a3,  RV_SLP_CTX_A3(t0)
    sw      a4,  RV_SLP_CTX_A4(t0)
    sw      a5,  RV_SLP_CTX_A5(t0)
    sw      a6,  RV_SLP_CTX_A6(t0)
    sw      a7,  RV_SLP_CTX_A7(t0)
    sw      s2,  RV_SLP_CTX_S2(t0)
    sw      s3,  RV_SLP_CTX_S3(t0)
    sw      s4,  RV_SLP_CTX_S4(t0)
    sw      s5,  RV_SLP_CTX_S5(t0)
    sw      s6,  RV_SLP_CTX_S6(t0)
    sw      s7,  RV_SLP_CTX_S7(t0)
    sw      s8,  RV_SLP_CTX_S8(t0)
    sw      s9,  RV_SLP_CTX_S9(t0)
    sw      s10, RV_SLP_CTX_S10(t0)
    sw      s11, RV_SLP_CTX_S11(t0)
    sw      t3,  RV_SLP_CTX_T3(t0)
    sw      t4,  RV_SLP_CTX_T4(t0)
    sw      t5,  RV_SLP_CTX_T5(t0)
    sw      t6,  RV_SLP_CTX_T6(t0)

    csrr    t1, mstatus
    sw      t1, RV_SLP_CTX_MSTATUS(t0)
    csrr    t2, mtvec
    sw      t2, RV_SLP_CTX_MTVEC(t0)
    csrr    t3, mcause
    sw      t3, RV_SLP_CTX_MCAUSE(t0)
    csrr    t1, mtval
    sw      t1, RV_SLP_CTX_MTVAL(t0)
    csrr    t2, mie
    sw      t2, RV_SLP_CTX_MIE(t0)
    csrr    t3, mip
    sw      t3, RV_SLP_CTX_MIP(t0)
    csrr    t1, mepc
    sw      t1, RV_SLP_CTX_MEPC(t0)

    # pmufunc: clear low 2 bits, set bit0 => "going to sleep" (== 1)
    li      t1, 0xFFFFFFFC
    lw      t2, RV_SLP_CTX_PMUFUNC(t0)
    and     t2, t1, t2
    ori     t2, t2, 0x1
    sw      t2, RV_SLP_CTX_PMUFUNC(t0)

    mv      t3, t0
    csrr    t0, mscratch
    lw      t1, RV_SLP_CTX_T1(t3)
    lw      t2, RV_SLP_CTX_T2(t3)
    lw      t3, RV_SLP_CTX_T3(t3)

    ret
    .size   rv_core_critical_regs_save, . - rv_core_critical_regs_save

    .global rv_core_critical_regs_restore
    .type   rv_core_critical_regs_restore, @function
    .align  4
rv_core_critical_regs_restore:
    la      t0, {frame}
    lw      t0, 0(t0)                   # t0 = &CriticalFrame
    beqz    t0, 1f                      # never jump to a zero address

    # pmufunc: set low 2 bits => "awake" (== 3)
    lw      t1, RV_SLP_CTX_PMUFUNC(t0)
    ori     t1, t1, 0x3
    sw      t1, RV_SLP_CTX_PMUFUNC(t0)

    lw      t2, RV_SLP_CTX_MEPC(t0)
    csrw    mepc, t2
    lw      t3, RV_SLP_CTX_MIP(t0)
    csrw    mip, t3
    lw      t1, RV_SLP_CTX_MIE(t0)
    csrw    mie, t1
    lw      t2, RV_SLP_CTX_MSTATUS(t0)
    csrw    mstatus, t2
    lw      t3, RV_SLP_CTX_MTVEC(t0)
    csrw    mtvec, t3
    lw      t1, RV_SLP_CTX_MCAUSE(t0)
    csrw    mcause, t1
    lw      t2, RV_SLP_CTX_MTVAL(t0)
    csrw    mtval, t2

    lw      t6,  RV_SLP_CTX_T6(t0)
    lw      t5,  RV_SLP_CTX_T5(t0)
    lw      t4,  RV_SLP_CTX_T4(t0)
    lw      t3,  RV_SLP_CTX_T3(t0)
    lw      s11, RV_SLP_CTX_S11(t0)
    lw      s10, RV_SLP_CTX_S10(t0)
    lw      s9,  RV_SLP_CTX_S9(t0)
    lw      s8,  RV_SLP_CTX_S8(t0)
    lw      s7,  RV_SLP_CTX_S7(t0)
    lw      s6,  RV_SLP_CTX_S6(t0)
    lw      s5,  RV_SLP_CTX_S5(t0)
    lw      s4,  RV_SLP_CTX_S4(t0)
    lw      s3,  RV_SLP_CTX_S3(t0)
    lw      s2,  RV_SLP_CTX_S2(t0)
    lw      a7,  RV_SLP_CTX_A7(t0)
    lw      a6,  RV_SLP_CTX_A6(t0)
    lw      a5,  RV_SLP_CTX_A5(t0)
    lw      a4,  RV_SLP_CTX_A4(t0)
    lw      a3,  RV_SLP_CTX_A3(t0)
    lw      a2,  RV_SLP_CTX_A2(t0)
    lw      a1,  RV_SLP_CTX_A1(t0)
    lw      a0,  RV_SLP_CTX_A0(t0)
    lw      s1,  RV_SLP_CTX_S1(t0)
    lw      s0,  RV_SLP_CTX_S0(t0)
    lw      t2,  RV_SLP_CTX_T2(t0)
    lw      t1,  RV_SLP_CTX_T1(t0)
    lw      tp,  RV_SLP_CTX_TP(t0)
    lw      gp,  RV_SLP_CTX_GP(t0)
    lw      sp,  RV_SLP_CTX_SP(t0)
    lw      ra,  RV_SLP_CTX_RA(t0)
    lw      t0,  RV_SLP_CTX_T0(t0)
1:
    ret
    .size   rv_core_critical_regs_restore, . - rv_core_critical_regs_restore
    "#,
    frame = sym RV_CORE_CRITICAL_REGS_FRAME,
);

// ---------------------------------------------------------------------------
// Non-critical CSRs (RvCoreNonCriticalSleepFrame)
// ---------------------------------------------------------------------------

/// Read a CSR by numeric address (must be a compile-time constant).
#[inline(always)]
unsafe fn read_csr<const CSR: u32>() -> u32 {
    let value: u32;
    unsafe {
        core::arch::asm!("csrr {0}, {1}", out(reg) value, const CSR, options(nostack));
    }
    value
}

/// Write a CSR by numeric address (must be a compile-time constant).
#[inline(always)]
unsafe fn write_csr<const CSR: u32>(value: u32) {
    unsafe {
        core::arch::asm!("csrw {1}, {0}", in(reg) value, const CSR, options(nostack));
    }
}

/// Generate the slot count and save/restore routines for the non-critical CSRs
/// from one list. `$name` is documentation only; CSRs are addressed by number
/// so custom Espressif CSRs need no assembler support.
// CSR list order matches ESP-IDF `sleep_cpu.c`.
macro_rules! noncritical_csrs {
    ($($name:ident = $csr:literal),+ $(,)?) => {
        /// Non-critical CSR slot count; sizes the `noncritical` field.
        const NONCRITICAL_WORDS: usize = [$($csr),+].len();

        #[ram]
        fn save_noncritical(buf: *mut u32) {
            let mut i = 0usize;
            $(
                unsafe { buf.add(i).write(read_csr::<$csr>()); }
                i += 1;
            )+
            let _ = i;
        }

        #[ram]
        fn restore_noncritical(buf: *const u32) {
            let mut i = 0usize;
            $(
                unsafe { write_csr::<$csr>(buf.add(i).read()); }
                i += 1;
            )+
            let _ = i;
        }
    };
}

noncritical_csrs! {
    mscratch = 0x340,
    mideleg  = 0x303,
    misa     = 0x301,
    tselect  = 0x7A0,
    tdata1   = 0x7A1,
    tdata2   = 0x7A2,
    tcontrol = 0x7A5,
    pmpaddr0 = 0x3B0, pmpaddr1 = 0x3B1, pmpaddr2 = 0x3B2, pmpaddr3 = 0x3B3,
    pmpaddr4 = 0x3B4, pmpaddr5 = 0x3B5, pmpaddr6 = 0x3B6, pmpaddr7 = 0x3B7,
    pmpaddr8 = 0x3B8, pmpaddr9 = 0x3B9, pmpaddr10 = 0x3BA, pmpaddr11 = 0x3BB,
    pmpaddr12 = 0x3BC, pmpaddr13 = 0x3BD, pmpaddr14 = 0x3BE, pmpaddr15 = 0x3BF,
    pmpcfg0 = 0x3A0, pmpcfg1 = 0x3A1, pmpcfg2 = 0x3A2, pmpcfg3 = 0x3A3,
    pmaaddr0 = 0xBD0, pmaaddr1 = 0xBD1, pmaaddr2 = 0xBD2, pmaaddr3 = 0xBD3,
    pmaaddr4 = 0xBD4, pmaaddr5 = 0xBD5, pmaaddr6 = 0xBD6, pmaaddr7 = 0xBD7,
    pmaaddr8 = 0xBD8, pmaaddr9 = 0xBD9, pmaaddr10 = 0xBDA, pmaaddr11 = 0xBDB,
    pmaaddr12 = 0xBDC, pmaaddr13 = 0xBDD, pmaaddr14 = 0xBDE, pmaaddr15 = 0xBDF,
    pmacfg0 = 0xBC0, pmacfg1 = 0xBC1, pmacfg2 = 0xBC2, pmacfg3 = 0xBC3,
    pmacfg4 = 0xBC4, pmacfg5 = 0xBC5, pmacfg6 = 0xBC6, pmacfg7 = 0xBC7,
    pmacfg8 = 0xBC8, pmacfg9 = 0xBC9, pmacfg10 = 0xBCA, pmacfg11 = 0xBCB,
    pmacfg12 = 0xBCC, pmacfg13 = 0xBCD, pmacfg14 = 0xBCE, pmacfg15 = 0xBCF,
    utvec   = 0x005,
    ustatus = 0x000,
    uepc    = 0x041,
    ucause  = 0x042,
    mpcer   = 0x7E0,
    mpcmr   = 0x7E1,
    mpccr   = 0x7E2,
    cpu_testbus_ctrl = 0x7E3,
    upcer   = 0x800,
    upcmr   = 0x801,
    upccr   = 0x802,
    ugpio_oen = 0x803,
    ugpio_in  = 0x804,
    ugpio_out = 0x805,
}

// ---------------------------------------------------------------------------
// CPU-domain device registers (INTPRI / cache / PLIC / CLINT)
// ---------------------------------------------------------------------------

/// A contiguous run of `words` 32-bit registers starting at `start`.
struct Region {
    start: u32,
    words: usize,
}

/// Total 32-bit words covered by a set of [`Region`]s, to size their store.
const fn total_words(regions: &[Region]) -> usize {
    let mut words = 0;
    let mut i = 0;
    while i < regions.len() {
        words += regions[i].words;
        i += 1;
    }
    words
}

// Per-chip base addresses (the region layout below is chip-agnostic).
use crate::rtc_cntl::retention::{
    CACHE_BASE,
    CLINT_MINT_BASE,
    CLINT_UINT_BASE,
    INTPRI_BASE,
    PLIC_MX_BASE,
    PLIC_UX_BASE,
};

// Interrupt matrix priority registers (`INTPRI`).
const INTPRI_REGIONS: [Region; 2] = [
    // INTPRI_CORE0_CPU_INT_ENABLE_REG ..= INTPRI_RND_ECO_LOW_REG
    Region {
        start: INTPRI_BASE,
        words: 45,
    },
    // INTPRI_RND_ECO_HIGH_REG
    Region {
        start: INTPRI_BASE + 0x3FC,
        words: 1,
    },
];

// L1 cache control (`EXTMEM`/`CACHE`).
const CACHE_REGIONS: [Region; 2] = [
    // *_L1_CACHE_CTRL_REG
    Region {
        start: CACHE_BASE + 0x4,
        words: 1,
    },
    // *_L1_CACHE_WRAP_AROUND_CTRL_REG
    Region {
        start: CACHE_BASE + 0x20,
        words: 1,
    },
];

// PLIC machine/user interrupt controllers.
const PLIC_REGIONS: [Region; 4] = [
    // PLIC_MXINT_ENABLE_REG ..= PLIC_MXINT_CLAIM_REG
    Region {
        start: PLIC_MX_BASE,
        words: 38,
    },
    // PLIC_MXINT_CONF_REG
    Region {
        start: PLIC_MX_BASE + 0x3FC,
        words: 1,
    },
    // PLIC_UXINT_ENABLE_REG ..= PLIC_UXINT_CLAIM_REG
    Region {
        start: PLIC_UX_BASE,
        words: 38,
    },
    // PLIC_UXINT_CONF_REG
    Region {
        start: PLIC_UX_BASE + 0x3FC,
        words: 1,
    },
];

// CLINT machine/user timers.
const CLINT_REGIONS: [Region; 2] = [
    // CLINT_MINT_SIP_REG ..= CLINT_MINT_MTIMECMP_H_REG
    Region {
        start: CLINT_MINT_BASE,
        words: 6,
    },
    // CLINT_UINT_SIP_REG ..= CLINT_UINT_UTIMECMP_H_REG
    Region {
        start: CLINT_UINT_BASE,
        words: 6,
    },
];

#[ram]
fn save_device_regs(regions: &[Region], buf: *mut u32) {
    let mut out = buf;
    for region in regions {
        let mut addr = region.start as *const u32;
        for _ in 0..region.words {
            unsafe {
                out.write(addr.read_volatile());
                out = out.add(1);
                addr = addr.add(1);
            }
        }
    }
}

#[ram]
fn restore_device_regs(regions: &[Region], buf: *const u32) {
    let mut src = buf;
    for region in regions {
        let mut addr = region.start as *mut u32;
        for _ in 0..region.words {
            unsafe {
                addr.write_volatile(src.read());
                src = src.add(1);
                addr = addr.add(1);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Caller-owned retention storage
// ---------------------------------------------------------------------------

/// Backing storage (~1 KiB) for CPU power-down register retention.
///
/// Caller-owned, opted into via [`RtcSleepConfig::with_cpu_power_down`] (or
/// [`RtcSleepConfig::with_top_power_down`], which also powers the CPU down).
///
/// [`RtcSleepConfig::with_cpu_power_down`]: crate::rtc_cntl::sleep::RtcSleepConfig::with_cpu_power_down
/// [`RtcSleepConfig::with_top_power_down`]: crate::rtc_cntl::sleep::RtcSleepConfig::with_top_power_down
#[instability::unstable]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C, align(4))]
pub struct CpuRetentionMemory {
    /// Critical frame: GP registers + machine CSRs, addressed by the assembly.
    critical: [u32; CRITICAL_FRAME_WORDS],
    /// Non-critical CSRs.
    noncritical: [u32; NONCRITICAL_WORDS],
    /// CPU-domain device registers.
    intpri: [u32; total_words(&INTPRI_REGIONS)],
    cache: [u32; total_words(&CACHE_REGIONS)],
    plic: [u32; total_words(&PLIC_REGIONS)],
    clint: [u32; total_words(&CLINT_REGIONS)],
}

impl CpuRetentionMemory {
    /// Create a new, zeroed CPU retention buffer.
    #[instability::unstable]
    pub const fn new() -> Self {
        Self {
            critical: [0; CRITICAL_FRAME_WORDS],
            noncritical: [0; NONCRITICAL_WORDS],
            intpri: [0; total_words(&INTPRI_REGIONS)],
            cache: [0; total_words(&CACHE_REGIONS)],
            plic: [0; total_words(&PLIC_REGIONS)],
            clint: [0; total_words(&CLINT_REGIONS)],
        }
    }
}

#[instability::unstable]
impl Default for CpuRetentionMemory {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Entry: save -> sleep -> restore
// ---------------------------------------------------------------------------

/// Read `mstatus` and clear its global machine-interrupt-enable bit (`MIE`),
/// returning the previous value. Mirrors `RV_READ_MSTATUS_AND_DISABLE_INTR()`.
#[inline(always)]
unsafe fn save_mstatus_and_disable_int() -> u32 {
    let mstatus: u32;
    unsafe {
        core::arch::asm!("csrrci {0}, mstatus, 0b1000", out(reg) mstatus, options(nostack));
    }
    mstatus
}

#[inline(always)]
unsafe fn restore_mstatus(mstatus: u32) {
    unsafe {
        core::arch::asm!("csrw mstatus, {0}", in(reg) mstatus, options(nostack));
    }
}

/// Save critical registers, program the wake stub and request sleep, spinning
/// until wakeup or rejection. The save pass
/// (`pmufunc & 0x3 == 1`) sleeps; on wakeup the ROM jumps to the restore
/// routine, which returns here with `pmufunc & 0x3 == 3`.
// Mirrors ESP-IDF `do_cpu_retention()`.
#[ram]
fn do_cpu_retention() {
    let frame = unsafe { rv_core_critical_regs_save() };

    let pmufunc = unsafe { frame.add(PMUFUNC_WORD).read_volatile() };
    if pmufunc & 0x3 == 0x1 {
        // Going to sleep. LP_AON_STORE8 is the ROM wake-stub address register.
        LP_AON::regs().store8().write(|w| unsafe {
            w.bits(rv_core_critical_regs_restore as *const () as usize as u32)
        });

        // pmu_ll_hp_set_sleep_enable
        PMU::regs()
            .slp_wakeup_cntl0()
            .write(|w| w.sleep_req().bit(true));

        // On power-down the CPU loses power here and resumes via the wake stub;
        // on a rejected sleep we fall out normally.
        loop {
            let int_raw = PMU::regs().int_raw().read();
            if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                break;
            }
        }
    } else if pmufunc & 0x3 == 0x3 {
        // Resumed via the ROM wake stub: the CPU domain really lost power.
        CPU_POWERDOWN_WAKES.fetch_add(1, Ordering::Relaxed);
    }
}

/// CPU-power-down light sleep with software register retention, wrapping the
/// sleep trigger in save/restore.
///
/// `top` is the TOP-domain regDMA store when the `TOP` domain is also powered
/// down (null otherwise). On chips whose PAU powers down with `TOP`
/// ([`SystemRetentionMemory`]'s software-triggered restore), the peripherals -
/// including the flash SPI controller - must be restored here in RAM before this
/// function returns to flash-resident code; on hardware-restore chips
/// (and when `top` is null) that call is a no-op.
///
/// # Safety
///
/// The PMU must already be configured for a `pd_cpu` light sleep, stopping the
/// CPU must be safe, and `mem`/`top` must stay valid across the sleep.
// Mirrors ESP-IDF's `esp_sleep_cpu_retention()`.
#[ram]
pub(crate) unsafe fn sleep_with_cpu_retention(
    mem: &mut CpuRetentionMemory,
    top: *mut crate::rtc_cntl::retention::SystemRetentionMemory,
) {
    unsafe {
        RV_CORE_CRITICAL_REGS_FRAME = mem.critical.as_mut_ptr();

        let mstatus = save_mstatus_and_disable_int();

        save_device_regs(&PLIC_REGIONS, mem.plic.as_mut_ptr());
        save_device_regs(&CLINT_REGIONS, mem.clint.as_mut_ptr());
        save_device_regs(&INTPRI_REGIONS, mem.intpri.as_mut_ptr());
        save_device_regs(&CACHE_REGIONS, mem.cache.as_mut_ptr());
        save_noncritical(mem.noncritical.as_mut_ptr());

        do_cpu_retention();

        // Software-triggered regDMA restore (TOP powered down): bring the TOP
        // peripherals - crucially the flash SPI controller - back first, while
        // still running from RAM. No-op on hardware-restore chips / cpu-only pd.
        if !top.is_null() {
            crate::rtc_cntl::retention::restore_top_retention(&mut *top);
        }

        // Restore in reverse order; the cache config must come back before we
        // return to flash-resident code.
        restore_noncritical(mem.noncritical.as_ptr());
        restore_device_regs(&CACHE_REGIONS, mem.cache.as_ptr());
        restore_device_regs(&INTPRI_REGIONS, mem.intpri.as_ptr());
        restore_device_regs(&CLINT_REGIONS, mem.clint.as_ptr());
        restore_device_regs(&PLIC_REGIONS, mem.plic.as_ptr());

        restore_mstatus(mstatus);
    }
}
