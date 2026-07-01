//! # CPU power-down retention during light sleep (ESP32-C6)
//!
//! ## Overview
//!
//! During light sleep the ESP32-C6 can additionally power **down the CPU power
//! domain** (`pd_cpu`) while the rest of the digital system (the `TOP` domain:
//! RAM, peripherals, ...) stays powered. Powering the CPU down loses all of its
//! state, so before sleeping we save everything required to resume execution and
//! restore it on wakeup.
//!
//! Unlike peripheral (`TOP`-domain) retention, CPU retention does **not** use the
//! regDMA/PAU engine. The CPU register file and CSRs are not reachable by regDMA,
//! so ESP-IDF saves/restores them in **software**, which is exactly what this
//! module does. It mirrors `esp_sleep_cpu_retention()` in
//! `components/esp_hw_support/lowpower/port/esp32c6/sleep_cpu.c`.
//!
//! The save/restore is split into three parts, matching ESP-IDF:
//!
//! 1. **Critical registers** - the general-purpose registers and the handful of
//!    machine CSRs needed to resume the interrupted control flow (`mepc`,
//!    `mstatus`, `mtvec`, ...). Saved and restored in assembly
//!    (`rv_core_critical_regs_save` / `rv_core_critical_regs_restore`), using a
//!    `setjmp`/`longjmp`-style trick: the save routine records the return
//!    context and, on wakeup, the ROM jumps to the restore routine which returns
//!    *as if the save routine had just returned*.
//! 2. **Non-critical CSRs** - the rest of the architectural CSR state (PMP/PMA,
//!    trigger module, performance counters, ...). Saved/restored in Rust via
//!    `csrr`/`csrw`.
//! 3. **CPU-domain device registers** - memory-mapped registers that live in the
//!    CPU power domain (interrupt matrix priority `INTPRI`, the `PLIC`/`CLINT`
//!    interrupt controllers and the L1 cache control). Saved/restored with plain
//!    loads/stores.
//!
//! ## Wakeup path
//!
//! The whole save -> sleep -> restore path runs from **internal RAM** (`.rwtext`,
//! i.e. IRAM). This is mandatory: when the CPU is powered back up the ROM jumps
//! directly to the wake-stub address we program into `LP_AON_STORE8`
//! (`RTC_SLEEP_WAKE_STUB_ADDR_REG`), and at that point the flash cache state has
//! been lost. Only after the cache configuration is restored may we touch flash
//! again, so every function on this path is annotated `#[ram]` and must avoid
//! calling into flash-resident code.
//!
//! References (ESP-IDF `v5.4`, commit
//! `8e27ea72c6688b79348b123ff40d556cfe16c8c3`, ESP32-C6):
//! - [`sleep_cpu.c`](https://github.com/espressif/esp-idf/blob/8e27ea72c6688b79348b123ff40d556cfe16c8c3/components/esp_hw_support/lowpower/port/esp32c6/sleep_cpu.c)
//! - [`sleep_cpu_asm.S`](https://github.com/espressif/esp-idf/blob/8e27ea72c6688b79348b123ff40d556cfe16c8c3/components/esp_hw_support/lowpower/port/esp32c6/sleep_cpu_asm.S)
//! - [`rvsleep-frames.h`](https://github.com/espressif/esp-idf/blob/8e27ea72c6688b79348b123ff40d556cfe16c8c3/components/esp_hw_support/lowpower/port/esp32c6/include/rvsleep-frames.h)

use core::{
    ptr::addr_of_mut,
    sync::atomic::{AtomicU32, Ordering},
};

use procmacros::ram;

use crate::peripherals::{LP_AON, PMU};

/// Number of times execution resumed through the ROM wake stub, i.e. how many
/// times the CPU power domain was actually powered down and restored. A sleep
/// that was rejected or where the CPU stayed powered does *not* increment this.
static CPU_POWERDOWN_WAKES: AtomicU32 = AtomicU32::new(0);

/// Returns how many times the CPU power domain has actually been powered down
/// and successfully restored via the ROM wake stub.
///
/// This is primarily a diagnostic: if it increases across light sleeps then the
/// CPU genuinely lost power (rather than the request being rejected or the CPU
/// merely clock-gated).
#[instability::unstable]
pub fn cpu_power_down_wake_count() -> u32 {
    CPU_POWERDOWN_WAKES.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// Critical register frame (RvCoreCriticalSleepFrame)
// ---------------------------------------------------------------------------

// The critical frame is a raw word buffer, not a typed struct: the assembly
// below is its only accessor and addresses every slot by byte offset
// (`RV_SLP_CTX_*`), so Rust just needs a correctly-sized, 4-byte-aligned buffer.
// The layout, word for word, matches ESP-IDF's `rvsleep-frames.h`:
//
//   0: mepc     1: ra       2: sp       3: gp       4: tp
//   5: t0       6: t1       7: t2       8: s0       9: s1
//  10: a0  ..  17: a7      18: s2  ..  27: s11     28: t3  ..  31: t6
//  32: mstatus 33: mtvec   34: mcause  35: mtval   36: mie  37: mip  38: pmufunc
const CRITICAL_FRAME_WORDS: usize = 39;

/// Word index of the `pmufunc` slot (byte offset `RV_SLP_CTX_PMUFUNC` = 152).
/// `pmufunc & 0x3` encodes the phase: `1` = going to sleep, `3` = resumed via
/// the wake stub.
const PMUFUNC_WORD: usize = 38;

/// Backing store for the critical frame. Lives in internal RAM (`.bss`), which
/// is retained while only the CPU domain is powered down.
static mut CRITICAL_FRAME: [u32; CRITICAL_FRAME_WORDS] = [0; CRITICAL_FRAME_WORDS];

/// Pointer the assembly reads to find [`CRITICAL_FRAME`]. Set before sleeping.
static mut RV_CORE_CRITICAL_REGS_FRAME: *mut u32 = core::ptr::null_mut();

unsafe extern "C" {
    /// Save the CPU critical registers into `RV_CORE_CRITICAL_REGS_FRAME` and
    /// mark the frame as "going to sleep". Returns the frame pointer.
    fn rv_core_critical_regs_save() -> *mut u32;
    /// Restore the CPU critical registers. Used as the ROM wake stub: on wakeup
    /// it returns control as if [`rv_core_critical_regs_save`] had just returned.
    fn rv_core_critical_regs_restore() -> *mut u32;
}

// Ported from ESP-IDF's `rv_core_critical_regs_save` / `..._restore` in
// `sleep_cpu_asm.S`:
// https://github.com/espressif/esp-idf/blob/8e27ea72c6688b79348b123ff40d556cfe16c8c3/components/esp_hw_support/lowpower/port/esp32c6/sleep_cpu_asm.S
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

/// Defines the set of non-critical CSRs to retain from a single canonical list,
/// generating the backing store plus the save and restore routines so the order
/// and slot count can never drift between them.
///
/// The list and its order mirror `rv_core_noncritical_regs_save()` /
/// `..._restore()` in ESP-IDF's [`sleep_cpu.c`](https://github.com/espressif/esp-idf/blob/8e27ea72c6688b79348b123ff40d556cfe16c8c3/components/esp_hw_support/lowpower/port/esp32c6/sleep_cpu.c#L238-L401).
/// The `$name` tokens are documentation only; the CSR is addressed by number so
/// that the custom Espressif CSRs (`pmaaddr*`/`pmacfg*`, performance counters,
/// user GPIO) work without assembler support.
macro_rules! noncritical_csrs {
    ($($name:ident = $csr:literal),+ $(,)?) => {
        /// Backing store for the non-critical CSR values, one `u32` slot each.
        static mut NONCRITICAL_FRAME: [u32; [$($csr),+].len()] = [0; [$($csr),+].len()];

        #[ram]
        fn save_noncritical() {
            let buf = addr_of_mut!(NONCRITICAL_FRAME) as *mut u32;
            let mut i = 0usize;
            $(
                unsafe { buf.add(i).write(read_csr::<$csr>()); }
                i += 1;
            )+
            let _ = i;
        }

        #[ram]
        fn restore_noncritical() {
            let buf = addr_of_mut!(NONCRITICAL_FRAME) as *const u32;
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

/// Total number of 32-bit words covered by a set of [`Region`]s. Used to size
/// the backing stores so they always match the regions they hold.
const fn total_words(regions: &[Region]) -> usize {
    let mut words = 0;
    let mut i = 0;
    while i < regions.len() {
        words += regions[i].words;
        i += 1;
    }
    words
}

// Interrupt matrix priority registers (`INTPRI`, base 0x600C_5000).
const INTPRI_REGIONS: [Region; 2] = [
    // INTPRI_CORE0_CPU_INT_ENABLE_REG ..= INTPRI_RND_ECO_LOW_REG
    Region { start: 0x600C_5000, words: 45 },
    // INTPRI_RND_ECO_HIGH_REG
    Region { start: 0x600C_53FC, words: 1 },
];

// L1 cache control (`EXTMEM`, base 0x600C_8000).
const CACHE_REGIONS: [Region; 2] = [
    // EXTMEM_L1_CACHE_CTRL_REG
    Region { start: 0x600C_8004, words: 1 },
    // EXTMEM_L1_CACHE_WRAP_AROUND_CTRL_REG
    Region { start: 0x600C_8020, words: 1 },
];

// PLIC machine/user interrupt controllers (bases 0x2000_1000 / 0x2000_1400).
const PLIC_REGIONS: [Region; 4] = [
    // PLIC_MXINT_ENABLE_REG ..= PLIC_MXINT_CLAIM_REG
    Region { start: 0x2000_1000, words: 38 },
    // PLIC_MXINT_CONF_REG
    Region { start: 0x2000_13FC, words: 1 },
    // PLIC_UXINT_ENABLE_REG ..= PLIC_UXINT_CLAIM_REG
    Region { start: 0x2000_1400, words: 38 },
    // PLIC_UXINT_CONF_REG
    Region { start: 0x2000_17FC, words: 1 },
];

// CLINT machine/user timers (bases 0x2000_1800 / 0x2000_1C00).
const CLINT_REGIONS: [Region; 2] = [
    // CLINT_MINT_SIP_REG ..= CLINT_MINT_MTIMECMP_H_REG
    Region { start: 0x2000_1800, words: 6 },
    // CLINT_UINT_SIP_REG ..= CLINT_UINT_UTIMECMP_H_REG
    Region { start: 0x2000_1C00, words: 6 },
];

static mut INTPRI_FRAME: [u32; total_words(&INTPRI_REGIONS)] = [0; total_words(&INTPRI_REGIONS)];
static mut CACHE_FRAME: [u32; total_words(&CACHE_REGIONS)] = [0; total_words(&CACHE_REGIONS)];
static mut PLIC_FRAME: [u32; total_words(&PLIC_REGIONS)] = [0; total_words(&PLIC_REGIONS)];
static mut CLINT_FRAME: [u32; total_words(&CLINT_REGIONS)] = [0; total_words(&CLINT_REGIONS)];

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

/// Save the CPU critical registers, program the wake stub, request sleep and
/// spin until the PMU reports wakeup (or rejects the request).
///
/// Mirrors ESP-IDF's `do_cpu_retention()`: on the *save* pass `pmufunc & 0x3 ==
/// 1`, so we set the wake stub and trigger sleep. On wakeup the ROM jumps to the
/// restore routine which returns here with `pmufunc & 0x3 == 3`, so we simply
/// fall through.
#[ram]
fn do_cpu_retention() {
    let frame = unsafe { rv_core_critical_regs_save() };

    let pmufunc = unsafe { frame.add(PMUFUNC_WORD).read_volatile() };
    if pmufunc & 0x3 == 0x1 {
        // Going to sleep.

        // RTC_SLEEP_WAKE_STUB_ADDR_REG (= LP_AON_STORE8): where the ROM jumps
        // to on light-sleep CPU-power-up.
        LP_AON::regs()
            .store8()
            .write(|w| unsafe { w.bits(rv_core_critical_regs_restore as *const () as usize as u32) });

        // pmu_ll_hp_set_sleep_enable
        PMU::regs().slp_wakeup_cntl0().write(|w| w.sleep_req().bit(true));

        // In the power-down case we never get past this loop: the CPU loses
        // power here and resumes via the wake stub. If the sleep is rejected we
        // fall out normally.
        loop {
            let int_raw = PMU::regs().int_raw().read();
            if int_raw.soc_wakeup().bit_is_set() || int_raw.soc_sleep_reject().bit_is_set() {
                break;
            }
        }
    } else if pmufunc & 0x3 == 0x3 {
        // We resumed here via the ROM wake stub, which only happens after the
        // CPU power domain was actually powered down and restored.
        CPU_POWERDOWN_WAKES.fetch_add(1, Ordering::Relaxed);
    }
}

/// Perform a full CPU-power-down light sleep with software register retention.
///
/// This is the equivalent of ESP-IDF's `esp_sleep_cpu_retention()`. The PMU sleep
/// configuration (wakeup/reject masks, power config, ...) must already have been
/// programmed by the caller; this function only adds the CPU save/restore around
/// the actual sleep trigger.
///
/// # Safety
///
/// Must be called with the PMU already configured for a `pd_cpu` light sleep and
/// with the system in a state where stopping the CPU is safe (interrupts are
/// disabled internally for the duration).
#[ram]
pub(crate) unsafe fn sleep_with_cpu_retention() {
    unsafe {
        RV_CORE_CRITICAL_REGS_FRAME = addr_of_mut!(CRITICAL_FRAME) as *mut u32;

        let mstatus = save_mstatus_and_disable_int();

        save_device_regs(&PLIC_REGIONS, addr_of_mut!(PLIC_FRAME) as *mut u32);
        save_device_regs(&CLINT_REGIONS, addr_of_mut!(CLINT_FRAME) as *mut u32);
        save_device_regs(&INTPRI_REGIONS, addr_of_mut!(INTPRI_FRAME) as *mut u32);
        save_device_regs(&CACHE_REGIONS, addr_of_mut!(CACHE_FRAME) as *mut u32);
        save_noncritical();

        do_cpu_retention();

        // Restored in the reverse order of saving. The cache configuration must
        // come back before we return to flash-resident code.
        restore_noncritical();
        restore_device_regs(&CACHE_REGIONS, addr_of_mut!(CACHE_FRAME) as *const u32);
        restore_device_regs(&INTPRI_REGIONS, addr_of_mut!(INTPRI_FRAME) as *const u32);
        restore_device_regs(&CLINT_REGIONS, addr_of_mut!(CLINT_FRAME) as *const u32);
        restore_device_regs(&PLIC_REGIONS, addr_of_mut!(PLIC_FRAME) as *const u32);

        restore_mstatus(mstatus);
    }
}
