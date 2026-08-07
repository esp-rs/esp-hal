//! CPU power-down retention during light sleep (RISC-V PMU chips).

// Mirrors ESP-IDF `v5.4` `esp_sleep_cpu_retention()` (`sleep_cpu.c`,
// `sleep_cpu_asm.S`, `rvsleep-frames.h`).

use core::sync::atomic::{AtomicU32, Ordering};

use procmacros::ram;

/// Second buffer required by
/// [`RtcSleepConfig::with_top_power_down`](crate::rtc_cntl::sleep::RtcSleepConfig::with_top_power_down).
#[instability::unstable]
pub use crate::rtc_cntl::retention::SystemRetentionMemory;
use crate::peripherals::{LP_AON, PMU};
use crate::soc::csr::{NONCRITICAL_WORDS, restore_noncritical, save_noncritical};

/// Incremented only when the CPU domain actually lost and regained power.
static CPU_POWERDOWN_WAKES: AtomicU32 = AtomicU32::new(0);

/// How many times the CPU power domain was actually powered down and restored.
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
// CPU-domain device registers (INTPRI / cache / PLIC / CLINT)
// ---------------------------------------------------------------------------

use crate::rtc_cntl::retention::{
    CACHE_REGIONS,
    CLINT_REGIONS,
    INTPRI_REGIONS,
    PLIC_REGIONS,
    Region,
    total_words,
};

#[ram]
fn save_device_regs(regions: &[Region], buf: *mut u32) {
    let mut out = buf;
    for region in regions {
        let mut addr = (region.start)() as *const u32;
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
        let mut addr = (region.start)() as *mut u32;
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

/// Backing storage for CPU power-down register retention.
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
/// returning the previous value.
// Mirrors `RV_READ_MSTATUS_AND_DISABLE_INTR()`.
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
