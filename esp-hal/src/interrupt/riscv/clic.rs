use super::{InterruptKind, Priority, RunLevel};
use crate::{interrupt::ElevatedRunLevel, soc::pac::CLIC};

#[cfg(feature = "rt")]
pub(super) fn init() {
    let clic = unsafe { CLIC::steal() };

    // 3 level bits = 8 priority levels.
    clic.int_config()
        .modify(|_, w| unsafe { w.mnlbits().bits(3) });

    // Enable hardware vectoring on every interrupt line.
    for int in clic.int_attr_iter() {
        int.modify(|_, w| {
            w.shv().hardware();
            w.trig().positive_level()
        });
    }
}

pub(super) fn enable_cpu_interrupt_raw(cpu_interrupt: u32) {
    // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
    let clic = unsafe { CLIC::steal() };
    clic.int_ie(cpu_interrupt as usize)
        .write(|w| w.int_ie().set_bit());
}

pub(super) fn set_kind_raw(cpu_interrupt: u32, kind: InterruptKind) {
    // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
    let clic = unsafe { CLIC::steal() };
    clic.int_attr(cpu_interrupt as usize)
        .modify(|_, w| match kind {
            InterruptKind::Level => w.trig().positive_level(),
            InterruptKind::Edge => w.trig().positive_edge(),
        });
}

pub(super) fn set_priority_raw(cpu_interrupt: u32, priority: Priority) {
    // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
    let clic = unsafe { CLIC::steal() };
    let prio_bits = prio_to_bits(RunLevel::Interrupt(ElevatedRunLevel::from_priority(
        priority,
    )));
    // The `ctl` field would only write the 3 programmable bits, but we have the correct final
    // value anyway so let's write it directly.
    clic.int_ctl(cpu_interrupt as usize)
        .write(|w| unsafe { w.bits(prio_bits) });
}

pub(super) fn clear_raw(cpu_interrupt: u32) {
    // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
    let clic = unsafe { CLIC::steal() };
    clic.int_ip(cpu_interrupt as usize)
        .write(|w| w.int_ip().clear_bit());
}

pub(super) fn cpu_interrupt_priority_raw(cpu_interrupt: u32) -> u8 {
    // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
    let clic = unsafe { CLIC::steal() };
    let prio_level = clic.int_ctl(cpu_interrupt as usize).read().bits() as usize;
    bits_to_prio(prio_level)
}

/// Changes the current interrupt runlevel (the level below which interrupts are masked),
/// and returns the previous runlevel.
pub(super) fn change_current_runlevel(level: RunLevel) -> u8 {
    let current_runlevel = current_runlevel();

    // All machine mode pending interrupts with levels less than or equal
    // to the effective threshold level are not allowed to preempt the execution.
    unsafe { intthresh::write(prio_to_bits(level) as usize) };

    current_runlevel
}

/// Get the current run level (the level below which interrupts are masked).
pub(crate) fn mil() -> usize {
    mintstatus::read().mil()
}

pub(super) fn current_runlevel() -> u8 {
    let thresh = intthresh::read();

    let mil = mil();

    let level = mil.max(thresh);

    bits_to_prio(level)
}

fn prio_to_bits(priority: RunLevel) -> u8 {
    if priority.is_thread() {
        0
    } else {
        0x1F | ((u32::from(priority) as u8 - 1) << 5)
    }
}

fn bits_to_prio(bits: usize) -> u8 {
    // If mintthresh starts from 0xf, make sure we don't return Priority1
    if bits < 0x1f {
        0
    } else {
        ((bits >> 5) + 1) as u8
    }
}

/// Interrupt-level threshold (the level below which interrupts are masked).
///
/// Standard CLIC (incl. ESP32-P4 v3.0+) uses the `mintthresh` CSR (0x347). Pre-v3
/// ESP32-P4's non-standard CLIC has no such CSR (0x347 traps illegal-instruction);
/// the threshold is the memory-mapped `CLIC_INT_THRESH_REG`, upper 8 bits. Selected by
/// the `esp32p4-rev-lt-v3` esp-config option (mirrors IDF
/// `CONFIG_ESP32P4_SELECTS_REV_LESS_V3`); the memory-mapped path mirrors IDF
/// `rv_utils_{get,restore}_interrupt_threshold` under `!INTTHRESH_STANDARD`.
#[cfg(not(esp32p4_rev_lt_v3))]
mod intthresh {
    riscv::read_csr_as_usize!(0x347);
    riscv::write_csr_as_usize!(0x347);
}

#[cfg(esp32p4_rev_lt_v3)]
mod intthresh {
    // CLIC_INT_THRESH_REG = DR_REG_CLIC_BASE (0x2080_0000) + 0x8, threshold in the top
    // byte (CLIC_CPU_INT_THRESH_S = 24). The CLIC region is CPU-local, so this fixed
    // address always targets the current core.
    const CLIC_INT_THRESH_REG: *mut u32 = 0x2080_0008 as *mut u32;
    const THRESH_SHIFT: u32 = 24;

    #[inline]
    pub fn read() -> usize {
        let v = unsafe { CLIC_INT_THRESH_REG.read_volatile() };
        ((v >> THRESH_SHIFT) & 0xff) as usize
    }

    #[inline]
    pub unsafe fn write(val: usize) {
        unsafe { CLIC_INT_THRESH_REG.write_volatile(((val as u32) & 0xff) << THRESH_SHIFT) };
        // Read back so the write lands before the caller re-enables mstatus.mie (per IDF).
        let _ = unsafe { CLIC_INT_THRESH_REG.read_volatile() };
    }
}

mod mintstatus {
    // Work around riscv using a non-qualified `assert!` in constants
    macro_rules! assert {
            ($($tt:tt)*) => {
                ::core::assert!($($tt)*)
            };
        }
    // Standard CLIC places `mintstatus` at 0xFB1; pre-v3 ESP32-P4's non-standard CLIC
    // at 0x346. (On the non-standard CLIC the 0x346 `mil` reads inconsistently with the
    // slot ctl encoding, so `handle_interrupts` does not rely on it — see riscv.rs.)
    #[cfg(not(esp32p4_rev_lt_v3))]
    riscv::read_only_csr! {
        /// `mintstatus` register
        Mintstatus: 0xfb1,
        mask: usize::MAX,
    }
    #[cfg(esp32p4_rev_lt_v3)]
    riscv::read_only_csr! {
        /// `mintstatus` register (non-standard ESP32-P4 CLIC address)
        Mintstatus: 0x346,
        mask: usize::MAX,
    }
    riscv::read_only_csr_field! {
        Mintstatus,
        /// Returns the `mil` field.
        mil: [24:31],
    }
}

#[cfg(feature = "rt")]
core::arch::global_asm!(
    r#"

    .section .trap, "ax"

    /* Prevent the compiler from generating 2-byte instruction in the vector tables */
    .option push
    .option norvc

    /**
        * Vectored interrupt table. MTVT CSR points here.
        *
        * If an interrupt occurs and is configured as (hardware) vectored, the CPU will jump to
        * MTVT[31:0] + 4 * interrupt_id
        *
        * In the case of the ESP32P4/ESP32C5, the interrupt matrix, between the CPU interrupt lines
        * and the peripherals, offers 32 lines, and the lower 16 interrupts are used for CLINT.
        */
    .balign 0x40
    .global _mtvt_table
    .type _mtvt_table, @function
_mtvt_table:
    .word 0
    .word _start_Trap1_trap
    .word _start_Trap2_trap
    .word _start_Trap3_trap
    .word _start_Trap4_trap
    .word _start_Trap5_trap
    .word _start_Trap6_trap
    .word _start_Trap7_trap
    .word _start_Trap8_trap
    .word _start_Trap9_trap
    .word _start_Trap10_trap
    .word _start_Trap11_trap
    .word _start_Trap12_trap
    .word _start_Trap13_trap
    .word _start_Trap14_trap
    .word _start_Trap15_trap
    .word _start_Trap16_trap
    .word _start_Trap17_trap
    .word _start_Trap18_trap
    .word _start_Trap19_trap
    .word _start_Trap20_trap
    .word _start_Trap21_trap
    .word _start_Trap22_trap
    .word _start_Trap23_trap
    .word _start_Trap24_trap
    .word _start_Trap25_trap
    .word _start_Trap26_trap
    .word _start_Trap27_trap
    .word _start_Trap28_trap
    .word _start_Trap29_trap
    .word _start_Trap30_trap
    .word _start_Trap31_trap
    .word _start_Trap32_trap
    .word _start_Trap33_trap
    .word _start_Trap34_trap
    .word _start_Trap35_trap
    .word _start_Trap36_trap
    .word _start_Trap37_trap
    .word _start_Trap38_trap
    .word _start_Trap39_trap
    .word _start_Trap40_trap
    .word _start_Trap41_trap
    .word _start_Trap42_trap
    .word _start_Trap43_trap
    .word _start_Trap44_trap
    .word _start_Trap45_trap
    .word _start_Trap46_trap
    .word _start_Trap47_trap

    .size _mtvt_table, .-_mtvt_table
    .option pop
    "#
);

// Core 2
#[cfg(all(feature = "rt", multi_core))]
core::arch::global_asm!(
    r#"
    .section .trap, "ax"

    /* Prevent the compiler from generating 2-byte instruction in the vector tables */
    .option push
    .option norvc

    .balign 0x40
    .global _mtvt_table2
    .type _mtvt_table2, @function
_mtvt_table2:
    .word 0
    .word _start_Trap1_trap
    .word _start_Trap2_trap
    .word _start_Trap3_trap
    .word _start_Trap4_trap
    .word _start_Trap5_trap
    .word _start_Trap6_trap
    .word _start_Trap7_trap
    .word _start_Trap8_trap
    .word _start_Trap9_trap
    .word _start_Trap10_trap
    .word _start_Trap11_trap
    .word _start_Trap12_trap
    .word _start_Trap13_trap
    .word _start_Trap14_trap
    .word _start_Trap15_trap
    .word _start_Trap16_trap
    .word _start_Trap17_trap
    .word _start_Trap18_trap
    .word _start_Trap19_trap
    .word _start_Trap20_trap
    .word _start_Trap21_trap
    .word _start_Trap22_trap
    .word _start_Trap23_trap
    .word _start_Trap24_trap
    .word _start_Trap25_trap
    .word _start_Trap26_trap
    .word _start_Trap27_trap
    .word _start_Trap28_trap
    .word _start_Trap29_trap
    .word _start_Trap30_trap
    .word _start_Trap31_trap
    .word _start_Trap32_trap
    .word _start_Trap33_trap
    .word _start_Trap34_trap
    .word _start_Trap35_trap
    .word _start_Trap36_trap
    .word _start_Trap37_trap
    .word _start_Trap38_trap
    .word _start_Trap39_trap
    .word _start_Trap40_trap
    .word _start_Trap41_trap
    .word _start_Trap42_trap
    .word _start_Trap43_trap
    .word _start_Trap44_trap
    .word _start_Trap45_trap
    .word _start_Trap46_trap
    .word _start_Trap47_trap

    .size _mtvt_table2, .-_mtvt_table2
    .option pop
    "#
);
