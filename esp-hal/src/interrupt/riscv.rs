//! Interrupt handling
//!
//! CPU interrupts 1 through 15 are reserved for each of the possible interrupt
//! priorities.
//!
//! On chips with a PLIC CPU interrupts 1,2,5,6,9 .. 19 are used.
//!
//! ```rust, ignore
//! interrupt1() => Priority::Priority1
//! interrupt2() => Priority::Priority2
//! ...
//! interrupt15() => Priority::Priority15
//! ```

#[cfg(feature = "rt")]
pub use esp_riscv_rt::TrapFrame;
use procmacros::ram;
use riscv::register::{mcause, mtvec};

#[cfg(not(any(clic, plic)))]
pub use self::classic::*;
#[cfg(clic)]
pub use self::clic::*;
#[cfg(plic)]
pub use self::plic::*;
pub use self::vectored::*;
use super::InterruptStatus;
use crate::{
    pac,
    peripherals::{INTERRUPT_CORE0, Interrupt},
    system::Cpu,
};

/// Interrupt Error
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The priority is not valid
    InvalidInterruptPriority,
    /// The CPU interrupt is a reserved interrupt
    CpuInterruptReserved,
}

/// Interrupt kind
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptKind {
    /// Level interrupt
    Level,
    /// Edge interrupt
    Edge,
}

/// Enumeration of available CPU interrupts.
/// It is possible to create a handler for each of the interrupts. (e.g.
/// `interrupt3`)
///
/// When using CLIC, these interrupts correspond to the interrupt numbers 16-47 (external
/// interrupts).
#[repr(u32)]
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CpuInterrupt {
    /// Interrupt number 1.
    #[cfg(not(clic))]
    Interrupt1 = 1,
    /// Interrupt number 1.
    #[cfg(clic)]
    Interrupt1 = 16,

    /// Interrupt number 2.
    Interrupt2,
    /// Interrupt number 3.
    Interrupt3,
    /// Interrupt number 4.
    Interrupt4,
    /// Interrupt number 5.
    Interrupt5,
    /// Interrupt number 6.
    Interrupt6,
    /// Interrupt number 7.
    Interrupt7,
    /// Interrupt number 8.
    Interrupt8,
    /// Interrupt number 9.
    Interrupt9,
    /// Interrupt number 10.
    Interrupt10,
    /// Interrupt number 11.
    Interrupt11,
    /// Interrupt number 12.
    Interrupt12,
    /// Interrupt number 13.
    Interrupt13,
    /// Interrupt number 14.
    Interrupt14,
    /// Interrupt number 15.
    Interrupt15,
    /// Interrupt number 16.
    Interrupt16,
    /// Interrupt number 17.
    Interrupt17,
    /// Interrupt number 18.
    Interrupt18,
    /// Interrupt number 19.
    Interrupt19,
    /// Interrupt number 20.
    Interrupt20,
    /// Interrupt number 21.
    Interrupt21,
    /// Interrupt number 22.
    Interrupt22,
    /// Interrupt number 23.
    Interrupt23,
    /// Interrupt number 24.
    Interrupt24,
    /// Interrupt number 25.
    Interrupt25,
    /// Interrupt number 26.
    Interrupt26,
    /// Interrupt number 27.
    Interrupt27,
    /// Interrupt number 28.
    Interrupt28,
    /// Interrupt number 29.
    Interrupt29,
    /// Interrupt number 30.
    Interrupt30,
    /// Interrupt number 31.
    Interrupt31,
    #[cfg(clic)]
    /// Interrupt number 32.
    Interrupt32,
}

/// Interrupt priority levels.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Priority {
    /// No priority.
    None = 0,
    /// Priority level 1.
    Priority1,
    /// Priority level 2.
    Priority2,
    /// Priority level 3.
    Priority3,
    /// Priority level 4.
    Priority4,
    /// Priority level 5.
    Priority5,
    /// Priority level 6.
    Priority6,
    /// Priority level 7.
    Priority7,
    /// Priority level 8.
    Priority8,
    /// Priority level 9.
    #[cfg(not(clic))]
    Priority9,
    /// Priority level 10.
    #[cfg(not(clic))]
    Priority10,
    /// Priority level 11.
    #[cfg(not(clic))]
    Priority11,
    /// Priority level 12.
    #[cfg(not(clic))]
    Priority12,
    /// Priority level 13.
    #[cfg(not(clic))]
    Priority13,
    /// Priority level 14.
    #[cfg(not(clic))]
    Priority14,
    /// Priority level 15.
    #[cfg(not(clic))]
    Priority15,
}

impl Priority {
    /// Maximum interrupt priority
    pub const fn max() -> Priority {
        cfg_if::cfg_if! {
            if #[cfg(not(clic))] {
                Priority::Priority15
            } else {
                Priority::Priority8
            }
        }
    }

    /// Minimum interrupt priority
    pub const fn min() -> Priority {
        Priority::Priority1
    }
}

impl TryFrom<u32> for Priority {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Priority::None),
            1 => Ok(Priority::Priority1),
            2 => Ok(Priority::Priority2),
            3 => Ok(Priority::Priority3),
            4 => Ok(Priority::Priority4),
            5 => Ok(Priority::Priority5),
            6 => Ok(Priority::Priority6),
            7 => Ok(Priority::Priority7),
            8 => Ok(Priority::Priority8),
            #[cfg(not(clic))]
            9 => Ok(Priority::Priority9),
            #[cfg(not(clic))]
            10 => Ok(Priority::Priority10),
            #[cfg(not(clic))]
            11 => Ok(Priority::Priority11),
            #[cfg(not(clic))]
            12 => Ok(Priority::Priority12),
            #[cfg(not(clic))]
            13 => Ok(Priority::Priority13),
            #[cfg(not(clic))]
            14 => Ok(Priority::Priority14),
            #[cfg(not(clic))]
            15 => Ok(Priority::Priority15),
            _ => Err(Error::InvalidInterruptPriority),
        }
    }
}

impl TryFrom<u8> for Priority {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Priority::try_from(value as u32)
    }
}

/// The interrupts reserved by the HAL
#[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
pub static RESERVED_INTERRUPTS: &[u32] = PRIORITY_TO_INTERRUPT;

/// Enable an interrupt by directly binding it to a available CPU interrupt
///
/// ⚠️ This installs a *raw trap handler*, the `handler` user provides is written directly into the
/// CPU interrupt vector table. That means:
///
/// - Provided handler will be used as an actual trap-handler
/// - It is user's responsibility to:
///   - Save and restore all registers they use.
///   - Clear the interrupt source if necessary.
///   - Return using the `mret` instruction.
/// - The handler should be declared as naked function. The compiler will not insert a function
///   prologue/epilogue for the user, normal Rust `fn` will result in an error.
///
/// Unless you are sure that you need such low-level control to achieve the lowest possible latency,
/// you most likely want to use [`enable`] instead.
///
/// Trying using a reserved interrupt from [`RESERVED_INTERRUPTS`] will return
/// an error.
pub fn enable_direct(
    interrupt: Interrupt,
    level: Priority,
    cpu_interrupt: CpuInterrupt,
    handler: unsafe extern "C" fn(),
) -> Result<(), Error> {
    if RESERVED_INTERRUPTS.contains(&(cpu_interrupt as _)) {
        return Err(Error::CpuInterruptReserved);
    }
    if matches!(level, Priority::None) {
        return Err(Error::InvalidInterruptPriority);
    }
    unsafe {
        map(Cpu::current(), interrupt, cpu_interrupt);
        set_priority(Cpu::current(), cpu_interrupt, level);

        let mt = mtvec::read();

        assert_eq!(
            mt.trap_mode().into_usize(),
            mtvec::TrapMode::Vectored.into_usize()
        );

        let base_addr = mt.address() as usize;

        let int_slot = base_addr.wrapping_add((cpu_interrupt as usize) * 4);

        let instr = encode_jal_x0(handler as usize, int_slot)?;

        if crate::debugger::debugger_connected() {
            core::ptr::write_volatile(int_slot as *mut u32, instr);
        } else {
            crate::debugger::DEBUGGER_LOCK.lock(|| {
                let wp = crate::debugger::clear_watchpoint(1);
                core::ptr::write_volatile(int_slot as *mut u32, instr);
                crate::debugger::restore_watchpoint(1, wp);
            });
        }

        core::arch::asm!("fence.i");

        enable_cpu_interrupt(cpu_interrupt);
    }
    Ok(())
}

// helper: returns correctly encoded RISC-V `jal` instruction
fn encode_jal_x0(target: usize, pc: usize) -> Result<u32, Error> {
    let offset = (target as isize) - (pc as isize);

    const MIN: isize = -(1isize << 20);
    const MAX: isize = (1isize << 20) - 1;

    assert!(offset % 2 == 0 && (MIN..=MAX).contains(&offset));

    let imm = offset as u32;
    let imm20 = (imm >> 20) & 0x1;
    let imm10_1 = (imm >> 1) & 0x3ff;
    let imm11 = (imm >> 11) & 0x1;
    let imm19_12 = (imm >> 12) & 0xff;

    let instr = (imm20 << 31)
        | (imm19_12 << 12)
        | (imm11 << 20)
        | (imm10_1 << 21)
        // https://lhtin.github.io/01world/app/riscv-isa/?xlen=32&insn_name=jal
        | 0b1101111u32;

    Ok(instr)
}

/// Disable the given peripheral interrupt.
pub fn disable(core: Cpu, interrupt: Interrupt) {
    map_raw(core, interrupt, DISABLED_CPU_INTERRUPT)
}

/// Get status of peripheral interrupts
#[inline]
pub fn status(_core: Cpu) -> InterruptStatus {
    cfg_if::cfg_if! {
        if #[cfg(interrupts_status_registers = "3")] {
            InterruptStatus::from(
                INTERRUPT_CORE0::regs().core_0_intr_status(0).read().bits(),
                INTERRUPT_CORE0::regs().core_0_intr_status(1).read().bits(),
                INTERRUPT_CORE0::regs().core_0_intr_status(2).read().bits(),
            )
        } else if #[cfg(interrupts_status_registers = "4")] {
            InterruptStatus::from(
                INTERRUPT_CORE0::regs().core_0_intr_status(0).read().bits(),
                INTERRUPT_CORE0::regs().core_0_intr_status(1).read().bits(),
                INTERRUPT_CORE0::regs().core_0_intr_status(2).read().bits(),
                INTERRUPT_CORE0::regs().core_0_intr_status(3).read().bits(),
            )
        } else {
            InterruptStatus::from(
                INTERRUPT_CORE0::regs().core_0_intr_status(0).read().bits(),
                INTERRUPT_CORE0::regs().core_0_intr_status(1).read().bits(),
            )
        }
    }
}

/// Assign a peripheral interrupt to an CPU interrupt.
///
/// # Safety
///
/// Do not use CPU interrupts in the [`RESERVED_INTERRUPTS`].
pub unsafe fn map(core: Cpu, interrupt: Interrupt, which: CpuInterrupt) {
    map_raw(core, interrupt, which as u32)
}

fn map_raw(core: Cpu, interrupt: Interrupt, cpu_interrupt_number: u32) {
    let interrupt_number = interrupt as usize;

    match core {
        Cpu::ProCpu => {
            INTERRUPT_CORE0::regs()
                .core_0_intr_map(interrupt_number)
                .write(|w| unsafe { w.bits(cpu_interrupt_number) });
        }
        #[cfg(multi_core)]
        Cpu::AppCpu => {
            INTERRUPT_CORE1::regs()
                .core_1_intr_map(interrupt_number)
                .write(|w| unsafe { w.bits(cpu_interrupt_number) });
        }
    }
}

/// Get cpu interrupt assigned to peripheral interrupt
#[inline]
unsafe fn assigned_cpu_interrupt(interrupt: Interrupt) -> Option<CpuInterrupt> {
    let cpu_intr = INTERRUPT_CORE0::regs()
        .core_0_intr_map(interrupt as usize)
        .read()
        .bits();

    if cpu_intr > 0 && cpu_intr != DISABLED_CPU_INTERRUPT {
        Some(unsafe { core::mem::transmute::<u32, CpuInterrupt>(cpu_intr) })
    } else {
        None
    }
}

#[cfg_attr(esp32c5, expect(unused))]
pub(crate) fn bound_cpu_interrupt_for(_cpu: Cpu, interrupt: Interrupt) -> Option<CpuInterrupt> {
    unsafe { assigned_cpu_interrupt(interrupt) }
}

fn cpu_wait_mode_on() -> bool {
    cfg_if::cfg_if! {
        if #[cfg(soc_has_pcr)] {
            crate::peripherals::PCR::regs().cpu_waiti_conf().read().cpu_wait_mode_force_on().bit_is_set()
        } else {
            crate::peripherals::SYSTEM::regs()
                .cpu_per_conf()
                .read()
                .cpu_wait_mode_force_on()
                .bit_is_set()
        }
    }
}

/// Wait for an interrupt to occur.
///
/// This function causes the current CPU core to execute its Wait For Interrupt
/// (WFI or equivalent) instruction. After executing this function, the CPU core
/// will stop execution until an interrupt occurs.
///
/// This function will return immediately when a debugger is attached, so it is intended to be
/// called in a loop.
#[inline(always)]
pub fn wait_for_interrupt() {
    if crate::debugger::debugger_connected() && !cpu_wait_mode_on() {
        // when SYSTEM_CPU_WAIT_MODE_FORCE_ON is disabled in WFI mode SBA access to memory does not
        // work for debugger, so do not enter that mode when debugger is connected.
        // https://github.com/espressif/esp-idf/blob/b9a308a47ca4128d018495662b009a7c461b6780/components/esp_hw_support/cpu.c#L57-L60
        return;
    }
    unsafe { core::arch::asm!("wfi") };
}

mod vectored {
    use super::*;
    use crate::interrupt::IsrCallback;

    // Setup interrupts ready for vectoring
    #[doc(hidden)]
    pub(crate) unsafe fn init_vectoring() {
        for (num, prio) in PRIORITY_TO_INTERRUPT.iter().copied().zip(1..) {
            let which = unsafe { core::mem::transmute::<u32, CpuInterrupt>(num) };
            set_kind(Cpu::current(), which, InterruptKind::Level);
            unsafe {
                set_priority(
                    Cpu::current(),
                    which,
                    core::mem::transmute::<u8, Priority>(prio),
                );
                enable_cpu_interrupt(which);
            }
        }
    }

    /// Get the interrupts configured for the core at the given priority
    /// matching the given status
    #[inline]
    pub(crate) fn configured_interrupts(
        core: Cpu,
        status: InterruptStatus,
        priority: Priority,
    ) -> InterruptStatus {
        unsafe {
            let mut res = InterruptStatus::empty();

            for interrupt_nr in status.iterator() {
                // safety: cast is safe because of repr(u16)
                if let Some(cpu_interrupt) =
                    assigned_cpu_interrupt(core::mem::transmute::<u16, Interrupt>(
                        interrupt_nr as u16,
                    ))
                    && priority_by_core(core, cpu_interrupt) == priority
                {
                    res.set(interrupt_nr);
                }
            }
            res
        }
    }

    /// Enables a interrupt at a given priority
    ///
    /// Note that interrupts still need to be enabled globally for interrupts
    /// to be serviced.
    pub fn enable(interrupt: Interrupt, level: Priority) -> Result<(), Error> {
        enable_on_cpu(Cpu::current(), interrupt, level)
    }

    pub(crate) fn enable_on_cpu(
        cpu: Cpu,
        interrupt: Interrupt,
        level: Priority,
    ) -> Result<(), Error> {
        if matches!(level, Priority::None) {
            return Err(Error::InvalidInterruptPriority);
        }
        unsafe {
            let cpu_interrupt = core::mem::transmute::<u32, CpuInterrupt>(
                PRIORITY_TO_INTERRUPT[(level as usize) - 1],
            );
            map(cpu, interrupt, cpu_interrupt);
            enable_cpu_interrupt(cpu_interrupt);
        }
        Ok(())
    }

    /// Binds the given interrupt to the given handler.
    ///
    /// # Safety
    ///
    /// This will replace any previously bound interrupt handler
    pub unsafe fn bind_interrupt(interrupt: Interrupt, handler: IsrCallback) {
        unsafe {
            let ptr =
                &pac::__EXTERNAL_INTERRUPTS[interrupt as usize]._handler as *const _ as *mut usize;

            if crate::debugger::debugger_connected() {
                ptr.write_volatile(handler.raw_value());
            } else {
                crate::debugger::DEBUGGER_LOCK.lock(|| {
                    let wp = crate::debugger::clear_watchpoint(1);
                    ptr.write_volatile(handler.raw_value());
                    crate::debugger::restore_watchpoint(1, wp);
                });
            }
        }
    }

    /// Returns the currently bound interrupt handler.
    pub fn bound_handler(interrupt: Interrupt) -> Option<IsrCallback> {
        unsafe {
            let addr = pac::__EXTERNAL_INTERRUPTS[interrupt as usize]._handler as usize;
            if addr == 0 {
                return None;
            }

            Some(IsrCallback::from_raw(addr))
        }
    }
}

#[cfg(not(any(clic, plic)))]
mod classic {
    use super::{CpuInterrupt, InterruptKind, Priority};
    use crate::{peripherals::INTERRUPT_CORE0, system::Cpu};

    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static DISABLED_CPU_INTERRUPT: u32 = 0;

    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static PRIORITY_TO_INTERRUPT: &[u32] =
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

    // First element is not used, just there to avoid a -1 in the interrupt handler.
    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static INTERRUPT_TO_PRIORITY: [Priority; 16] = [
        Priority::None,
        Priority::Priority1,
        Priority::Priority2,
        Priority::Priority3,
        Priority::Priority4,
        Priority::Priority5,
        Priority::Priority6,
        Priority::Priority7,
        Priority::Priority8,
        Priority::Priority9,
        Priority::Priority10,
        Priority::Priority11,
        Priority::Priority12,
        Priority::Priority13,
        Priority::Priority14,
        Priority::Priority15,
    ];

    /// Enable a CPU interrupt
    ///
    /// # Safety
    ///
    /// Make sure there is an interrupt handler registered.
    pub unsafe fn enable_cpu_interrupt(which: CpuInterrupt) {
        let cpu_interrupt_number = which as isize;
        let intr = INTERRUPT_CORE0::regs();
        intr.cpu_int_enable()
            .modify(|r, w| unsafe { w.bits((1 << cpu_interrupt_number) | r.bits()) });
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// The vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    pub fn set_kind(_core: Cpu, which: CpuInterrupt, kind: InterruptKind) {
        unsafe {
            let intr = INTERRUPT_CORE0::regs();
            let cpu_interrupt_number = which as isize;

            let interrupt_type = match kind {
                InterruptKind::Level => 0,
                InterruptKind::Edge => 1,
            };
            intr.cpu_int_type().modify(|r, w| {
                w.bits(
                    r.bits() & !(1 << cpu_interrupt_number)
                        | (interrupt_type << cpu_interrupt_number),
                )
            });
        }
    }

    /// Set the priority level of an CPU interrupt
    ///
    /// # Safety
    ///
    /// Great care must be taken when using this function; avoid changing the
    /// priority of interrupts 1 - 15.
    pub unsafe fn set_priority(_core: Cpu, which: CpuInterrupt, priority: Priority) {
        let intr = INTERRUPT_CORE0::regs();
        intr.cpu_int_pri(which as usize)
            .write(|w| unsafe { w.map().bits(priority as u8) });
    }

    /// Clear a CPU interrupt
    #[inline]
    pub fn clear(_core: Cpu, which: CpuInterrupt) {
        unsafe {
            let cpu_interrupt_number = which as usize;
            let intr = INTERRUPT_CORE0::regs();
            intr.cpu_int_clear()
                .write(|w| w.bits(1 << cpu_interrupt_number));
        }
    }

    /// Get interrupt priority
    #[inline]
    pub(super) fn priority_by_core(_core: Cpu, cpu_interrupt: CpuInterrupt) -> Priority {
        priority(cpu_interrupt)
    }

    /// Get interrupt priority - called by assembly code
    #[inline]
    pub(super) fn priority(cpu_interrupt: CpuInterrupt) -> Priority {
        let intr = INTERRUPT_CORE0::regs();
        unsafe {
            core::mem::transmute::<u8, Priority>(
                intr.cpu_int_pri(cpu_interrupt as usize).read().map().bits(),
            )
        }
    }

    /// Get the current run level (the level below which interrupts are masked).
    pub fn current_runlevel() -> Priority {
        let intr = INTERRUPT_CORE0::regs();
        let prev_interrupt_priority = intr.cpu_int_thresh().read().bits().saturating_sub(1) as u8;

        unwrap!(Priority::try_from(prev_interrupt_priority))
    }

    /// Changes the current run level (the level below which interrupts are
    /// masked), and returns the previous run level.
    ///
    /// # Safety
    ///
    /// This function must only be used to raise the runlevel and to restore it
    /// to a previous value. It must not be used to arbitrarily lower the
    /// runlevel.
    pub(crate) unsafe fn change_current_runlevel(level: Priority) -> Priority {
        let prev_interrupt_priority = current_runlevel();

        // The CPU responds to interrupts `>= level`, but we want to also disable
        // interrupts at `level` so we set the threshold to `level + 1`.
        INTERRUPT_CORE0::regs()
            .cpu_int_thresh()
            .write(|w| unsafe { w.bits(level as u32 + 1) });

        prev_interrupt_priority
    }
}

#[cfg(clic)]
mod clic {
    use super::{CpuInterrupt, InterruptKind, Priority};
    use crate::{soc::pac::CLIC, system::Cpu};

    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static DISABLED_CPU_INTERRUPT: u32 = 0;

    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static PRIORITY_TO_INTERRUPT: &[u32] = &[16, 17, 18, 19, 20, 21, 22, 23];

    /// Set up CLIC.
    pub(super) fn init() {
        let clic = unsafe { CLIC::steal() };

        // Set 3 level bits = 8 priority levels
        clic.int_config()
            .modify(|_, w| unsafe { w.mnlbits().bits(3) });

        // Enable hardware vectoring
        for int in clic.int_attr_iter() {
            int.modify(|_, w| {
                w.shv().hardware();
                w.trig().positive_edge()
            });
        }
    }

    /// Enable a CPU interrupt
    ///
    /// # Safety
    ///
    /// Make sure there is an interrupt handler registered.
    pub unsafe fn enable_cpu_interrupt(cpu_interrupt: CpuInterrupt) {
        // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
        let clic = unsafe { CLIC::steal() };
        clic.int_ie(cpu_interrupt as usize)
            .write(|w| w.int_ie().set_bit());
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// This is safe to call when the `vectored` feature is enabled. The
    /// vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    pub fn set_kind(_core: Cpu, cpu_interrupt: CpuInterrupt, kind: InterruptKind) {
        // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
        let clic = unsafe { CLIC::steal() };
        clic.int_attr(cpu_interrupt as usize)
            .modify(|_, w| match kind {
                InterruptKind::Level => w.trig().positive_level(),
                InterruptKind::Edge => w.trig().positive_edge(),
            });
    }

    /// Set the priority level of an CPU interrupt
    ///
    /// # Safety
    ///
    /// Great care must be taken when using the `vectored` feature (enabled by
    /// default). Avoid changing the priority of interrupts 1 - 8 when
    /// interrupt vectoring is enabled.
    pub unsafe fn set_priority(_core: Cpu, cpu_interrupt: CpuInterrupt, priority: Priority) {
        // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.

        let clic = unsafe { CLIC::steal() };
        let prio_bits = prio_to_bits(priority);
        // The `ctl` field would only write the 3 programmable bits, but we have the correct final
        // value anyway so let's write it directly.
        clic.int_ctl(cpu_interrupt as usize)
            .write(|w| unsafe { w.bits(prio_bits) });
    }

    /// Clear a CPU interrupt
    #[inline]
    pub fn clear(_core: Cpu, cpu_interrupt: CpuInterrupt) {
        // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
        let clic = unsafe { CLIC::steal() };
        clic.int_ip(cpu_interrupt as usize)
            .write(|w| w.int_ip().clear_bit());
    }

    /// Get interrupt priority
    #[inline]
    pub(super) fn priority_by_core(_core: Cpu, cpu_interrupt: CpuInterrupt) -> Priority {
        priority(cpu_interrupt)
    }

    #[inline]
    /// Get interrupt priority.
    pub fn priority(cpu_interrupt: CpuInterrupt) -> Priority {
        // Lower 16 interrupts are reserved for CLINT, which is currently not implemented.
        let clic = unsafe { CLIC::steal() };
        let prio_level = clic.int_ctl(cpu_interrupt as usize).read().bits() as usize;
        bits_to_prio(prio_level)
    }

    /// Changes the current interrupt runlevel (the level below which interrupts are masked),
    /// and returns the previous runlevel.
    pub(crate) unsafe fn change_current_runlevel(level: Priority) -> Priority {
        let current_runlevel = current_runlevel();

        // All machine mode pending interrupts with levels less than or equal
        // to the effective threshold level are not allowed to preempt the execution.
        unsafe { mintthresh::write(prio_to_bits(level) as usize) };

        current_runlevel
    }

    /// Get the current run level (the level below which interrupts are masked).
    pub(crate) fn mil() -> usize {
        mintstatus::read().mil()
    }

    /// Get the current run level (the level below which interrupts are masked).
    pub fn current_runlevel() -> Priority {
        let mintthresh = mintthresh::read();

        let mil = mil();

        let level = mil.max(mintthresh);

        let prio = bits_to_prio(level);

        prio
    }

    fn prio_to_bits(priority: Priority) -> u8 {
        if priority == Priority::None {
            0
        } else {
            0x1F | ((priority as u8 - 1) << 5)
        }
    }

    fn bits_to_prio(bits: usize) -> Priority {
        // If mintthresh starts from 0xf, make sure we don't return Priority1
        if bits < 0x1f {
            Priority::None
        } else {
            unwrap!(Priority::try_from(((bits >> 5) + 1) as u32))
        }
    }

    mod mintthresh {
        riscv::read_csr_as_usize!(0x347);
        riscv::write_csr_as_usize!(0x347);
    }

    mod mintstatus {
        // Work around riscv using a non-qualified `assert!` in constants
        macro_rules! assert {
            ($($tt:tt)*) => {
                ::core::assert!($($tt)*)
            };
        }
        riscv::read_only_csr! {
            /// `mintstatus` register
            Mintstatus: 0xfb1,
            mask: usize::MAX,
        }
        riscv::read_only_csr_field! {
            Mintstatus,
            /// Returns the `mil` field.
            mil: [24:31],
        }
    }

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
}

#[cfg(plic)]
mod plic {
    use super::{CpuInterrupt, InterruptKind, Priority};
    use crate::{peripherals::PLIC_MX, system::Cpu};

    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static DISABLED_CPU_INTERRUPT: u32 = 31;

    // 0,3,4,7 are reserved for CLINT
    // for some reason also CPU interrupt 8 doesn't work by default since it's
    // disabled after reset - so don't use that, too
    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static PRIORITY_TO_INTERRUPT: &[u32] =
        &[1, 2, 5, 6, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19];

    // First element is not used, just there to avoid a -1 in the interrupt handler.
    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static INTERRUPT_TO_PRIORITY: [Priority; 20] = [
        Priority::None,
        Priority::Priority1,
        Priority::Priority2,
        Priority::None,
        Priority::None,
        Priority::Priority3,
        Priority::Priority4,
        Priority::None,
        Priority::None,
        Priority::Priority5,
        Priority::Priority6,
        Priority::Priority7,
        Priority::Priority8,
        Priority::Priority9,
        Priority::Priority10,
        Priority::Priority11,
        Priority::Priority12,
        Priority::Priority13,
        Priority::Priority14,
        Priority::Priority15,
    ];

    /// Enable a CPU interrupt
    ///
    /// # Safety
    ///
    /// Make sure there is an interrupt handler registered.
    pub unsafe fn enable_cpu_interrupt(which: CpuInterrupt) {
        PLIC_MX::regs().mxint_enable().modify(|r, w| {
            let old = r.cpu_mxint_enable().bits();
            let new = old | (1 << (which as isize));
            unsafe { w.cpu_mxint_enable().bits(new) }
        });
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// The vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    pub fn set_kind(_core: Cpu, which: CpuInterrupt, kind: InterruptKind) {
        let interrupt_type = match kind {
            InterruptKind::Level => 0,
            InterruptKind::Edge => 1,
        };

        PLIC_MX::regs().mxint_type().modify(|r, w| {
            let old = r.cpu_mxint_type().bits();
            let new = old & !(1 << (which as isize)) | (interrupt_type << (which as isize));
            unsafe { w.cpu_mxint_type().bits(new) }
        });
    }

    /// Set the priority level of an CPU interrupt
    ///
    /// # Safety
    ///
    /// Great care must be taken when using this function; avoid changing the
    /// priority of interrupts 1 - 15.
    pub unsafe fn set_priority(_core: Cpu, which: CpuInterrupt, priority: Priority) {
        PLIC_MX::regs()
            .mxint_pri(which as usize)
            .modify(|_, w| unsafe { w.cpu_mxint_pri().bits(priority as u8) });
    }

    /// Clear a CPU interrupt
    #[inline]
    pub fn clear(_core: Cpu, which: CpuInterrupt) {
        PLIC_MX::regs().mxint_clear().modify(|r, w| {
            let old = r.cpu_mxint_clear().bits();
            let new = old | (1 << (which as isize));
            unsafe { w.cpu_mxint_clear().bits(new) }
        });
    }

    /// Get interrupt priority for the CPU
    #[inline]
    pub fn priority_by_core(_core: Cpu, cpu_interrupt: CpuInterrupt) -> Priority {
        priority(cpu_interrupt)
    }

    #[inline]
    /// Get interrupt priority.
    pub fn priority(cpu_interrupt: CpuInterrupt) -> Priority {
        let prio = PLIC_MX::regs()
            .mxint_pri(cpu_interrupt as usize)
            .read()
            .cpu_mxint_pri()
            .bits();
        unsafe { core::mem::transmute::<u8, Priority>(prio) }
    }

    /// Get the current run level (the level below which interrupts are masked).
    pub fn current_runlevel() -> Priority {
        let prev_interrupt_priority = PLIC_MX::regs()
            .mxint_thresh()
            .read()
            .cpu_mxint_thresh()
            .bits()
            .saturating_sub(1);

        unwrap!(Priority::try_from(prev_interrupt_priority))
    }

    /// Changes the current run level (the level below which interrupts are
    /// masked), and returns the previous run level.
    ///
    /// # Safety
    ///
    /// This function must only be used to raise the runlevel and to restore it
    /// to a previous value. It must not be used to arbitrarily lower the
    /// runlevel.
    pub(crate) unsafe fn change_current_runlevel(level: Priority) -> Priority {
        let prev_interrupt_priority = current_runlevel();

        // The CPU responds to interrupts `>= level`, but we want to also disable
        // interrupts at `level` so we set the threshold to `level + 1`.
        PLIC_MX::regs()
            .mxint_thresh()
            .write(|w| unsafe { w.cpu_mxint_thresh().bits(level as u8 + 1) });

        prev_interrupt_priority
    }
}

#[cfg(feature = "rt")]
mod rt {
    use esp_riscv_rt::TrapFrame;

    use super::*;

    /// # Safety
    ///
    /// This function is called from an assembly trap handler.
    #[doc(hidden)]
    #[unsafe(link_section = ".trap.rust")]
    #[unsafe(export_name = "_start_trap_rust_hal")]
    unsafe extern "C" fn start_trap_rust_hal(trap_frame: *mut TrapFrame) {
        assert!(
            mcause::read().is_exception(),
            "Arrived into _start_trap_rust_hal but mcause is not an exception!"
        );
        unsafe extern "C" {
            fn ExceptionHandler(tf: *mut TrapFrame);
        }
        unsafe {
            ExceptionHandler(trap_frame);
        }
    }

    #[doc(hidden)]
    #[unsafe(no_mangle)]
    unsafe fn _setup_interrupts() {
        unsafe extern "C" {
            static _vector_table: u32;
            #[cfg(clic)]
            static _mtvt_table: u32;
        }

        // disable all known interrupts
        // at least after the 2nd stage bootloader there are some interrupts enabled
        // (e.g. UART)
        for peripheral_interrupt in 0..255 {
            crate::peripherals::Interrupt::try_from(peripheral_interrupt)
                .map(|intr| {
                    #[cfg(multi_core)]
                    disable(Cpu::AppCpu, intr);
                    disable(Cpu::ProCpu, intr);
                })
                .ok();
        }

        #[cfg(clic)]
        clic::init();

        unsafe {
            let vec_table = (&raw const _vector_table).addr();

            #[cfg(not(clic))]
            mtvec::write({
                let mut mtvec = mtvec::Mtvec::from_bits(0);
                mtvec.set_trap_mode(mtvec::TrapMode::Vectored);
                mtvec.set_address(vec_table);
                mtvec
            });
            #[cfg(clic)]
            {
                mtvec::write({
                    let mut mtvec = mtvec::Mtvec::from_bits(0x03); // MODE = CLIC
                    mtvec.set_address(vec_table);
                    mtvec
                });

                // set mtvt (hardware vector base)
                let mtvt_table = (&raw const _mtvt_table).addr();
                core::arch::asm!("csrw 0x307, {0}", in(reg) mtvt_table);
            }
            crate::interrupt::init_vectoring();
        };

        #[cfg(plic)]
        unsafe {
            core::arch::asm!("csrw mie, {0}", in(reg) u32::MAX);
        }
    }

    #[unsafe(no_mangle)]
    #[ram]
    #[cfg(not(clic))]
    unsafe fn handle_interrupts(cpu_intr: CpuInterrupt) {
        let core = Cpu::current();
        let status = status(core);

        // this has no effect on level interrupts, but the interrupt may be an edge one
        // so we clear it anyway
        clear(core, cpu_intr);

        let prio = INTERRUPT_TO_PRIORITY[cpu_intr as usize];
        let configured_interrupts = vectored::configured_interrupts(core, status, prio);

        // Change the current runlevel so that interrupt handlers can access the correct runlevel.
        let level = unsafe { change_current_runlevel(prio) };

        // When nesting is possible, we run the nestable interrupts first. This ensures that we
        // don't violate assumptions made by non-nestable handlers.

        if prio != Priority::max() {
            for interrupt_nr in configured_interrupts.iterator() {
                let handler =
                    unsafe { pac::__EXTERNAL_INTERRUPTS[interrupt_nr as usize]._handler } as usize;
                let nested = (handler & 1) == 0;
                if nested {
                    let handler: fn() =
                        unsafe { core::mem::transmute::<usize, fn()>(handler & !1) };

                    unsafe { riscv::interrupt::nested(handler) };
                }
            }
        }

        // Now we can run the non-nestable interrupt handlers.

        for interrupt_nr in configured_interrupts.iterator() {
            let handler =
                unsafe { pac::__EXTERNAL_INTERRUPTS[interrupt_nr as usize]._handler } as usize;
            let not_nested = (handler & 1) == 1;
            if not_nested || prio == Priority::max() {
                let handler: fn() = unsafe { core::mem::transmute::<usize, fn()>(handler & !1) };

                handler();
            }
        }

        unsafe { change_current_runlevel(level) };
    }

    #[unsafe(no_mangle)]
    #[ram]
    #[cfg(clic)]
    unsafe fn handle_interrupts(cpu_intr: CpuInterrupt) {
        fn handle(cpu_intr: CpuInterrupt) {
            let core = Cpu::current();
            let status = status(core);

            // this has no effect on level interrupts, but the interrupt may be an edge one
            // so we clear it anyway
            clear(core, cpu_intr);

            let prio = clic::current_runlevel();
            let configured_interrupts = vectored::configured_interrupts(core, status, prio);

            for interrupt_nr in configured_interrupts.iterator() {
                let handler =
                    unsafe { pac::__EXTERNAL_INTERRUPTS[interrupt_nr as usize]._handler } as usize;

                let handler: fn() = unsafe { core::mem::transmute::<usize, fn()>(handler) };

                handler();
            }
        }

        unsafe { riscv::interrupt::nested(|| handle(cpu_intr)) };
    }
}
