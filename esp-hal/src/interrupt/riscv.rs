//! Interrupt handling
//!
//! Peripheral interrupts go through the interrupt matrix, which routes them to the appropriate CPU
//! interrupt line. The interrupt matrix is largely the same across devices, but CPU interrupts are
//! device-specific before CLIC.
//!
//! Peripheral interrupts can be bound directly to CPU interrupts for better performance, but
//! due to the limited number of CPU interrupts, the preferred mechanism is to use the vectored
//! interrupts. The vectored interrupt handlers will call the appropriate interrupt handlers.
//! The configuration of vectored interrupt handlers cannot be changed in runtime.

#[cfg(feature = "rt")]
#[instability::unstable]
pub use esp_riscv_rt::TrapFrame;

#[cfg_attr(interrupt_controller = "riscv_basic", path = "riscv/basic.rs")]
#[cfg_attr(interrupt_controller = "plic", path = "riscv/plic.rs")]
#[cfg_attr(interrupt_controller = "clic", path = "riscv/clic.rs")]
mod cpu_int;

use crate::{
    interrupt::{PriorityError, RunLevel},
    peripherals::Interrupt,
    system::Cpu,
};

/// Interrupt kind
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum InterruptKind {
    /// Level interrupt
    Level,
    /// Edge interrupt
    Edge,
}

for_each_interrupt!(
    (all $( ([$class:ident $idx_in_class:literal] $n:literal) ),*) => {
        paste::paste! {
            /// Enumeration of available CPU interrupts.
            #[repr(u32)]
            #[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            #[instability::unstable]
            pub enum CpuInterrupt {
                $(
                    #[doc = concat!(" Interrupt number ", stringify!($n), ".")]
                    [<Interrupt $n>] = $n,
                )*
            }

            impl CpuInterrupt {
                #[inline]
                pub(crate) fn from_u32(n: u32) -> Option<Self> {
                    match n {
                        $(n if n == $n && n != DISABLED_CPU_INTERRUPT => Some(Self:: [<Interrupt $n>]),)*
                        _ => None
                    }
                }
            }
        }
    };
);

for_each_classified_interrupt!(
    (direct_bindable $( ([$class:ident $idx_in_class:literal] $n:literal) ),*) => {
        paste::paste! {
            /// Enumeration of CPU interrupts available for direct binding.
            #[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
            #[cfg_attr(feature = "defmt", derive(defmt::Format))]
            pub enum DirectBindableCpuInterrupt {
                $(
                    #[doc = concat!(" Direct bindable CPU interrupt number ", stringify!($idx_in_class), ".")]
                    #[doc = " "]
                    #[doc = concat!(" Corresponds to CPU interrupt ", stringify!($n), ".")]
                    [<Interrupt $idx_in_class>] = $n,
                )*
            }

            impl From<DirectBindableCpuInterrupt> for CpuInterrupt {
                fn from(bindable: DirectBindableCpuInterrupt) -> CpuInterrupt {
                    match bindable {
                        $(
                            DirectBindableCpuInterrupt::[<Interrupt $idx_in_class>] => CpuInterrupt::[<Interrupt $n>],
                        )*
                    }
                }
            }
        }
    };
);

impl CpuInterrupt {
    #[inline]
    #[cfg(feature = "rt")]
    pub(crate) fn is_vectored(self) -> bool {
        // Assumes contiguous interrupt allocation.
        const VECTORED_CPU_INTERRUPT_RANGE: core::ops::RangeInclusive<u32> = PRIORITY_TO_INTERRUPT
            [0] as u32
            ..=PRIORITY_TO_INTERRUPT[PRIORITY_TO_INTERRUPT.len() - 1] as u32;
        VECTORED_CPU_INTERRUPT_RANGE.contains(&(self as u32))
    }

    /// Enable the CPU interrupt
    #[inline]
    #[instability::unstable]
    pub fn enable(self) {
        cpu_int::enable_cpu_interrupt_raw(self as u32);
    }

    /// Clear the CPU interrupt status bit
    #[inline]
    #[instability::unstable]
    pub fn clear(self) {
        cpu_int::clear_raw(self as u32);
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// This is safe to call when the `vectored` feature is enabled. The
    /// vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    #[inline]
    #[instability::unstable]
    pub fn set_kind(self, kind: InterruptKind) {
        cpu_int::set_kind_raw(self as u32, kind);
    }

    /// Set the priority level of a CPU interrupt
    #[inline]
    #[instability::unstable]
    pub fn set_priority(self, priority: Priority) {
        cpu_int::set_priority_raw(self as u32, priority);
    }

    /// Get interrupt priority for the CPU
    #[inline]
    #[instability::unstable]
    pub fn priority(self) -> Priority {
        unwrap!(Priority::try_from_u32(self.level()))
    }

    #[inline]
    pub(crate) fn level(self) -> u32 {
        cpu_int::cpu_interrupt_priority_raw(self as u32) as u32
    }
}

for_each_interrupt_priority!(
    (all $( ($idx:literal, $n:literal, $ident:ident) ),*) => {
        /// Interrupt priority levels.
        ///
        /// A higher numeric value means higher priority. Interrupt requests at higher priority
        /// levels will be able to preempt code running at a lower [`RunLevel`][super::RunLevel].
        #[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[repr(u8)]
        pub enum Priority {
            $(
                #[doc = concat!(" Priority level ", stringify!($n), ".")]
                $ident = $n,
            )*
        }

        impl Priority {
            fn iter() -> impl Iterator<Item = Priority> {
                [$(Priority::$ident,)*].into_iter()
            }
        }
    };
);

impl Priority {
    /// Maximum interrupt priority
    #[allow(unused_assignments)]
    #[instability::unstable]
    pub const fn max() -> Priority {
        const {
            let mut last = Self::min();
            for_each_interrupt_priority!(
                ($_idx:literal, $_n:literal, $ident:ident) => {
                    last = Self::$ident;
                };
            );
            last
        }
    }

    /// Minimum interrupt priority
    pub const fn min() -> Priority {
        Priority::Priority1
    }

    pub(crate) fn try_from_u32(priority: u32) -> Result<Self, PriorityError> {
        let result;
        for_each_interrupt_priority!(
            (all $( ($idx:literal, $n:literal, $ident:ident) ),*) => {
                result = match priority {
                    $($n => Ok(Priority::$ident),)*
                    _ => Err(PriorityError::InvalidInterruptPriority),
                }
            };
        );
        result
    }
}

#[instability::unstable]
impl TryFrom<u32> for Priority {
    type Error = PriorityError;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Self::try_from_u32(value)
    }
}

#[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
pub(super) static DISABLED_CPU_INTERRUPT: u32 = property!("interrupts.disabled_interrupt");

/// The number of vectored interrupts / The number of priority levels.
const VECTOR_COUNT: usize = const {
    let mut count = 0;
    for_each_interrupt!(([vector $n:tt] $_:literal) => { count += 1; };);

    core::assert!(count == Priority::max() as usize);

    count
};

/// Maps priority levels to their corresponding interrupt vectors.
#[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
pub(super) static PRIORITY_TO_INTERRUPT: [CpuInterrupt; VECTOR_COUNT] = const {
    let mut counter = 0;
    let mut vector = [CpuInterrupt::Interrupt0; VECTOR_COUNT];

    for_each_interrupt!(
        ([vector $_n:tt] $interrupt:literal) => {
            vector[counter] = paste::paste! { CpuInterrupt::[<Interrupt $interrupt>] };
            counter += 1;
        };
    );
    vector
};

/// Enable an interrupt by directly binding it to an available CPU interrupt
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
/// you most likely want to use [`enable`][crate::interrupt::enable] instead.
#[instability::unstable]
pub fn enable_direct(
    interrupt: Interrupt,
    level: Priority,
    cpu_interrupt: DirectBindableCpuInterrupt,
    handler: unsafe extern "C" fn(),
) {
    cfg_if::cfg_if! {
        if #[cfg(interrupt_controller = "clic")] {
            let clic = unsafe { crate::soc::pac::CLIC::steal() };

            // Enable hardware vectoring
            clic.int_attr(cpu_interrupt as usize).modify(|_, w| {
                w.shv().hardware();
                w.trig().positive_level()
            });

            let mtvt_table: *mut [u32; 48];
            unsafe { core::arch::asm!("csrr {0}, 0x307", out(reg) mtvt_table) };

            let int_slot = mtvt_table
                .cast::<u32>()
                .wrapping_add(cpu_interrupt as usize);

            let instr = handler as usize as u32;
        } else {
            use riscv::register::mtvec;
            let mt = mtvec::read();

            assert_eq!(
                mt.trap_mode().into_usize(),
                mtvec::TrapMode::Vectored.into_usize()
            );

            let base_addr = mt.address() as usize;

            let int_slot = base_addr.wrapping_add((cpu_interrupt as usize) * 4) as *mut u32;

            let instr = encode_jal_x0(handler as usize, int_slot as usize);
        }
    }

    if crate::debugger::debugger_connected() {
        unsafe { core::ptr::write_volatile(int_slot, instr) };
    } else {
        crate::debugger::DEBUGGER_LOCK.lock(|| unsafe {
            let wp = crate::debugger::clear_watchpoint(1);
            core::ptr::write_volatile(int_slot, instr);
            crate::debugger::restore_watchpoint(1, wp);
        });
    }
    unsafe {
        core::arch::asm!("fence.i");
    }

    super::map_raw(Cpu::current(), interrupt, cpu_interrupt as u32);
    cpu_int::set_priority_raw(cpu_interrupt as u32, level);
    cpu_int::set_kind_raw(cpu_interrupt as u32, InterruptKind::Level);
    cpu_int::enable_cpu_interrupt_raw(cpu_interrupt as u32);
}

// helper: returns correctly encoded RISC-V `jal` instruction
#[cfg(not(interrupt_controller = "clic"))]
fn encode_jal_x0(target: usize, pc: usize) -> u32 {
    let offset = (target as isize) - (pc as isize);

    const MIN: isize = -(1isize << 20);
    const MAX: isize = (1isize << 20) - 1;

    assert!(offset % 2 == 0 && (MIN..=MAX).contains(&offset));

    let imm = offset as u32;
    let imm20 = (imm >> 20) & 0x1;
    let imm10_1 = (imm >> 1) & 0x3ff;
    let imm11 = (imm >> 11) & 0x1;
    let imm19_12 = (imm >> 12) & 0xff;

    (imm20 << 31)
        | (imm19_12 << 12)
        | (imm11 << 20)
        | (imm10_1 << 21)
        // https://lhtin.github.io/01world/app/riscv-isa/?xlen=32&insn_name=jal
        | 0b1101111u32
}

// Runlevel APIs

/// Get the current run level (the level below which interrupts are masked).
pub(crate) fn current_runlevel() -> RunLevel {
    let priority = cpu_int::current_runlevel();
    unwrap!(RunLevel::try_from_u32(priority as u32))
}

/// Changes the current run level (the level below which interrupts are
/// masked), and returns the previous run level.
///
/// # Safety
///
/// This function must only be used to raise the runlevel and to restore it
/// to a previous value. It must not be used to arbitrarily lower the
/// runlevel.
pub(crate) unsafe fn change_current_runlevel(level: RunLevel) -> RunLevel {
    let previous = cpu_int::change_current_runlevel(level);
    unwrap!(RunLevel::try_from_u32(previous as u32))
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
#[instability::unstable]
pub fn wait_for_interrupt() {
    if crate::debugger::debugger_connected() && !cpu_wait_mode_on() {
        // when SYSTEM_CPU_WAIT_MODE_FORCE_ON is disabled in WFI mode SBA access to memory does not
        // work for debugger, so do not enter that mode when debugger is connected.
        // https://github.com/espressif/esp-idf/blob/b9a308a47ca4128d018495662b009a7c461b6780/components/esp_hw_support/cpu.c#L57-L60
        return;
    }
    unsafe { core::arch::asm!("wfi") };
}

pub(crate) fn priority_to_cpu_interrupt(_interrupt: Interrupt, level: Priority) -> CpuInterrupt {
    PRIORITY_TO_INTERRUPT[(level as usize) - 1]
}

/// Setup interrupts ready for vectoring
///
/// # Safety
///
/// This function must be called only during core startup.
#[cfg(any(feature = "rt", all(feature = "unstable", multi_core)))]
pub(crate) unsafe fn init_vectoring() {
    use riscv::register::mtvec;

    unsafe extern "C" {
        static _vector_table: u32;
        #[cfg(interrupt_controller = "clic")]
        static _mtvt_table: u32;
    }

    unsafe {
        let vec_table = (&raw const _vector_table).addr();

        #[cfg(not(interrupt_controller = "clic"))]
        {
            mtvec::write({
                let mut mtvec = mtvec::Mtvec::from_bits(0);
                mtvec.set_trap_mode(mtvec::TrapMode::Vectored);
                mtvec.set_address(vec_table);
                mtvec
            });
        }

        #[cfg(interrupt_controller = "clic")]
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
    };

    // Configure and enable vectored interrupts
    for (int, prio) in PRIORITY_TO_INTERRUPT.iter().copied().zip(Priority::iter()) {
        let num = int as u32;
        cpu_int::set_kind_raw(num, InterruptKind::Level);
        cpu_int::set_priority_raw(num, prio);
        cpu_int::enable_cpu_interrupt_raw(num);
    }
}

#[cfg(feature = "rt")]
pub(crate) mod rt {
    use esp_riscv_rt::TrapFrame;
    use riscv::register::mcause;

    use super::*;
    use crate::interrupt::InterruptStatus;

    /// The total number of interrupts.
    #[cfg(not(interrupt_controller = "clic"))]
    const INTERRUPT_COUNT: usize = const {
        let mut count = 0;
        for_each_interrupt!(([$_class:tt $n:tt] $_:literal) => { count += 1; };);
        count
    };

    /// Maps interrupt numbers to their vector priority levels.
    #[cfg(not(interrupt_controller = "clic"))]
    #[cfg_attr(place_switch_tables_in_ram, unsafe(link_section = ".rwtext"))]
    pub(super) static INTERRUPT_TO_PRIORITY: [Option<Priority>; INTERRUPT_COUNT] = const {
        let mut priorities = [None; INTERRUPT_COUNT];

        for_each_interrupt!(
            ([vector $n:tt] $int:literal) => {
                for_each_interrupt_priority!(($n, $__:tt, $ident:ident) => { priorities[$int] = Some(Priority::$ident); };);
            };
        );

        priorities
    };

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
        crate::interrupt::setup_interrupts();

        cpu_int::init();

        #[cfg(interrupt_controller = "plic")]
        unsafe {
            core::arch::asm!("csrw mie, {0}", in(reg) u32::MAX);
        }
    }

    #[unsafe(no_mangle)]
    #[crate::ram]
    unsafe fn handle_interrupts(cpu_intr: CpuInterrupt) {
        let status = InterruptStatus::current();

        // this has no effect on level interrupts, but the interrupt may be an edge one
        // so we clear it anyway
        cpu_intr.clear();

        cfg_if::cfg_if! {
            if #[cfg(interrupt_controller = "clic")] {
                let prio = unwrap!(Priority::try_from_u32(cpu_int::current_runlevel() as u32));
                let mcause = riscv::register::mcause::read();
            } else {
                // Change the current runlevel so that interrupt handlers can access the correct runlevel.
                let prio = unwrap!(INTERRUPT_TO_PRIORITY[cpu_intr as usize]);
                let level = unsafe { change_current_runlevel(RunLevel::Interrupt(prio)) };
            }
        }

        let handle_interrupts = || unsafe {
            for interrupt_nr in status.iterator().filter(|&interrupt_nr| {
                crate::interrupt::should_handle(Cpu::current(), interrupt_nr as u32, prio as u32)
            }) {
                let handler =
                    crate::soc::pac::__EXTERNAL_INTERRUPTS[interrupt_nr as usize]._handler;

                handler();
            }
        };

        // Do not enable nesting on the highest priority level. Older interrupt controllers couldn't
        // properly mask the highest priority interrupt, and for CLIC we don't want to waste
        // the cycles it takes to enable nesting unnecessarily.
        if prio != Priority::max() {
            unsafe {
                riscv::interrupt::nested(handle_interrupts);
            }
        } else {
            handle_interrupts();
        }

        cfg_if::cfg_if! {
            if #[cfg(interrupt_controller = "clic")] {
                // In case the target uses the CLIC, it is mandatory to restore `mcause` register
                // since it contains the former CPU priority. When executing `mret`,
                // the hardware will restore the former threshold, from `mcause` to
                // `mintstatus` CSR
                unsafe {
                    core::arch::asm!("csrw 0x342, {}", in(reg) mcause.bits())
                }
            } else {
                unsafe { change_current_runlevel(level) };
            }
        }
    }
}
