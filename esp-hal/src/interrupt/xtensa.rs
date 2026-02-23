//! Interrupt handling

#[cfg(esp32)]
pub(crate) use xtensa_lx::interrupt::free;

use crate::{
    interrupt::{PriorityError, RunLevel},
    peripherals::Interrupt,
};

/// Enumeration of available CPU interrupts
///
/// It's possible to create one handler per priority level. (e.g
/// `level1_interrupt`)
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u32)]
#[instability::unstable]
pub enum CpuInterrupt {
    /// Level-triggered interrupt with priority 1.
    Interrupt0LevelPriority1      = 0,
    /// Level-triggered interrupt with priority 1.
    Interrupt1LevelPriority1      = 1,
    /// Level-triggered interrupt with priority 1.
    Interrupt2LevelPriority1      = 2,
    /// Level-triggered interrupt with priority 1.
    Interrupt3LevelPriority1      = 3,
    /// Level-triggered interrupt with priority 1.
    Interrupt4LevelPriority1      = 4,
    /// Level-triggered interrupt with priority 1.
    Interrupt5LevelPriority1      = 5,
    /// Timer 0 interrupt with priority 1.
    Interrupt6Timer0Priority1     = 6,
    /// Software-triggered interrupt with priority 1.
    Interrupt7SoftwarePriority1   = 7,
    /// Level-triggered interrupt with priority 1.
    Interrupt8LevelPriority1      = 8,
    /// Level-triggered interrupt with priority 1.
    Interrupt9LevelPriority1      = 9,
    /// Edge-triggered interrupt with priority 1.
    Interrupt10EdgePriority1      = 10,
    /// Profiling-related interrupt with priority 3.
    Interrupt11ProfilingPriority3 = 11,
    /// Level-triggered interrupt with priority 1.
    Interrupt12LevelPriority1     = 12,
    /// Level-triggered interrupt with priority 1.
    Interrupt13LevelPriority1     = 13,
    /// Timer 1 interrupt with priority 3.
    Interrupt15Timer1Priority3    = 15,
    /// Level-triggered interrupt with priority 1.
    Interrupt17LevelPriority1     = 17,
    /// Level-triggered interrupt with priority 1.
    Interrupt18LevelPriority1     = 18,
    /// Level-triggered interrupt with priority 2.
    Interrupt19LevelPriority2     = 19,
    /// Level-triggered interrupt with priority 2.
    Interrupt20LevelPriority2     = 20,
    /// Level-triggered interrupt with priority 2.
    Interrupt21LevelPriority2     = 21,
    /// Edge-triggered interrupt with priority 3.
    Interrupt22EdgePriority3      = 22,
    /// Level-triggered interrupt with priority 3.
    Interrupt23LevelPriority3     = 23,
    /// Level-triggered interrupt with priority 3.
    Interrupt27LevelPriority3     = 27,
    /// Software-triggered interrupt with priority 3.
    Interrupt29SoftwarePriority3  = 29,
    // TODO: re-add higher level interrupts
}

impl CpuInterrupt {
    pub(super) fn from_u32(n: u32) -> Option<Self> {
        match n {
            0 => Some(Self::Interrupt0LevelPriority1),
            1 => Some(Self::Interrupt1LevelPriority1),
            2 => Some(Self::Interrupt2LevelPriority1),
            3 => Some(Self::Interrupt3LevelPriority1),
            4 => Some(Self::Interrupt4LevelPriority1),
            5 => Some(Self::Interrupt5LevelPriority1),
            6 => Some(Self::Interrupt6Timer0Priority1),
            7 => Some(Self::Interrupt7SoftwarePriority1),
            8 => Some(Self::Interrupt8LevelPriority1),
            9 => Some(Self::Interrupt9LevelPriority1),
            10 => Some(Self::Interrupt10EdgePriority1),
            11 => Some(Self::Interrupt11ProfilingPriority3),
            12 => Some(Self::Interrupt12LevelPriority1),
            13 => Some(Self::Interrupt13LevelPriority1),
            15 => Some(Self::Interrupt15Timer1Priority3),
            17 => Some(Self::Interrupt17LevelPriority1),
            18 => Some(Self::Interrupt18LevelPriority1),
            19 => Some(Self::Interrupt19LevelPriority2),
            20 => Some(Self::Interrupt20LevelPriority2),
            21 => Some(Self::Interrupt21LevelPriority2),
            22 => Some(Self::Interrupt22EdgePriority3),
            23 => Some(Self::Interrupt23LevelPriority3),
            27 => Some(Self::Interrupt27LevelPriority3),
            29 => Some(Self::Interrupt29SoftwarePriority3),
            _ => None,
        }
    }

    #[inline]
    #[cfg(feature = "rt")]
    pub(crate) fn is_vectored(self) -> bool {
        // Even "direct bound" interrupts go through the vectored interrupt handler
        true
    }

    /// Enable the CPU interrupt
    #[inline]
    #[instability::unstable]
    pub fn enable(self) {
        enable_cpu_interrupt_raw(self as u32);
    }

    /// Clear the CPU interrupt status bit
    #[inline]
    #[instability::unstable]
    pub fn clear(self) {
        unsafe { xtensa_lx::interrupt::clear(1 << self as u32) };
    }

    /// Get interrupt priority for the CPU
    #[inline]
    #[instability::unstable]
    pub fn priority(self) -> Priority {
        match self {
            CpuInterrupt::Interrupt0LevelPriority1
            | CpuInterrupt::Interrupt1LevelPriority1
            | CpuInterrupt::Interrupt2LevelPriority1
            | CpuInterrupt::Interrupt3LevelPriority1
            | CpuInterrupt::Interrupt4LevelPriority1
            | CpuInterrupt::Interrupt5LevelPriority1
            | CpuInterrupt::Interrupt6Timer0Priority1
            | CpuInterrupt::Interrupt7SoftwarePriority1
            | CpuInterrupt::Interrupt8LevelPriority1
            | CpuInterrupt::Interrupt9LevelPriority1
            | CpuInterrupt::Interrupt10EdgePriority1
            | CpuInterrupt::Interrupt12LevelPriority1
            | CpuInterrupt::Interrupt13LevelPriority1
            | CpuInterrupt::Interrupt17LevelPriority1
            | CpuInterrupt::Interrupt18LevelPriority1 => Priority::Priority1,

            CpuInterrupt::Interrupt19LevelPriority2
            | CpuInterrupt::Interrupt20LevelPriority2
            | CpuInterrupt::Interrupt21LevelPriority2 => Priority::Priority2,

            CpuInterrupt::Interrupt11ProfilingPriority3
            | CpuInterrupt::Interrupt15Timer1Priority3
            | CpuInterrupt::Interrupt22EdgePriority3
            | CpuInterrupt::Interrupt27LevelPriority3
            | CpuInterrupt::Interrupt29SoftwarePriority3
            | CpuInterrupt::Interrupt23LevelPriority3 => Priority::Priority3,
        }
    }

    #[inline]
    #[cfg(feature = "rt")]
    pub(crate) fn level(self) -> u32 {
        self.priority() as u32
    }
}

/// Interrupt priority levels.
///
/// A higher numeric value means higher priority. Interrupt requests at higher priority levels will
/// be able to preempt code running at a lower [`RunLevel`][super::RunLevel].
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[non_exhaustive]
pub enum Priority {
    /// Priority level 1.
    Priority1 = 1,
    /// Priority level 2.
    Priority2 = 2,
    /// Priority level 3.
    Priority3 = 3,
    // TODO: Xtensa has 7 priority levels, the higher ones are only not recommended for use.
    // We should add these levels, and a mechanism to bind assembly-written handlers for them.
}

impl Priority {
    /// Maximum interrupt priority
    #[instability::unstable]
    pub const fn max() -> Priority {
        Priority::Priority3
    }

    /// Minimum interrupt priority
    pub const fn min() -> Priority {
        Priority::Priority1
    }

    pub(crate) fn try_from_u32(priority: u32) -> Result<Self, PriorityError> {
        match priority {
            1 => Ok(Priority::Priority1),
            2 => Ok(Priority::Priority2),
            3 => Ok(Priority::Priority3),
            _ => Err(PriorityError::InvalidInterruptPriority),
        }
    }
}

#[instability::unstable]
impl TryFrom<u32> for Priority {
    type Error = PriorityError;

    fn try_from(value: u32) -> Result<Self, Self::Error> {
        Self::try_from_u32(value)
    }
}

#[instability::unstable]
impl TryFrom<u8> for Priority {
    type Error = PriorityError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        Self::try_from(value as u32)
    }
}

pub(super) const DISABLED_CPU_INTERRUPT: u32 = 16;

// CPU interrupt API. These don't take a core, because the control mechanisms are generally
// core-local.

pub(crate) fn enable_cpu_interrupt_raw(cpu_interrupt: u32) {
    unsafe { xtensa_lx::interrupt::enable_mask(1 << cpu_interrupt) };
}

// Runlevel APIs

/// Get the current run level (the level below which interrupts are masked).
pub(crate) fn current_runlevel() -> RunLevel {
    let ps: u32;
    unsafe { core::arch::asm!("rsr.ps {0}", out(reg) ps) };

    unwrap!(RunLevel::try_from_u32(ps & 0x0F))
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
    let token: u32;
    unsafe {
        match level {
            RunLevel::ThreadMode => core::arch::asm!("rsil {0}, 0", out(reg) token),
            RunLevel::Interrupt(Priority::Priority1) => {
                core::arch::asm!("rsil {0}, 1", out(reg) token)
            }
            RunLevel::Interrupt(Priority::Priority2) => {
                core::arch::asm!("rsil {0}, 2", out(reg) token)
            }
            RunLevel::Interrupt(Priority::Priority3) => {
                core::arch::asm!("rsil {0}, 3", out(reg) token)
            }
        };
    }

    unwrap!(RunLevel::try_from_u32(token & 0x0F))
}

/// Wait for an interrupt to occur.
///
/// This function causes the current CPU core to execute its Wait For Interrupt
/// (WFI or equivalent) instruction. After executing this function, the CPU core
/// will stop execution until an interrupt occurs.
#[inline(always)]
#[instability::unstable]
pub fn wait_for_interrupt() {
    unsafe { core::arch::asm!("waiti 0") };
}

pub(crate) fn priority_to_cpu_interrupt(interrupt: Interrupt, level: Priority) -> CpuInterrupt {
    if EDGE_INTERRUPTS.contains(&interrupt) {
        match level {
            Priority::Priority1 => CpuInterrupt::Interrupt10EdgePriority1,
            Priority::Priority2 => {
                warn!("Priority 2 edge interrupts are not supported, using Priority 1 instead");
                CpuInterrupt::Interrupt10EdgePriority1
            }
            Priority::Priority3 => CpuInterrupt::Interrupt22EdgePriority3,
        }
    } else {
        match level {
            Priority::Priority1 => CpuInterrupt::Interrupt1LevelPriority1,
            Priority::Priority2 => CpuInterrupt::Interrupt19LevelPriority2,
            Priority::Priority3 => CpuInterrupt::Interrupt23LevelPriority3,
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        pub(crate) const EDGE_INTERRUPTS: [Interrupt; 8] = [
            Interrupt::TG0_T0_EDGE,
            Interrupt::TG0_T1_EDGE,
            Interrupt::TG0_WDT_EDGE,
            Interrupt::TG0_LACT_EDGE,
            Interrupt::TG1_T0_EDGE,
            Interrupt::TG1_T1_EDGE,
            Interrupt::TG1_WDT_EDGE,
            Interrupt::TG1_LACT_EDGE,
        ];
    } else if #[cfg(esp32s2)] {
        pub(crate) const EDGE_INTERRUPTS: [Interrupt; 11] = [
            Interrupt::TG0_T0_EDGE,
            Interrupt::TG0_T1_EDGE,
            Interrupt::TG0_WDT_EDGE,
            Interrupt::TG0_LACT_EDGE,
            Interrupt::TG1_T0_EDGE,
            Interrupt::TG1_T1_EDGE,
            Interrupt::TG1_WDT_EDGE,
            Interrupt::TG1_LACT_EDGE,
            Interrupt::SYSTIMER_TARGET0,
            Interrupt::SYSTIMER_TARGET1,
            Interrupt::SYSTIMER_TARGET2,
        ];
    } else if #[cfg(esp32s3)] {
        pub(crate) const EDGE_INTERRUPTS: [Interrupt; 0] = [];
    } else {
        compile_error!("Unsupported chip");
    }
}

/// Setup interrupts ready for vectoring
///
/// # Safety
///
/// This function must be called only during core startup.
#[cfg(any(feature = "rt", all(feature = "unstable", multi_core)))]
pub(crate) unsafe fn init_vectoring() {
    // Enable vectored interrupts. No configuration is needed because these interrupts have
    // fixed priority and trigger mode.
    for cpu_int in [
        CpuInterrupt::Interrupt10EdgePriority1,
        CpuInterrupt::Interrupt22EdgePriority3,
        CpuInterrupt::Interrupt1LevelPriority1,
        CpuInterrupt::Interrupt19LevelPriority2,
        CpuInterrupt::Interrupt23LevelPriority3,
    ] {
        cpu_int.enable();
    }
}

#[cfg(feature = "rt")]
pub(crate) mod rt {
    use procmacros::ram;
    use xtensa_lx_rt::{exception::Context, interrupt::CpuInterruptLevel};

    use super::*;
    use crate::{interrupt::InterruptStatus, system::Cpu};

    #[cfg_attr(place_switch_tables_in_ram, ram)]
    pub(crate) static CPU_INTERRUPT_INTERNAL: u32 = 0b_0010_0000_0000_0001_1000_1000_1100_0000;
    #[cfg_attr(place_switch_tables_in_ram, ram)]
    pub(crate) static CPU_INTERRUPT_EDGE: u32 = 0b_0111_0000_0100_0000_0000_1100_1000_0000;

    #[cfg_attr(place_switch_tables_in_ram, ram)]
    pub(crate) static CPU_INTERRUPT_LEVELS: [u32; 8] = [
        0, // Dummy level 0
        CpuInterruptLevel::Level1.mask(),
        CpuInterruptLevel::Level2.mask(),
        CpuInterruptLevel::Level3.mask(),
        CpuInterruptLevel::Level4.mask(),
        CpuInterruptLevel::Level5.mask(),
        CpuInterruptLevel::Level6.mask(),
        CpuInterruptLevel::Level7.mask(),
    ];

    /// A bitmap of edge-triggered peripheral interrupts. See `handle_interrupts` why this is
    /// necessary
    #[cfg_attr(place_switch_tables_in_ram, ram)]
    pub static INTERRUPT_EDGE: InterruptStatus = const {
        let mut masks = [0; crate::interrupt::STATUS_WORDS];

        let mut idx = 0;
        while idx < EDGE_INTERRUPTS.len() {
            let interrupt_idx = EDGE_INTERRUPTS[idx] as usize;
            let word_idx = interrupt_idx / 32;
            masks[word_idx] |= 1 << (interrupt_idx % 32);
            idx += 1;
        }

        InterruptStatus { status: masks }
    };

    #[unsafe(no_mangle)]
    #[ram]
    unsafe fn __level_1_interrupt(save_frame: &mut Context) {
        unsafe {
            handle_interrupts::<1>(save_frame);
        }
    }

    #[unsafe(no_mangle)]
    #[ram]
    unsafe fn __level_2_interrupt(save_frame: &mut Context) {
        unsafe {
            handle_interrupts::<2>(save_frame);
        }
    }

    #[unsafe(no_mangle)]
    #[ram]
    unsafe fn __level_3_interrupt(save_frame: &mut Context) {
        unsafe {
            handle_interrupts::<3>(save_frame);
        }
    }

    #[inline(always)]
    unsafe fn handle_interrupts<const LEVEL: u32>(save_frame: &mut Context) {
        let cpu_interrupt_mask = xtensa_lx::interrupt::get()
            & xtensa_lx::interrupt::get_mask()
            & CPU_INTERRUPT_LEVELS[LEVEL as usize];

        if cpu_interrupt_mask & CPU_INTERRUPT_INTERNAL != 0 {
            // Let's handle CPU-internal interrupts (NMI, Timer, Software, Profiling).
            // These are rarely used by the HAL.

            // Mask the relevant bits
            let cpu_interrupt_mask = cpu_interrupt_mask & CPU_INTERRUPT_INTERNAL;

            // Pick one
            let cpu_interrupt_nr = cpu_interrupt_mask.trailing_zeros();

            // If the interrupt is edge triggered, we need to clear the request on the CPU's
            // side.
            if ((1 << cpu_interrupt_nr) & CPU_INTERRUPT_EDGE) != 0 {
                unsafe {
                    xtensa_lx::interrupt::clear(1 << cpu_interrupt_nr);
                }
            }

            if let Some(handler) = cpu_interrupt_nr_to_cpu_interrupt_handler(cpu_interrupt_nr) {
                unsafe { handler(save_frame) };
            }
        } else {
            let status = if !cfg!(esp32s3) && (cpu_interrupt_mask & CPU_INTERRUPT_EDGE) != 0 {
                // Next, handle edge triggered peripheral interrupts. Note that on the S3 all
                // peripheral interrupts are level-triggered.

                // If the interrupt is edge triggered, we need to clear the
                // request on the CPU's side
                unsafe { xtensa_lx::interrupt::clear(cpu_interrupt_mask & CPU_INTERRUPT_EDGE) };

                // For edge interrupts we cannot rely on the peripherals' interrupt status
                // registers, therefore call all registered handlers for current level.
                INTERRUPT_EDGE
            } else {
                // Finally, check level-triggered peripheral sources.
                // These interrupts are cleared by the peripheral.
                InterruptStatus::current()
            };

            let core = Cpu::current();
            for interrupt_nr in status.iterator().filter(|&interrupt_nr| {
                crate::interrupt::should_handle(core, interrupt_nr as u32, LEVEL)
            }) {
                let handler = unsafe { crate::pac::__INTERRUPTS[interrupt_nr as usize]._handler };
                let handler: fn(&mut Context) = unsafe {
                    core::mem::transmute::<unsafe extern "C" fn(), fn(&mut Context)>(handler)
                };
                handler(save_frame);
            }
        }
    }

    #[inline]
    pub(crate) fn cpu_interrupt_nr_to_cpu_interrupt_handler(
        number: u32,
    ) -> Option<unsafe extern "C" fn(save_frame: &mut Context)> {
        use xtensa_lx_rt::*;
        // we're fortunate that all esp variants use the same CPU interrupt layout
        Some(match number {
            6 => Timer0,
            7 => Software0,
            11 => Profiling,
            14 => NMI,
            15 => Timer1,
            16 => Timer2,
            29 => Software1,
            _ => return None,
        })
    }

    // Raw handlers for CPU interrupts, assembly only.
    unsafe extern "C" {
        fn level4_interrupt(save_frame: &mut Context);
        fn level5_interrupt(save_frame: &mut Context);
        #[cfg(not(all(feature = "rt", feature = "exception-handler", stack_guard_monitoring)))]
        fn level6_interrupt(save_frame: &mut Context);
        fn level7_interrupt(save_frame: &mut Context);
    }

    #[unsafe(no_mangle)]
    #[ram]
    unsafe fn __level_4_interrupt(save_frame: &mut Context) {
        unsafe { level4_interrupt(save_frame) }
    }

    #[unsafe(no_mangle)]
    #[ram]
    unsafe fn __level_5_interrupt(save_frame: &mut Context) {
        unsafe { level5_interrupt(save_frame) }
    }

    #[unsafe(no_mangle)]
    #[ram]
    unsafe fn __level_6_interrupt(save_frame: &mut Context) {
        cfg_if::cfg_if! {
            if #[cfg(all(feature = "rt", feature = "exception-handler", stack_guard_monitoring))] {
                crate::exception_handler::breakpoint_interrupt(save_frame);
            } else {
                unsafe { level6_interrupt(save_frame) }
            }
        }
    }

    #[unsafe(no_mangle)]
    #[ram]
    unsafe fn __level_7_interrupt(save_frame: &mut Context) {
        unsafe { level7_interrupt(save_frame) }
    }
}
