//! Interrupt handling

use xtensa_lx::interrupt;
use xtensa_lx_rt::exception::Context;

pub use self::vectored::*;
use super::InterruptStatus;
use crate::{
    peripherals::{self, Interrupt},
    Cpu,
};

/// Interrupt Error
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// The given interrupt is not a valid interrupt
    InvalidInterrupt,
    /// The CPU interrupt is a reserved interrupt
    CpuInterruptReserved,
}

/// Enumeration of available CPU interrupts
///
/// It's possible to create one handler per priority level. (e.g
/// `level1_interrupt`)
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u32)]
pub enum CpuInterrupt {
    /// Level-triggered interrupt with priority 1.
    Interrupt0LevelPriority1 = 0,
    /// Level-triggered interrupt with priority 1.
    Interrupt1LevelPriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt2LevelPriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt3LevelPriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt4LevelPriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt5LevelPriority1,
    /// Timer 0 interrupt with priority 1.
    Interrupt6Timer0Priority1,
    /// Software-triggered interrupt with priority 1.
    Interrupt7SoftwarePriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt8LevelPriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt9LevelPriority1,
    /// Edge-triggered interrupt with priority 1.
    Interrupt10EdgePriority1,
    /// Profiling-related interrupt with priority 3.
    Interrupt11ProfilingPriority3,
    /// Level-triggered interrupt with priority 1.
    Interrupt12LevelPriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt13LevelPriority1,
    /// Non-maskable interrupt (NMI) with priority 7.
    Interrupt14NmiPriority7,
    /// Timer 1 interrupt with priority 3.
    Interrupt15Timer1Priority3,
    /// Timer 2 interrupt with priority 5.
    Interrupt16Timer2Priority5,
    /// Level-triggered interrupt with priority 1.
    Interrupt17LevelPriority1,
    /// Level-triggered interrupt with priority 1.
    Interrupt18LevelPriority1,
    /// Level-triggered interrupt with priority 2.
    Interrupt19LevelPriority2,
    /// Level-triggered interrupt with priority 2.
    Interrupt20LevelPriority2,
    /// Level-triggered interrupt with priority 2.
    Interrupt21LevelPriority2,
    /// Edge-triggered interrupt with priority 3.
    Interrupt22EdgePriority3,
    /// Level-triggered interrupt with priority 3.
    Interrupt23LevelPriority3,
    /// Level-triggered interrupt with priority 4.
    Interrupt24LevelPriority4,
    /// Level-triggered interrupt with priority 4.
    Interrupt25LevelPriority4,
    /// Level-triggered interrupt with priority 5.
    Interrupt26LevelPriority5,
    /// Level-triggered interrupt with priority 3.
    Interrupt27LevelPriority3,
    /// Edge-triggered interrupt with priority 4.
    Interrupt28EdgePriority4,
    /// Software-triggered interrupt with priority 3.
    Interrupt29SoftwarePriority3,
    /// Edge-triggered interrupt with priority 4.
    Interrupt30EdgePriority4,
    /// Edge-triggered interrupt with priority 5.
    Interrupt31EdgePriority5,
}

/// The interrupts reserved by the HAL
pub const RESERVED_INTERRUPTS: &[usize] = &[
    CpuInterrupt::Interrupt1LevelPriority1 as _,
    CpuInterrupt::Interrupt19LevelPriority2 as _,
    CpuInterrupt::Interrupt23LevelPriority3 as _,
    CpuInterrupt::Interrupt10EdgePriority1 as _,
    CpuInterrupt::Interrupt22EdgePriority3 as _,
];

pub(crate) fn setup_interrupts() {
    // disable all known interrupts
    // at least after the 2nd stage bootloader there are some interrupts enabled
    // (e.g. UART)
    for peripheral_interrupt in 0..255 {
        crate::soc::peripherals::Interrupt::try_from(peripheral_interrupt)
            .map(|intr| {
                #[cfg(multi_core)]
                disable(Cpu::AppCpu, intr);
                disable(Cpu::ProCpu, intr);
            })
            .ok();
    }
}

/// Enable an interrupt by directly binding it to a available CPU interrupt
///
/// Unless you are sure, you most likely want to use [`enable`] instead.
///
/// Trying using a reserved interrupt from [`RESERVED_INTERRUPTS`] will return
/// an error.
pub fn enable_direct(interrupt: Interrupt, cpu_interrupt: CpuInterrupt) -> Result<(), Error> {
    if RESERVED_INTERRUPTS.contains(&(cpu_interrupt as _)) {
        return Err(Error::CpuInterruptReserved);
    }
    unsafe {
        map(crate::get_core(), interrupt, cpu_interrupt);

        xtensa_lx::interrupt::enable_mask(
            xtensa_lx::interrupt::get_mask() | 1 << cpu_interrupt as u32,
        );
    }
    Ok(())
}

/// Assign a peripheral interrupt to an CPU interrupt
///
/// Note: this only maps the interrupt to the CPU interrupt. The CPU interrupt
/// still needs to be enabled afterwards
///
/// # Safety
///
/// Do not use CPU interrupts in the [`RESERVED_INTERRUPTS`].
pub unsafe fn map(core: Cpu, interrupt: Interrupt, which: CpuInterrupt) {
    let interrupt_number = interrupt as isize;
    let cpu_interrupt_number = which as isize;
    let intr_map_base = match core {
        Cpu::ProCpu => (*core0_interrupt_peripheral()).pro_mac_intr_map().as_ptr(),
        #[cfg(multi_core)]
        Cpu::AppCpu => (*core1_interrupt_peripheral()).app_mac_intr_map().as_ptr(),
    };
    intr_map_base
        .offset(interrupt_number)
        .write_volatile(cpu_interrupt_number as u32);
}

/// Disable the given peripheral interrupt
pub fn disable(core: Cpu, interrupt: Interrupt) {
    unsafe {
        let interrupt_number = interrupt as isize;
        let intr_map_base = match core {
            Cpu::ProCpu => (*core0_interrupt_peripheral()).pro_mac_intr_map().as_ptr(),
            #[cfg(multi_core)]
            Cpu::AppCpu => (*core1_interrupt_peripheral()).app_mac_intr_map().as_ptr(),
        };
        // To disable an interrupt, map it to a CPU peripheral interrupt
        intr_map_base
            .offset(interrupt_number)
            .write_volatile(CpuInterrupt::Interrupt16Timer2Priority5 as _);
    }
}

/// Clear the given CPU interrupt
pub fn clear(_core: Cpu, which: CpuInterrupt) {
    unsafe {
        xtensa_lx::interrupt::clear(1 << which as u32);
    }
}

/// Get status of peripheral interrupts
#[cfg(large_intr_status)]
pub fn get_status(core: Cpu) -> InterruptStatus {
    unsafe {
        match core {
            Cpu::ProCpu => InterruptStatus::from(
                (*core0_interrupt_peripheral())
                    .pro_intr_status_0()
                    .read()
                    .bits(),
                (*core0_interrupt_peripheral())
                    .pro_intr_status_1()
                    .read()
                    .bits(),
                (*core0_interrupt_peripheral())
                    .pro_intr_status_2()
                    .read()
                    .bits(),
            ),
            #[cfg(multi_core)]
            Cpu::AppCpu => InterruptStatus::from(
                (*core1_interrupt_peripheral())
                    .app_intr_status_0()
                    .read()
                    .bits(),
                (*core1_interrupt_peripheral())
                    .app_intr_status_1()
                    .read()
                    .bits(),
                (*core1_interrupt_peripheral())
                    .app_intr_status_2()
                    .read()
                    .bits(),
            ),
        }
    }
}

/// Get status of peripheral interrupts
#[cfg(very_large_intr_status)]
pub fn get_status(core: Cpu) -> InterruptStatus {
    unsafe {
        match core {
            Cpu::ProCpu => InterruptStatus::from(
                (*core0_interrupt_peripheral())
                    .pro_intr_status_0()
                    .read()
                    .bits(),
                (*core0_interrupt_peripheral())
                    .pro_intr_status_1()
                    .read()
                    .bits(),
                (*core0_interrupt_peripheral())
                    .pro_intr_status_2()
                    .read()
                    .bits(),
                (*core0_interrupt_peripheral())
                    .pro_intr_status_3()
                    .read()
                    .bits(),
            ),
            #[cfg(multi_core)]
            Cpu::AppCpu => InterruptStatus::from(
                (*core1_interrupt_peripheral())
                    .app_intr_status_0()
                    .read()
                    .bits(),
                (*core1_interrupt_peripheral())
                    .app_intr_status_1()
                    .read()
                    .bits(),
                (*core1_interrupt_peripheral())
                    .app_intr_status_2()
                    .read()
                    .bits(),
                (*core1_interrupt_peripheral())
                    .app_intr_status_3()
                    .read()
                    .bits(),
            ),
        }
    }
}

#[cfg(esp32)]
unsafe fn core0_interrupt_peripheral() -> *const crate::peripherals::dport::RegisterBlock {
    crate::peripherals::DPORT::PTR
}

#[cfg(esp32)]
unsafe fn core1_interrupt_peripheral() -> *const crate::peripherals::dport::RegisterBlock {
    crate::peripherals::DPORT::PTR
}

#[cfg(any(esp32s2, esp32s3))]
unsafe fn core0_interrupt_peripheral() -> *const crate::peripherals::interrupt_core0::RegisterBlock
{
    crate::peripherals::INTERRUPT_CORE0::PTR
}

#[cfg(esp32s3)]
unsafe fn core1_interrupt_peripheral() -> *const crate::peripherals::interrupt_core1::RegisterBlock
{
    crate::peripherals::INTERRUPT_CORE1::PTR
}

mod vectored {
    use procmacros::ram;

    use super::*;
    use crate::get_core;

    /// Interrupt priority levels.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
    }

    impl Priority {
        /// Maximum interrupt priority
        pub const fn max() -> Priority {
            Priority::Priority3
        }

        /// Minimum interrupt priority
        pub const fn min() -> Priority {
            Priority::Priority1
        }
    }

    impl CpuInterrupt {
        #[inline]
        fn level(&self) -> Priority {
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

                // we direct these to None because we do not support interrupts at this level
                // through Rust
                CpuInterrupt::Interrupt24LevelPriority4
                | CpuInterrupt::Interrupt25LevelPriority4
                | CpuInterrupt::Interrupt28EdgePriority4
                | CpuInterrupt::Interrupt30EdgePriority4
                | CpuInterrupt::Interrupt31EdgePriority5
                | CpuInterrupt::Interrupt16Timer2Priority5
                | CpuInterrupt::Interrupt26LevelPriority5
                | CpuInterrupt::Interrupt14NmiPriority7 => Priority::None,
            }
        }
    }

    /// Get the interrupts configured for the core
    #[inline(always)]
    fn get_configured_interrupts(
        core: Cpu,
        status: InterruptStatus,
        level: u32,
    ) -> InterruptStatus {
        unsafe {
            let intr_map_base = match core {
                Cpu::ProCpu => (*core0_interrupt_peripheral()).pro_mac_intr_map().as_ptr(),
                #[cfg(multi_core)]
                Cpu::AppCpu => (*core1_interrupt_peripheral()).app_mac_intr_map().as_ptr(),
            };

            let mut res = InterruptStatus::empty();

            for interrupt_nr in status.iterator() {
                let i = interrupt_nr as isize;
                let cpu_interrupt = intr_map_base.offset(i).read_volatile();
                // safety: cast is safe because of repr(u32)
                let cpu_interrupt: CpuInterrupt =
                    core::mem::transmute::<u32, CpuInterrupt>(cpu_interrupt);
                let int_level = cpu_interrupt.level() as u8 as u32;

                if int_level == level {
                    res.set(interrupt_nr as u16);
                }
            }

            res
        }
    }

    /// Enable the given peripheral interrupt
    pub fn enable(interrupt: Interrupt, level: Priority) -> Result<(), Error> {
        let cpu_interrupt =
            interrupt_level_to_cpu_interrupt(level, chip_specific::interrupt_is_edge(interrupt))?;

        unsafe {
            map(get_core(), interrupt, cpu_interrupt);

            xtensa_lx::interrupt::enable_mask(
                xtensa_lx::interrupt::get_mask() | 1 << cpu_interrupt as u32,
            );
        }
        Ok(())
    }

    /// Binds the given interrupt to the given handler.
    ///
    /// # Safety
    ///
    /// This will replace any previously bound interrupt handler
    pub unsafe fn bind_interrupt(interrupt: Interrupt, handler: unsafe extern "C" fn()) {
        let ptr = &peripherals::__INTERRUPTS[interrupt as usize]._handler as *const _
            as *mut unsafe extern "C" fn();
        ptr.write_volatile(handler);
    }

    /// Returns the currently bound interrupt handler.
    pub fn bound_handler(interrupt: Interrupt) -> Option<unsafe extern "C" fn()> {
        unsafe {
            let addr = peripherals::__INTERRUPTS[interrupt as usize]._handler;
            if addr as usize == 0 {
                return None;
            }

            Some(addr)
        }
    }

    fn interrupt_level_to_cpu_interrupt(
        level: Priority,
        is_edge: bool,
    ) -> Result<CpuInterrupt, Error> {
        Ok(if is_edge {
            match level {
                Priority::None => return Err(Error::InvalidInterrupt),
                Priority::Priority1 => CpuInterrupt::Interrupt10EdgePriority1,
                Priority::Priority2 => return Err(Error::InvalidInterrupt),
                Priority::Priority3 => CpuInterrupt::Interrupt22EdgePriority3,
            }
        } else {
            match level {
                Priority::None => return Err(Error::InvalidInterrupt),
                Priority::Priority1 => CpuInterrupt::Interrupt1LevelPriority1,
                Priority::Priority2 => CpuInterrupt::Interrupt19LevelPriority2,
                Priority::Priority3 => CpuInterrupt::Interrupt23LevelPriority3,
            }
        })
    }

    // TODO use CpuInterrupt::LevelX.mask() // TODO make it const
    const CPU_INTERRUPT_LEVELS: [u32; 8] = [
        0b_0000_0000_0000_0000_0000_0000_0000_0000, // Dummy level 0
        0b_0000_0000_0000_0110_0011_0111_1111_1111, // Level_1
        0b_0000_0000_0011_1000_0000_0000_0000_0000, // Level 2
        0b_0010_1000_1100_0000_1000_1000_0000_0000, // Level 3
        0b_0101_0011_0000_0000_0000_0000_0000_0000, // Level 4
        0b_1000_0100_0000_0001_0000_0000_0000_0000, // Level 5
        0b_0000_0000_0000_0000_0000_0000_0000_0000, // Level 6
        0b_0000_0000_0000_0000_0100_0000_0000_0000, // Level 7
    ];
    const CPU_INTERRUPT_INTERNAL: u32 = 0b_0010_0000_0000_0001_1000_1000_1100_0000;
    const CPU_INTERRUPT_EDGE: u32 = 0b_0111_0000_0100_0000_0000_1100_1000_0000;

    #[inline]
    fn cpu_interrupt_nr_to_cpu_interrupt_handler(
        number: u32,
    ) -> Option<unsafe extern "C" fn(u32, save_frame: &mut Context)> {
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

    // The linker script defines `__level_1_interrupt`, `__level_2_interrupt` and
    // `__level_3_interrupt` as `handle_interrupts`

    #[no_mangle]
    #[ram]
    unsafe fn handle_interrupts(level: u32, save_frame: &mut Context) {
        let core = crate::get_core();

        let cpu_interrupt_mask =
            interrupt::get() & interrupt::get_mask() & CPU_INTERRUPT_LEVELS[level as usize];

        if cpu_interrupt_mask & CPU_INTERRUPT_INTERNAL != 0 {
            let cpu_interrupt_mask = cpu_interrupt_mask & CPU_INTERRUPT_INTERNAL;
            let cpu_interrupt_nr = cpu_interrupt_mask.trailing_zeros();

            if (cpu_interrupt_mask & CPU_INTERRUPT_EDGE) != 0 {
                interrupt::clear(1 << cpu_interrupt_nr);
            }
            if let Some(handler) = cpu_interrupt_nr_to_cpu_interrupt_handler(cpu_interrupt_nr) {
                handler(level, save_frame);
            }
        } else if (cpu_interrupt_mask & CPU_INTERRUPT_EDGE) != 0 {
            let cpu_interrupt_mask = cpu_interrupt_mask & CPU_INTERRUPT_EDGE;
            let cpu_interrupt_nr = cpu_interrupt_mask.trailing_zeros();
            interrupt::clear(1 << cpu_interrupt_nr);

            // for edge interrupts cannot rely on the interrupt status
            // register, therefore call all registered
            // handlers for current level
            let configured_interrupts =
                get_configured_interrupts(core, chip_specific::INTERRUPT_EDGE, level);

            for interrupt_nr in configured_interrupts.iterator() {
                // Don't use `Interrupt::try_from`. It's slower and placed in flash
                let interrupt: Interrupt = unsafe { core::mem::transmute(interrupt_nr as u16) };
                handle_interrupt(level, interrupt, save_frame);
            }
        } else {
            // finally check peripheral sources and fire of handlers from pac
            // peripheral mapped interrupts are cleared by the peripheral
            let status = get_status(core);
            let configured_interrupts = get_configured_interrupts(core, status, level);

            for interrupt_nr in configured_interrupts.iterator() {
                // Don't use `Interrupt::try_from`. It's slower and placed in flash
                let interrupt: Interrupt = unsafe { core::mem::transmute(interrupt_nr as u16) };
                handle_interrupt(level, interrupt, save_frame);
            }
        }
    }

    #[ram]
    unsafe fn handle_interrupt(level: u32, interrupt: Interrupt, save_frame: &mut Context) {
        extern "C" {
            // defined in each hal
            fn EspDefaultHandler(level: u32, interrupt: Interrupt);
        }

        let handler = peripherals::__INTERRUPTS[interrupt as usize]._handler;
        if core::ptr::eq(
            handler as *const _,
            EspDefaultHandler as *const unsafe extern "C" fn(),
        ) {
            EspDefaultHandler(level, interrupt);
        } else {
            let handler: fn(&mut Context) =
                core::mem::transmute::<unsafe extern "C" fn(), fn(&mut Context)>(handler);
            handler(save_frame);
        }
    }

    #[allow(clippy::unusual_byte_groupings)]
    #[cfg(esp32)]
    mod chip_specific {
        use super::*;
        pub const INTERRUPT_EDGE: InterruptStatus = InterruptStatus::from(
            0b0000_0000_0000_0000_0000_0000_0000_0000,
            0b1111_1100_0000_0000_0000_0000_0000_0000,
            0b0000_0000_0000_0000_0000_0000_0000_0011,
        );
        #[inline]
        pub fn interrupt_is_edge(interrupt: Interrupt) -> bool {
            use peripherals::Interrupt::*;
            [
                TG0_T0_EDGE,
                TG0_T1_EDGE,
                TG0_WDT_EDGE,
                TG0_LACT_EDGE,
                TG1_T0_EDGE,
                TG1_T1_EDGE,
                TG1_WDT_EDGE,
                TG1_LACT_EDGE,
            ]
            .contains(&interrupt)
        }
    }

    #[cfg(esp32s2)]
    mod chip_specific {
        use super::*;
        pub const INTERRUPT_EDGE: InterruptStatus = InterruptStatus::from(
            0b0000_0000_0000_0000_0000_0000_0000_0000,
            0b1100_0000_0000_0000_0000_0000_0000_0000,
            0b0000_0000_0000_0000_0000_0011_1011_1111,
        );
        #[inline]
        pub fn interrupt_is_edge(interrupt: Interrupt) -> bool {
            use peripherals::Interrupt::*;
            [
                TG0_T0_EDGE,
                TG0_T1_EDGE,
                TG0_WDT_EDGE,
                TG0_LACT_EDGE,
                TG1_T0_EDGE,
                TG1_T1_EDGE,
                TG1_WDT_EDGE,
                TG1_LACT_EDGE,
                SYSTIMER_TARGET0,
                SYSTIMER_TARGET1,
                SYSTIMER_TARGET2,
            ]
            .contains(&interrupt)
        }
    }

    #[cfg(esp32s3)]
    mod chip_specific {
        use super::*;
        pub const INTERRUPT_EDGE: InterruptStatus = InterruptStatus::empty();
        #[inline]
        pub fn interrupt_is_edge(_interrupt: Interrupt) -> bool {
            false
        }
    }
}

mod raw {
    use super::*;

    extern "C" {
        fn level4_interrupt(save_frame: &mut Context);
        fn level5_interrupt(save_frame: &mut Context);
        fn level6_interrupt(save_frame: &mut Context);
        fn level7_interrupt(save_frame: &mut Context);
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_4_interrupt(_level: u32, save_frame: &mut Context) {
        level4_interrupt(save_frame)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_5_interrupt(_level: u32, save_frame: &mut Context) {
        level5_interrupt(save_frame)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_6_interrupt(_level: u32, save_frame: &mut Context) {
        level6_interrupt(save_frame)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_7_interrupt(_level: u32, save_frame: &mut Context) {
        level7_interrupt(save_frame)
    }
}
