use xtensa_lx::interrupt::{self, InterruptNumber};
use xtensa_lx_rt::exception::Context;

use crate::{
    pac::{self, Interrupt},
    Cpu,
};

/// Enumeration of available CPU interrupts
/// It's possible to create one handler per priority level. (e.g
/// `level1_interrupt`)
#[allow(unused)]
#[derive(Debug, Copy, Clone)]
pub enum CpuInterrupt {
    Interrupt0LevelPriority1 = 0,
    Interrupt1LevelPriority1,
    Interrupt2LevelPriority1,
    Interrupt3LevelPriority1,
    Interrupt4LevelPriority1,
    Interrupt5LevelPriority1,
    Interrupt6Timer0Priority1,
    Interrupt7SoftwarePriority1,
    Interrupt8LevelPriority1,
    Interrupt9LevelPriority1,
    Interrupt10EdgePriority1,
    Interrupt11ProfilingPriority3,
    Interrupt12LevelPriority1,
    Interrupt13LevelPriority1,
    Interrupt14NmiPriority7,
    Interrupt15Timer1Priority3,
    Interrupt16Timer2Priority5,
    Interrupt17LevelPriority1,
    Interrupt18LevelPriority1,
    Interrupt19LevelPriority2,
    Interrupt20LevelPriority2,
    Interrupt21LevelPriority2,
    Interrupt22EdgePriority3,
    Interrupt23LevelPriority3,
    Interrupt24LevelPriority4,
    Interrupt25LevelPriority4,
    Interrupt26LevelPriority5,
    Interrupt27LevelPriority3,
    Interrupt28EdgePriority4,
    Interrupt29SoftwarePriority3,
    Interrupt30EdgePriority4,
    Interrupt31EdgePriority5,
}

/// Enable and assign a peripheral interrupt to an CPU interrupt.
pub fn enable(core: Cpu, interrupt: Interrupt, which: CpuInterrupt) {
    unsafe {
        let interrupt_number = interrupt as isize;
        let cpu_interrupt_number = which as isize;
        let intr_map_base = match core {
            Cpu::ProCpu => (*core0_interrupt_peripheral()).pro_mac_intr_map.as_ptr(),
            #[cfg(feature = "dual_core")]
            Cpu::AppCpu => (*core1_interrupt_peripheral()).app_mac_intr_map.as_ptr(),
            #[cfg(feature = "single_core")]
            Cpu::AppCpu => (*core0_interrupt_peripheral()).pro_mac_intr_map.as_ptr(),
        };
        intr_map_base
            .offset(interrupt_number)
            .write_volatile(cpu_interrupt_number as u32);
    }
}

/// Disable the given peripheral interrupt.
pub fn disable(core: Cpu, interrupt: Interrupt) {
    unsafe {
        let interrupt_number = interrupt as isize;
        let intr_map_base = match core {
            Cpu::ProCpu => (*core0_interrupt_peripheral()).pro_mac_intr_map.as_ptr(),
            #[cfg(feature = "dual_core")]
            Cpu::AppCpu => (*core1_interrupt_peripheral()).app_mac_intr_map.as_ptr(),
            #[cfg(feature = "single_core")]
            Cpu::AppCpu => (*core0_interrupt_peripheral()).pro_mac_intr_map.as_ptr(),
        };
        intr_map_base.offset(interrupt_number).write_volatile(0);
    }
}

/// Clear the given CPU interrupt
pub fn clear(_core: Cpu, which: CpuInterrupt) {
    unsafe {
        xtensa_lx::interrupt::clear(1 << which as u32);
    }
}

/// Get status of peripheral interrupts
pub fn get_status(core: Cpu) -> u128 {
    unsafe {
        match core {
            Cpu::ProCpu => {
                ((*core0_interrupt_peripheral())
                    .pro_intr_status_0
                    .read()
                    .bits() as u128)
                    | ((*core0_interrupt_peripheral())
                        .pro_intr_status_1
                        .read()
                        .bits() as u128)
                        << 32
                    | ((*core0_interrupt_peripheral())
                        .pro_intr_status_2
                        .read()
                        .bits() as u128)
                        << 64
            }
            #[cfg(feature = "dual_core")]
            Cpu::AppCpu => {
                ((*core1_interrupt_peripheral())
                    .app_intr_status_0
                    .read()
                    .bits() as u128)
                    | ((*core1_interrupt_peripheral())
                        .app_intr_status_1
                        .read()
                        .bits() as u128)
                        << 32
                    | ((*core1_interrupt_peripheral())
                        .app_intr_status_2
                        .read()
                        .bits() as u128)
                        << 64
            }
            #[cfg(feature = "single_core")]
            Cpu::AppCpu => {
                ((*core0_interrupt_peripheral())
                    .pro_intr_status_0
                    .read()
                    .bits() as u128)
                    | ((*core0_interrupt_peripheral())
                        .pro_intr_status_1
                        .read()
                        .bits() as u128)
                        << 32
                    | ((*core0_interrupt_peripheral())
                        .pro_intr_status_2
                        .read()
                        .bits() as u128)
                        << 64
            }
        }
    }
}

#[cfg(feature = "esp32")]
unsafe fn core0_interrupt_peripheral() -> *const crate::pac::dport::RegisterBlock {
    crate::pac::DPORT::PTR
}

#[cfg(feature = "esp32")]
unsafe fn core1_interrupt_peripheral() -> *const crate::pac::dport::RegisterBlock {
    crate::pac::DPORT::PTR
}

#[cfg(feature = "esp32s2")]
unsafe fn core0_interrupt_peripheral() -> *const crate::pac::interrupt::RegisterBlock {
    crate::pac::INTERRUPT::PTR
}

#[cfg(feature = "esp32s3")]
unsafe fn core0_interrupt_peripheral() -> *const crate::pac::interrupt_core0::RegisterBlock {
    crate::pac::INTERRUPT_CORE0::PTR
}

#[cfg(feature = "esp32s3")]
unsafe fn core1_interrupt_peripheral() -> *const crate::pac::interrupt_core1::RegisterBlock {
    crate::pac::INTERRUPT_CORE1::PTR
}

#[cfg(feature = "vectored")]
pub mod vectored {
    use super::*;

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum Error {
        InvalidInterrupt,
    }

    /// Interrupt priority levels.
    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum Priority {
        None = 0,
        Priority1,
        Priority2,
        Priority3,
        // TODO the following priorities are 'unusable' due to the value in PS_INTLEVEL_EXCM
        // defined in xtensa_lx_rt Xtensa programmer guide indicates we _can_ implement
        // high level interrupts in Rust, but they recommend we dont (however their reasoning
        // seemed to indicate interference with rtos')
        //
        // Another suggestion from Xtensa programmers guide:
        // Alternatively, a level-one interrupt can also be used instead (with software
        // prioritization if needed) but I have no idea what software prioritization would
        // look like in this context.

        // Priority4,
        // Priority5,
        // Priority6,
    }

    /// Stores what interrupts are enabled at each interrupt level
    static mut INTERRUPT_LEVELS: [u128; 8] = [0u128; 8];

    /// A priority of None, disables the interrupt
    pub fn enable_with_priority(
        core: crate::Cpu,
        interrupt: Interrupt,
        level: Priority,
    ) -> Result<(), Error> {
        let cpu_interrupt =
            interrupt_level_to_cpu_interrupt(level, chip_specific::interrupt_is_edge(interrupt))?;

        // (&INTERRUPT_LEVELS_MUTEX).lock(|_| unsafe {
        // TODO: CS here
        unsafe {
            for i in 0..=7 {
                INTERRUPT_LEVELS[i] &= !(1 << interrupt.number());
            }
            INTERRUPT_LEVELS[level as usize] |= 1 << interrupt.number();

            enable(core, interrupt, cpu_interrupt);

            xtensa_lx::interrupt::enable_mask(
                xtensa_lx::interrupt::get_mask() | 1 << cpu_interrupt as u32,
            );
        }
        Ok(())
        // })
    }

    // TODO mention that theses interrupts are reservered in vector mode
    // TODO make the normal `enable` unsafe? Would break vectoring unless careful
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
                // Priority::Priority4 => CpuInterrupt::Interrupt28EdgePriority4,
                // Priority::Priority5 => CpuInterrupt::Interrupt31EdgePriority5,
                // Priority::Priority6 => return Err(Error::InvalidInterrupt),
                // Priority::Priority7 => CpuInterrupt::Interrupt14NmiPriority7,
            }
        } else {
            match level {
                Priority::None => return Err(Error::InvalidInterrupt),
                Priority::Priority1 => CpuInterrupt::Interrupt1LevelPriority1,
                Priority::Priority2 => CpuInterrupt::Interrupt19LevelPriority2,
                Priority::Priority3 => CpuInterrupt::Interrupt23LevelPriority3,
                // Priority::Priority4 => CpuInterrupt::Interrupt24LevelPriority4,
                // Priority::Priority5 => CpuInterrupt::Interrupt26LevelPriority5,
                // Priority::Priority6 => return Err(Error::InvalidInterrupt),
                // Priority::Priority7 => CpuInterrupt::Interrupt14NmiPriority7,
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

    fn cpu_interrupt_nr_to_cpu_interrupt_handler(_number: u32) -> Option<unsafe extern "C" fn()> {
        // TODO this needs be in a xtensa_lx with a default impl! Like cortex-m crates
        // extern "C" {
        //     fn Timer0();
        //     fn Timer1();
        //     fn Timer2();

        //     fn Profiling();

        //     fn SoftwareLevel1();
        //     fn SoftwareLevel3();

        //     fn NMI();
        // }

        // match number {
        //     6 => Timer0,
        //     7 => SoftwareLevel1,
        //     11 => Profiling,
        //     14 => NMI,
        //     15 => Timer1,
        //     16 => Timer2,
        //     29 => SoftwareLevel3,
        //     _ => return None;
        // }

        unsafe extern "C" fn nop() {}

        Some(nop)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_1_interrupt(level: u32, _save_frame: &mut Context) {
        handle_interrupts(level)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_2_interrupt(level: u32, _save_frame: &mut Context) {
        handle_interrupts(level)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_3_interrupt(level: u32, _save_frame: &mut Context) {
        handle_interrupts(level)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_4_interrupt(level: u32, _save_frame: &mut Context) {
        handle_interrupts(level)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_5_interrupt(level: u32, _save_frame: &mut Context) {
        handle_interrupts(level)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_6_interrupt(level: u32, _save_frame: &mut Context) {
        handle_interrupts(level)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_7_interrupt(level: u32, _save_frame: &mut Context) {
        handle_interrupts(level)
    }

    unsafe fn handle_interrupts(level: u32) {
        let cpu_interrupt_mask =
            interrupt::get() & interrupt::get_mask() & CPU_INTERRUPT_LEVELS[level as usize];

        if cpu_interrupt_mask & CPU_INTERRUPT_INTERNAL != 0 {
            let cpu_interrupt_mask = cpu_interrupt_mask & CPU_INTERRUPT_INTERNAL;
            let cpu_interrupt_nr = cpu_interrupt_mask.trailing_zeros();

            if (cpu_interrupt_mask & CPU_INTERRUPT_EDGE) != 0 {
                interrupt::clear(1 << cpu_interrupt_nr);
            }
            if let Some(handler) = cpu_interrupt_nr_to_cpu_interrupt_handler(cpu_interrupt_nr) {
                handler();
            }
        } else {
            if (cpu_interrupt_mask & CPU_INTERRUPT_EDGE) != 0 {
                let cpu_interrupt_mask = cpu_interrupt_mask & CPU_INTERRUPT_EDGE;
                let cpu_interrupt_nr = cpu_interrupt_mask.trailing_zeros();
                interrupt::clear(1 << cpu_interrupt_nr);

                // for edge interrupts cannot rely on the interrupt status
                // register, therefore call all registered
                // handlers for current level
                let mut interrupt_mask =
                    INTERRUPT_LEVELS[level as usize] & chip_specific::INTERRUPT_EDGE;
                loop {
                    let interrupt_nr = interrupt_mask.trailing_zeros();
                    if let Ok(interrupt) = pac::Interrupt::try_from(interrupt_nr as u16) {
                        handle_interrupt(level, interrupt)
                    } else {
                        break;
                    }
                    interrupt_mask &= !(1u128 << interrupt_nr);
                }
            } else {
                // finally check periperal sources and fire of handlers from pac
                // peripheral mapped interrupts are cleared by the peripheral
                let interrupt_mask =
                    get_status(crate::get_core()) & INTERRUPT_LEVELS[level as usize];
                let interrupt_nr = interrupt_mask.trailing_zeros();

                // Interrupt::try_from can fail if interrupt already de-asserted:
                // silently ignore
                if let Ok(interrupt) = pac::Interrupt::try_from(interrupt_nr as u16) {
                    handle_interrupt(level, interrupt);
                }
            }
        }
    }

    unsafe fn handle_interrupt(level: u32, interrupt: Interrupt) {
        extern "C" {
            // defined in each hal
            fn DefaultHandler(level: u32, interrupt: Interrupt);
        }

        let handler = pac::__INTERRUPTS[interrupt.number() as usize]._handler;
        if handler as *const _ == DefaultHandler as *const unsafe extern "C" fn() {
            DefaultHandler(level, interrupt);
        } else {
            handler();
        }
    }

    #[cfg(feature = "esp32")]
    mod chip_specific {
        use super::*;
        pub const INTERRUPT_EDGE: u128 =
    0b_0000_0000_0000_0000_0000_0000_0000_0000__0000_0000_0000_0000_0000_0000_0000_0011_1111_1100_0000_0000_0000_0000_0000_0000__0000_0000_0000_0000_0000_0000_0000_0000;
        pub fn interrupt_is_edge(interrupt: Interrupt) -> bool {
            use pac::Interrupt::*;
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
}

#[cfg(not(feature = "vectored"))]
mod raw {
    use super::*;

    extern "C" {
        fn level1_interrupt(save_frame: &mut Context);
        fn level2_interrupt(save_frame: &mut Context);
        fn level3_interrupt(save_frame: &mut Context);
        fn level4_interrupt(save_frame: &mut Context);
        fn level5_interrupt(save_frame: &mut Context);
        fn level6_interrupt(save_frame: &mut Context);
        fn level7_interrupt(save_frame: &mut Context);
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_1_interrupt(_level: u32, save_frame: &mut Context) {
        level1_interrupt(save_frame)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_2_interrupt(_level: u32, save_frame: &mut Context) {
        level2_interrupt(save_frame)
    }

    #[no_mangle]
    #[link_section = ".rwtext"]
    unsafe fn __level_3_interrupt(_level: u32, save_frame: &mut Context) {
        level3_interrupt(save_frame)
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
