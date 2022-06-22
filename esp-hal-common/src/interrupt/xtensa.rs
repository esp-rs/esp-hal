use xtensa_lx::interrupt::InterruptNumber;
use xtensa_lx_rt::exception::Context;

use crate::{
    pac::{self, Interrupt},
    Cpu,
};

/// Enumeration of available CPU interrupts
/// It's possible to create one handler per priority level. (e.g
/// `level1_interrupt`)
#[allow(unused)]
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
mod vectored {
    use super::*;

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

    // /// Stores what interrupts are enabled at each interrupt level
    // static mut INTERRUPT_LEVELS: [u128; 8] = [0u128; 8];

    unsafe fn handle_interrupts(level: u32) {
        // TODO check if its a CPU interrupt
        // TODO check if its an edge interrupt
        // CPU & EDGE interrupts were declared inside the pac on the old esp32
        // crates, but I think we should define them with extern "C"
        // inside this crate, e.g
        //
        // extern "C" {
        //     fn Timer0();
        // }

        // finally check periperal sources and fire of handlers from pac
        let interrupt_mask = get_status(crate::get_core()); //  & INTERRUPT_LEVELS[level as usize] // TODO priority
        let interrupt_nr = interrupt_mask.trailing_zeros();

        // Interrupt::try_from can fail if interrupt already de-asserted:
        // silently ignore
        if let Ok(interrupt) = pac::Interrupt::try_from(interrupt_nr as u16) {
            handle_interrupt(level, interrupt);
        }
        // peripheral mapped interrupts are cleared by the peripheral
    }

    unsafe fn handle_interrupt(level: u32, interrupt: Interrupt) {
        extern "C" {
            // defined in xtensa_lx_rt
            fn DefaultHandler(level: u32, interrupt: Interrupt);
        }

        let handler = pac::__INTERRUPTS[interrupt.number() as usize]._handler;
        if handler as *const _ == DefaultHandler as *const unsafe extern "C" fn() {
            DefaultHandler(level, interrupt);
        } else {
            handler();
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
