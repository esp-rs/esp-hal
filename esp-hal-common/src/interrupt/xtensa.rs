use xtensa_lx_rt::exception::Context;

use crate::{pac::Interrupt, Cpu};

extern "C" {
    fn level1_interrupt(save_frame: &mut Context);
    fn level2_interrupt(save_frame: &mut Context);
    fn level3_interrupt(save_frame: &mut Context);
    fn level4_interrupt(save_frame: &mut Context);
    fn level5_interrupt(save_frame: &mut Context);
    fn level6_interrupt(save_frame: &mut Context);
    fn level7_interrupt(save_frame: &mut Context);
}

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
            #[cfg(feature = "multicore")]
            Cpu::AppCpu => (*core1_interrupt_peripheral()).app_mac_intr_map.as_ptr(),
            #[cfg(feature = "unicore")]
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
            #[cfg(feature = "multicore")]
            Cpu::AppCpu => (*core1_interrupt_peripheral()).app_mac_intr_map.as_ptr(),
            #[cfg(feature = "unicore")]
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
            #[cfg(feature = "multicore")]
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
            #[cfg(feature = "unicore")]
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

#[no_mangle]
#[link_section = ".rwtext"]
fn __level_1_interrupt(_level: u32, save_frame: &mut Context) {
    unsafe { level1_interrupt(save_frame) };
}

#[no_mangle]
#[link_section = ".rwtext"]
fn __level_2_interrupt(_level: u32, save_frame: &mut Context) {
    unsafe { level2_interrupt(save_frame) };
}

#[no_mangle]
#[link_section = ".rwtext"]
fn __level_3_interrupt(_level: u32, save_frame: &mut Context) {
    unsafe { level3_interrupt(save_frame) };
}

#[no_mangle]
#[link_section = ".rwtext"]
fn __level_4_interrupt(_level: u32, save_frame: &mut Context) {
    unsafe { level4_interrupt(save_frame) };
}

#[no_mangle]
#[link_section = ".rwtext"]
fn __level_5_interrupt(_level: u32, save_frame: &mut Context) {
    unsafe { level5_interrupt(save_frame) };
}

#[no_mangle]
#[link_section = ".rwtext"]
fn __level_6_interrupt(_level: u32, save_frame: &mut Context) {
    unsafe { level6_interrupt(save_frame) };
}

#[no_mangle]
#[link_section = ".rwtext"]
fn __level_7_interrupt(_level: u32, save_frame: &mut Context) {
    unsafe { level7_interrupt(save_frame) };
}
