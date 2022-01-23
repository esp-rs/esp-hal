use crate::{pac::Interrupt, Cpu};

extern "C" {
    fn level1_interrupt();
    fn level2_interrupt();
    fn level3_interrupt();
    fn level4_interrupt();
    fn level5_interrupt();
    fn level6_interrupt();
    fn level7_interrupt();
}

/// Enumeration of available CPU interrupts
/// It's possible to create one handler per priority level. (e.g `level1_interrupt`)
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
    Interrupt16Timer2Priority3,
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
        let intr = &*crate::pac::DPORT::ptr();
        let intr_map_base = match core {
            Cpu::ProCpu => intr.pro_mac_intr_map.as_ptr(),
            #[cfg(feature = "dual_core")]
            Cpu::AppCpu => intr.app_mac_intr_map.as_ptr(),
            #[cfg(feature = "single_core")]
            Cpu::AppCpu => intr.pro_mac_intr_map.as_ptr(),
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
        let intr = &*crate::pac::DPORT::ptr();
        let intr_map_base = match core {
            Cpu::ProCpu => intr.pro_mac_intr_map.as_ptr(),
            #[cfg(feature = "dual_core")]
            Cpu::AppCpu => intr.app_mac_intr_map.as_ptr(),
            #[cfg(feature = "single_core")]
            Cpu::AppCpu => intr.pro_mac_intr_map.as_ptr(),
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
                ((*crate::pac::DPORT::ptr()).pro_intr_status_0.read().bits() as u128)
                    | ((*crate::pac::DPORT::ptr()).pro_intr_status_1.read().bits() as u128) << 32
                    | ((*crate::pac::DPORT::ptr()).pro_intr_status_2.read().bits() as u128) << 64
            }
            Cpu::AppCpu => {
                ((*crate::pac::DPORT::ptr()).app_intr_status_0.read().bits() as u128)
                    | ((*crate::pac::DPORT::ptr()).app_intr_status_1.read().bits() as u128) << 32
                    | ((*crate::pac::DPORT::ptr()).app_intr_status_2.read().bits() as u128) << 64
            }
        }
    }
}

#[xtensa_lx_rt::interrupt(1)]
fn _level1_interrupt() {
    unsafe { level1_interrupt() };
}

#[xtensa_lx_rt::interrupt(2)]
fn _level2_interrupt() {
    unsafe { level2_interrupt() };
}

#[xtensa_lx_rt::interrupt(3)]
fn _level3_interrupt() {
    unsafe { level3_interrupt() };
}

#[xtensa_lx_rt::interrupt(4)]
fn _level4_interrupt() {
    unsafe { level4_interrupt() };
}

#[xtensa_lx_rt::interrupt(5)]
fn _level5_interrupt() {
    unsafe { level5_interrupt() };
}

#[xtensa_lx_rt::interrupt(6)]
fn _level6_interrupt() {
    unsafe { level6_interrupt() };
}

#[xtensa_lx_rt::interrupt(7)]
fn _level7_interrupt() {
    unsafe { level7_interrupt() };
}
