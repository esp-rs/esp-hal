use riscv::register::mcause;

use crate::{pac::Interrupt, Cpu};

// User code shouldn't usually take the mutable TrapFrame or the TrapFrame in
// general. However this makes things like preemtive multitasking easier in
// future
extern "C" {
    fn interrupt1(frame: &mut TrapFrame);
    fn interrupt2(frame: &mut TrapFrame);
    fn interrupt3(frame: &mut TrapFrame);
    fn interrupt4(frame: &mut TrapFrame);
    fn interrupt5(frame: &mut TrapFrame);
    fn interrupt6(frame: &mut TrapFrame);
    fn interrupt7(frame: &mut TrapFrame);
    fn interrupt8(frame: &mut TrapFrame);
    fn interrupt9(frame: &mut TrapFrame);
    fn interrupt10(frame: &mut TrapFrame);
    fn interrupt11(frame: &mut TrapFrame);
    fn interrupt12(frame: &mut TrapFrame);
    fn interrupt13(frame: &mut TrapFrame);
    fn interrupt14(frame: &mut TrapFrame);
    fn interrupt15(frame: &mut TrapFrame);
    fn interrupt16(frame: &mut TrapFrame);
    fn interrupt17(frame: &mut TrapFrame);
    fn interrupt18(frame: &mut TrapFrame);
    fn interrupt19(frame: &mut TrapFrame);
    fn interrupt20(frame: &mut TrapFrame);
    fn interrupt21(frame: &mut TrapFrame);
    fn interrupt22(frame: &mut TrapFrame);
    fn interrupt23(frame: &mut TrapFrame);
    fn interrupt24(frame: &mut TrapFrame);
    fn interrupt25(frame: &mut TrapFrame);
    fn interrupt26(frame: &mut TrapFrame);
    fn interrupt27(frame: &mut TrapFrame);
    fn interrupt28(frame: &mut TrapFrame);
    fn interrupt29(frame: &mut TrapFrame);
    fn interrupt30(frame: &mut TrapFrame);
    fn interrupt31(frame: &mut TrapFrame);
}

/// Interrupt kind
pub enum InterruptKind {
    /// Level interrupt
    Level,
    /// Edge interrupt
    Edge,
}

/// Enumeration of available CPU interrupts.
/// It is possible to create a handler for each of the interrupts. (e.g.
/// `interrupt3`)
pub enum CpuInterrupt {
    Interrupt1 = 1,
    Interrupt2,
    Interrupt3,
    Interrupt4,
    Interrupt5,
    Interrupt6,
    Interrupt7,
    Interrupt8,
    Interrupt9,
    Interrupt10,
    Interrupt11,
    Interrupt12,
    Interrupt13,
    Interrupt14,
    Interrupt15,
    Interrupt16,
    Interrupt17,
    Interrupt18,
    Interrupt19,
    Interrupt20,
    Interrupt21,
    Interrupt22,
    Interrupt23,
    Interrupt24,
    Interrupt25,
    Interrupt26,
    Interrupt27,
    Interrupt28,
    Interrupt29,
    Interrupt30,
    Interrupt31,
}

/// Interrupt priority levels.
pub enum Priority {
    None,
    Priority1,
    Priority2,
    Priority3,
    Priority4,
    Priority5,
    Priority6,
    Priority7,
    Priority8,
    Priority9,
    Priority10,
    Priority11,
    Priority12,
    Priority13,
    Priority14,
    Priority15,
}

/// Enable and assign a peripheral interrupt to an CPU interrupt.
pub fn enable(_core: Cpu, interrupt: Interrupt, which: CpuInterrupt) {
    unsafe {
        let interrupt_number = interrupt as isize;
        let cpu_interrupt_number = which as isize;
        let intr = &*crate::pac::INTERRUPT_CORE0::ptr();
        let intr_map_base = intr.mac_intr_map.as_ptr();
        intr_map_base
            .offset(interrupt_number)
            .write_volatile(cpu_interrupt_number as u32);

        // enable interrupt
        intr.cpu_int_enable
            .write(|w| w.bits(1 << cpu_interrupt_number));
    }
}

/// Disable the given peripheral interrupt.
pub fn disable(_core: Cpu, interrupt: Interrupt) {
    unsafe {
        let interrupt_number = interrupt as isize;
        let intr = &*crate::pac::INTERRUPT_CORE0::ptr();
        let intr_map_base = intr.mac_intr_map.as_ptr();
        intr_map_base.offset(interrupt_number).write_volatile(0);
    }
}

/// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
pub fn set_kind(_core: Cpu, which: CpuInterrupt, kind: InterruptKind) {
    unsafe {
        let intr = &*crate::pac::INTERRUPT_CORE0::ptr();
        let cpu_interrupt_number = which as isize;

        let interrupt_type = match kind {
            InterruptKind::Level => 0,
            InterruptKind::Edge => 1,
        };
        intr.cpu_int_type.modify(|r, w| {
            w.bits(
                r.bits() & !(1 << cpu_interrupt_number) | (interrupt_type << cpu_interrupt_number),
            )
        });
    }
}

/// Set the priority level of an CPU interrupt
pub fn set_priority(_core: Cpu, which: CpuInterrupt, priority: Priority) {
    unsafe {
        let intr = &*crate::pac::INTERRUPT_CORE0::ptr();
        let cpu_interrupt_number = which as isize;
        let intr_prio_base = intr.cpu_int_pri_0.as_ptr();

        intr_prio_base
            .offset(cpu_interrupt_number as isize)
            .write_volatile(priority as u32);
    }
}

/// Clear a CPU interrupt
pub fn clear(_core: Cpu, which: CpuInterrupt) {
    unsafe {
        let cpu_interrupt_number = which as isize;
        let intr = &*crate::pac::INTERRUPT_CORE0::ptr();
        intr.cpu_int_clear
            .write(|w| w.bits(1 << cpu_interrupt_number));
    }
}

/// Get status of peripheral interrupts
pub fn get_status(_core: Cpu) -> u128 {
    unsafe {
        ((*crate::pac::INTERRUPT_CORE0::ptr())
            .intr_status_reg_0
            .read()
            .bits() as u128)
            | ((*crate::pac::INTERRUPT_CORE0::ptr())
                .intr_status_reg_1
                .read()
                .bits() as u128)
                << 32
    }
}

// TODO should this be aligned with Atomic Emulation Trap Handler in future?
/// Registers saved in trap handler
#[doc(hidden)]
#[allow(missing_docs)]
#[derive(Debug, Default, Clone, Copy)]
#[repr(C)]
pub struct TrapFrame {
    pub ra: usize,
    pub t0: usize,
    pub t1: usize,
    pub t2: usize,
    pub t3: usize,
    pub t4: usize,
    pub t5: usize,
    pub t6: usize,
    pub a0: usize,
    pub a1: usize,
    pub a2: usize,
    pub a3: usize,
    pub a4: usize,
    pub a5: usize,
    pub a6: usize,
    pub a7: usize,
    pub s0: usize,
    pub s1: usize,
    pub s2: usize,
    pub s3: usize,
    pub s4: usize,
    pub s5: usize,
    pub s6: usize,
    pub s7: usize,
    pub s8: usize,
    pub s9: usize,
    pub s10: usize,
    pub s11: usize,
    pub gp: usize,
    pub tp: usize,
    pub sp: usize,
}

/// # Safety
///
/// This function is called from an assembly trap handler.
#[doc(hidden)]
#[link_section = ".trap.rust"]
#[export_name = "_start_trap_rust_hal"]
pub unsafe extern "C" fn start_trap_rust_hal(trap_frame: *mut TrapFrame) {
    extern "C" {
        pub fn _start_trap_rust(trap_frame: *const TrapFrame);

        pub fn DefaultHandler();
    }

    let cause = mcause::read();
    if cause.is_exception() {
        _start_trap_rust(trap_frame);
    } else {
        let code = riscv::register::mcause::read().code();
        match code {
            1 => interrupt1(trap_frame.as_mut().unwrap()),
            2 => interrupt2(trap_frame.as_mut().unwrap()),
            3 => interrupt3(trap_frame.as_mut().unwrap()),
            4 => interrupt4(trap_frame.as_mut().unwrap()),
            5 => interrupt5(trap_frame.as_mut().unwrap()),
            6 => interrupt6(trap_frame.as_mut().unwrap()),
            7 => interrupt7(trap_frame.as_mut().unwrap()),
            8 => interrupt8(trap_frame.as_mut().unwrap()),
            9 => interrupt9(trap_frame.as_mut().unwrap()),
            10 => interrupt10(trap_frame.as_mut().unwrap()),
            11 => interrupt11(trap_frame.as_mut().unwrap()),
            12 => interrupt12(trap_frame.as_mut().unwrap()),
            13 => interrupt13(trap_frame.as_mut().unwrap()),
            14 => interrupt14(trap_frame.as_mut().unwrap()),
            16 => interrupt16(trap_frame.as_mut().unwrap()),
            15 => interrupt15(trap_frame.as_mut().unwrap()),
            17 => interrupt17(trap_frame.as_mut().unwrap()),
            18 => interrupt18(trap_frame.as_mut().unwrap()),
            19 => interrupt19(trap_frame.as_mut().unwrap()),
            20 => interrupt20(trap_frame.as_mut().unwrap()),
            21 => interrupt21(trap_frame.as_mut().unwrap()),
            22 => interrupt22(trap_frame.as_mut().unwrap()),
            23 => interrupt23(trap_frame.as_mut().unwrap()),
            24 => interrupt24(trap_frame.as_mut().unwrap()),
            25 => interrupt25(trap_frame.as_mut().unwrap()),
            26 => interrupt26(trap_frame.as_mut().unwrap()),
            27 => interrupt27(trap_frame.as_mut().unwrap()),
            28 => interrupt28(trap_frame.as_mut().unwrap()),
            29 => interrupt29(trap_frame.as_mut().unwrap()),
            30 => interrupt30(trap_frame.as_mut().unwrap()),
            31 => interrupt31(trap_frame.as_mut().unwrap()),
            _ => DefaultHandler(),
        };
    }
}

#[doc(hidden)]
#[no_mangle]
pub fn _setup_interrupts() {
    extern "C" {
        static _vector_table: *const u32;
    }

    unsafe {
        let vec_table = &_vector_table as *const _ as usize;
        riscv::register::mtvec::write(vec_table, riscv::register::mtvec::TrapMode::Vectored);
    };
}
