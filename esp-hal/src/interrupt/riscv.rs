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

pub use esp_riscv_rt::TrapFrame;
use riscv::register::{mcause, mtvec};

#[cfg(not(any(plic, clic)))]
pub use self::classic::*;
#[cfg(clic)]
pub use self::clic::*;
#[cfg(plic)]
pub use self::plic::*;
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
#[repr(u32)]
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
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
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
#[allow(missing_docs)]
pub enum Priority {
    None = 0,
    Priority1,
    Priority2,
    Priority3,
    Priority4,
    Priority5,
    Priority6,
    Priority7,
    #[cfg(not(clic))]
    Priority8,
    #[cfg(not(clic))]
    Priority9,
    #[cfg(not(clic))]
    Priority10,
    #[cfg(not(clic))]
    Priority11,
    #[cfg(not(clic))]
    Priority12,
    #[cfg(not(clic))]
    Priority13,
    #[cfg(not(clic))]
    Priority14,
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
                Priority::Priority7
            }
        }
    }

    /// Minimum interrupt priority
    pub const fn min() -> Priority {
        Priority::Priority1
    }
}

/// The interrupts reserved by the HAL
pub const RESERVED_INTERRUPTS: &[usize] = INTERRUPT_TO_PRIORITY;

/// # Safety
///
/// This function is called from an assembly trap handler.
#[doc(hidden)]
#[link_section = ".trap.rust"]
#[export_name = "_start_trap_rust_hal"]
pub unsafe extern "C" fn start_trap_rust_hal(trap_frame: *mut TrapFrame) {
    assert!(
        mcause::read().is_exception(),
        "Arrived into _start_trap_rust_hal but mcause is not an exception!"
    );
    extern "C" {
        fn ExceptionHandler(tf: *mut TrapFrame);
    }
    ExceptionHandler(trap_frame);
}

#[doc(hidden)]
#[no_mangle]
pub fn _setup_interrupts() {
    extern "C" {
        static _vector_table: *const u32;
    }

    unsafe {
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

        let vec_table = &_vector_table as *const _ as usize;
        mtvec::write(vec_table, mtvec::TrapMode::Vectored);

        crate::interrupt::init_vectoring();
    };

    #[cfg(plic)]
    unsafe {
        core::arch::asm!("csrw mie, {0}", in(reg) u32::MAX);
    }
}

/// Enable an interrupt by directly binding it to a available CPU interrupt
///
/// Unless you are sure, you most likely want to use [`enable`] instead.
///
/// Trying using a reserved interrupt from [`RESERVED_INTERRUPTS`] will return
/// an error.
pub fn enable_direct(
    interrupt: Interrupt,
    level: Priority,
    cpu_interrupt: CpuInterrupt,
) -> Result<(), Error> {
    if RESERVED_INTERRUPTS.contains(&(cpu_interrupt as _)) {
        return Err(Error::CpuInterruptReserved);
    }
    if matches!(level, Priority::None) {
        return Err(Error::InvalidInterruptPriority);
    }
    unsafe {
        map(crate::get_core(), interrupt, cpu_interrupt);
        set_priority(crate::get_core(), cpu_interrupt, level);
        enable_cpu_interrupt(cpu_interrupt);
    }
    Ok(())
}

/// Disable the given peripheral interrupt.
pub fn disable(_core: Cpu, interrupt: Interrupt) {
    unsafe {
        let interrupt_number = interrupt as isize;
        let intr_map_base = crate::soc::registers::INTERRUPT_MAP_BASE as *mut u32;

        // set to 0 to disable the peripheral interrupt on chips with an interrupt
        // controller other than PLIC use the disabled interrupt 31 otherwise
        intr_map_base
            .offset(interrupt_number)
            .write_volatile(DISABLED_CPU_INTERRUPT);
    }
}

/// Get status of peripheral interrupts
#[inline]
pub fn get_status(_core: Cpu) -> InterruptStatus {
    #[cfg(large_intr_status)]
    unsafe {
        InterruptStatus::from(
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_0()
                .read()
                .bits(),
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_1()
                .read()
                .bits(),
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .int_status_reg_2()
                .read()
                .bits(),
        )
    }

    #[cfg(very_large_intr_status)]
    unsafe {
        InterruptStatus::from(
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_0()
                .read()
                .bits(),
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_1()
                .read()
                .bits(),
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_2()
                .read()
                .bits(),
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_3()
                .read()
                .bits(),
        )
    }

    #[cfg(not(any(large_intr_status, very_large_intr_status)))]
    unsafe {
        InterruptStatus::from(
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_0()
                .read()
                .bits(),
            (*crate::peripherals::INTERRUPT_CORE0::PTR)
                .intr_status_reg_1()
                .read()
                .bits(),
        )
    }
}

/// Assign a peripheral interrupt to an CPU interrupt.
///
/// # Safety
///
/// Do not use CPU interrupts in the [`RESERVED_INTERRUPTS`].
pub unsafe fn map(_core: Cpu, interrupt: Interrupt, which: CpuInterrupt) {
    let interrupt_number = interrupt as isize;
    let cpu_interrupt_number = which as isize;
    #[cfg(not(multi_core))]
    let intr_map_base = crate::soc::registers::INTERRUPT_MAP_BASE as *mut u32;
    #[cfg(multi_core)]
    let intr_map_base = match _core {
        Cpu::ProCpu => crate::soc::registers::INTERRUPT_MAP_BASE as *mut u32,
        Cpu::AppCpu => crate::soc::registers::INTERRUPT_MAP_BASE_APP_CPU as *mut u32,
    };

    intr_map_base
        .offset(interrupt_number)
        .write_volatile(cpu_interrupt_number as u32 + EXTERNAL_INTERRUPT_OFFSET);
}

/// Get cpu interrupt assigned to peripheral interrupt
#[inline]
unsafe fn get_assigned_cpu_interrupt(interrupt: Interrupt) -> Option<CpuInterrupt> {
    let interrupt_number = interrupt as isize;
    let intr_map_base = crate::soc::registers::INTERRUPT_MAP_BASE as *mut u32;

    let cpu_intr = intr_map_base.offset(interrupt_number).read_volatile();
    if cpu_intr > 0 {
        Some(core::mem::transmute::<u32, CpuInterrupt>(
            cpu_intr - EXTERNAL_INTERRUPT_OFFSET,
        ))
    } else {
        None
    }
}

mod vectored {
    use procmacros::ram;

    use super::*;

    // Setup interrupts ready for vectoring
    #[doc(hidden)]
    pub(crate) unsafe fn init_vectoring() {
        for (prio, num) in PRIORITY_TO_INTERRUPT.iter().enumerate() {
            set_kind(
                crate::get_core(),
                core::mem::transmute::<u32, CpuInterrupt>(*num as u32),
                InterruptKind::Level,
            );
            set_priority(
                crate::get_core(),
                core::mem::transmute::<u32, CpuInterrupt>(*num as u32),
                core::mem::transmute::<u8, Priority>((prio as u8) + 1),
            );
            enable_cpu_interrupt(core::mem::transmute::<u32, CpuInterrupt>(*num as u32));
        }
    }

    /// Get the interrupts configured for the core at the given priority
    /// matching the given status
    #[inline]
    fn get_configured_interrupts(
        core: Cpu,
        status: InterruptStatus,
        priority: Priority,
    ) -> InterruptStatus {
        unsafe {
            let mut res = InterruptStatus::empty();

            for interrupt_nr in status.iterator() {
                // safety: cast is safe because of repr(u16)
                if let Some(cpu_interrupt) =
                    get_assigned_cpu_interrupt(core::mem::transmute::<u16, Interrupt>(
                        interrupt_nr as u16,
                    ))
                {
                    if get_priority_by_core(core, cpu_interrupt) == priority {
                        res.set(interrupt_nr as u16);
                    }
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
        if matches!(level, Priority::None) {
            return Err(Error::InvalidInterruptPriority);
        }
        unsafe {
            let cpu_interrupt = core::mem::transmute::<u32, CpuInterrupt>(
                PRIORITY_TO_INTERRUPT[(level as usize) - 1] as u32,
            );
            map(crate::get_core(), interrupt, cpu_interrupt);
            enable_cpu_interrupt(cpu_interrupt);
        }
        Ok(())
    }

    /// Bind the given interrupt to the given handler
    ///
    /// # Safety
    ///
    /// This will replace any previously bound interrupt handler
    pub unsafe fn bind_interrupt(interrupt: Interrupt, handler: unsafe extern "C" fn() -> ()) {
        let ptr = &peripherals::__EXTERNAL_INTERRUPTS[interrupt as usize]._handler as *const _
            as *mut unsafe extern "C" fn() -> ();
        ptr.write_volatile(handler);
    }

    #[no_mangle]
    #[ram]
    unsafe fn handle_interrupts(cpu_intr: CpuInterrupt, context: &mut TrapFrame) {
        let core = crate::get_core();
        let status = get_status(core);

        // this has no effect on level interrupts, but the interrupt may be an edge one
        // so we clear it anyway
        clear(core, cpu_intr);
        let prio: Priority =
            unsafe { core::mem::transmute(INTERRUPT_TO_PRIORITY[cpu_intr as usize - 1] as u8) };
        let configured_interrupts = get_configured_interrupts(core, status, prio);

        for interrupt_nr in configured_interrupts.iterator() {
            // Don't use `Interrupt::try_from`. It's slower and placed in flash
            let interrupt: Interrupt = unsafe { core::mem::transmute(interrupt_nr as u16) };
            handle_interrupt(interrupt, context);
        }
    }

    #[inline(always)]
    unsafe fn handle_interrupt(interrupt: Interrupt, save_frame: &mut TrapFrame) {
        extern "C" {
            // defined in each hal
            fn EspDefaultHandler(interrupt: Interrupt);
        }

        let handler = peripherals::__EXTERNAL_INTERRUPTS[interrupt as usize]._handler;

        if core::ptr::eq(
            handler as *const _,
            EspDefaultHandler as *const unsafe extern "C" fn(),
        ) {
            EspDefaultHandler(interrupt);
        } else {
            let handler: fn(&mut TrapFrame) =
                core::mem::transmute::<unsafe extern "C" fn(), fn(&mut TrapFrame)>(handler);
            handler(save_frame);
        }
    }

    // The compiler generates quite unfortunate code for
    // ```rust,ignore
    // #[no_mangle]
    // #[ram]
    // unsafe fn interrupt1(context: &mut TrapFrame) {
    //    handle_interrupts(CpuInterrupt::Interrupt1, context)
    // }
    // ```
    //
    // Resulting in
    // ```asm,ignore
    // interrupt1:
    // add	sp,sp,-16
    // sw	ra,12(sp)
    // sw	s0,8(sp)
    // add	s0,sp,16
    // mv	a1,a0
    // li	a0,1
    // lw	ra,12(sp)
    // lw	s0,8(sp)
    // add	sp,sp,16
    // auipc	t1,0x0
    // jr	handle_interrupts
    // ```
    //
    // We can do better manually - use Rust again once/if that changes
    macro_rules! interrupt_handler {
        ($num:literal) => {
            core::arch::global_asm! {
                concat!(
                r#"
                    .section .rwtext, "ax"
                    .global interrupt"#,$num,r#"

                interrupt"#,$num,r#":
                    mv a1, a0
                    li a0,"#,$num,r#"
                    j handle_interrupts
                "#
            )
            }
        };
    }

    interrupt_handler!(1);
    interrupt_handler!(2);
    interrupt_handler!(3);
    interrupt_handler!(4);
    interrupt_handler!(5);
    interrupt_handler!(6);
    interrupt_handler!(7);
    interrupt_handler!(8);
    interrupt_handler!(9);
    interrupt_handler!(10);
    interrupt_handler!(11);
    interrupt_handler!(12);
    interrupt_handler!(13);
    interrupt_handler!(14);
    interrupt_handler!(15);

    #[cfg(plic)]
    interrupt_handler!(16);
    #[cfg(plic)]
    interrupt_handler!(17);
    #[cfg(plic)]
    interrupt_handler!(18);
    #[cfg(plic)]
    interrupt_handler!(19);
}

#[cfg(not(any(plic, clic)))]
mod classic {
    use super::{CpuInterrupt, InterruptKind, Priority};
    use crate::Cpu;

    pub(super) const DISABLED_CPU_INTERRUPT: u32 = 0;

    pub(super) const EXTERNAL_INTERRUPT_OFFSET: u32 = 0;

    pub(super) const PRIORITY_TO_INTERRUPT: &[usize] =
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

    pub(super) const INTERRUPT_TO_PRIORITY: &[usize] =
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

    /// Enable a CPU interrupt
    ///
    /// # Safety
    ///
    /// Make sure there is an interrupt handler registered.
    pub unsafe fn enable_cpu_interrupt(which: CpuInterrupt) {
        let cpu_interrupt_number = which as isize;
        let intr = &*crate::peripherals::INTERRUPT_CORE0::PTR;
        intr.cpu_int_enable()
            .modify(|r, w| w.bits((1 << cpu_interrupt_number) | r.bits()));
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// The vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    pub fn set_kind(_core: Cpu, which: CpuInterrupt, kind: InterruptKind) {
        unsafe {
            let intr = &*crate::peripherals::INTERRUPT_CORE0::PTR;
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
        let intr = &*crate::peripherals::INTERRUPT_CORE0::PTR;
        let cpu_interrupt_number = which as isize;
        let intr_prio_base = intr.cpu_int_pri(0).as_ptr();

        intr_prio_base
            .offset(cpu_interrupt_number)
            .write_volatile(priority as u32);
    }

    /// Clear a CPU interrupt
    #[inline]
    pub fn clear(_core: Cpu, which: CpuInterrupt) {
        unsafe {
            let cpu_interrupt_number = which as isize;
            let intr = &*crate::peripherals::INTERRUPT_CORE0::PTR;
            intr.cpu_int_clear()
                .write(|w| w.bits(1 << cpu_interrupt_number));
        }
    }

    /// Get interrupt priority
    #[inline]
    pub(super) fn get_priority_by_core(_core: Cpu, cpu_interrupt: CpuInterrupt) -> Priority {
        unsafe { get_priority(cpu_interrupt) }
    }

    /// Get interrupt priority - called by assembly code
    #[inline]
    pub(super) unsafe extern "C" fn get_priority(cpu_interrupt: CpuInterrupt) -> Priority {
        let intr = &*crate::peripherals::INTERRUPT_CORE0::PTR;
        let intr_prio_base = intr.cpu_int_pri(0).as_ptr();

        let prio = intr_prio_base
            .offset(cpu_interrupt as isize)
            .read_volatile();
        core::mem::transmute::<u8, Priority>(prio as u8)
    }
    #[no_mangle]
    #[link_section = ".trap"]
    pub(super) unsafe extern "C" fn _handle_priority() -> u32 {
        use super::mcause;
        let interrupt_id: usize = mcause::read().code(); // MSB is whether its exception or interrupt.
        let intr = &*crate::peripherals::INTERRUPT_CORE0::PTR;
        let interrupt_priority = intr
            .cpu_int_pri(0)
            .as_ptr()
            .add(interrupt_id)
            .read_volatile();

        let prev_interrupt_priority = intr.cpu_int_thresh().read().bits();
        if interrupt_priority < 15 {
            // leave interrupts disabled if interrupt is of max priority.
            intr.cpu_int_thresh()
                .write(|w| w.bits(interrupt_priority + 1)); // set the prio threshold to 1 more than current interrupt prio
            unsafe {
                riscv::interrupt::enable();
            }
        }
        prev_interrupt_priority
    }
    #[no_mangle]
    #[link_section = ".trap"]
    pub(super) unsafe extern "C" fn _restore_priority(stored_prio: u32) {
        riscv::interrupt::disable();
        let intr = &*crate::peripherals::INTERRUPT_CORE0::PTR;
        intr.cpu_int_thresh().write(|w| w.bits(stored_prio));
    }
}

#[cfg(plic)]
mod plic {
    use super::{CpuInterrupt, InterruptKind, Priority};
    use crate::Cpu;

    pub(super) const DISABLED_CPU_INTERRUPT: u32 = 31;

    pub(super) const EXTERNAL_INTERRUPT_OFFSET: u32 = 0;

    // don't use interrupts reserved for CLIC (0,3,4,7)
    // for some reason also CPU interrupt 8 doesn't work by default since it's
    // disabled after reset - so don't use that, too
    pub(super) const PRIORITY_TO_INTERRUPT: &[usize] =
        &[1, 2, 5, 6, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19];

    pub(super) const INTERRUPT_TO_PRIORITY: &[usize] = &[
        1, 2, 0, 0, 3, 4, 0, 0, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    ];

    const DR_REG_PLIC_MX_BASE: u32 = 0x20001000;
    const PLIC_MXINT_ENABLE_REG: u32 = DR_REG_PLIC_MX_BASE;
    const PLIC_MXINT_TYPE_REG: u32 = DR_REG_PLIC_MX_BASE + 0x4;
    const PLIC_MXINT_CLEAR_REG: u32 = DR_REG_PLIC_MX_BASE + 0x8;
    const PLIC_MXINT0_PRI_REG: u32 = DR_REG_PLIC_MX_BASE + 0x10;
    const PLIC_MXINT_THRESH_REG: u32 = DR_REG_PLIC_MX_BASE + 0x90;
    /// Enable a CPU interrupt
    ///
    /// # Safety
    ///
    /// Make sure there is an interrupt handler registered.
    pub unsafe fn enable_cpu_interrupt(which: CpuInterrupt) {
        let cpu_interrupt_number = which as isize;
        let mxint_enable = PLIC_MXINT_ENABLE_REG as *mut u32;
        unsafe {
            mxint_enable.write_volatile(mxint_enable.read_volatile() | 1 << cpu_interrupt_number);
        }
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// The vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    pub fn set_kind(_core: Cpu, which: CpuInterrupt, kind: InterruptKind) {
        unsafe {
            let intr = PLIC_MXINT_TYPE_REG as *mut u32;
            let cpu_interrupt_number = which as isize;

            let interrupt_type = match kind {
                InterruptKind::Level => 0,
                InterruptKind::Edge => 1,
            };
            intr.write_volatile(
                intr.read_volatile() & !(1 << cpu_interrupt_number)
                    | (interrupt_type << cpu_interrupt_number),
            );
        }
    }

    /// Set the priority level of an CPU interrupt
    ///
    /// # Safety
    ///
    /// Great care must be taken when using this function; avoid changing the
    /// priority of interrupts 1 - 15.
    pub unsafe fn set_priority(_core: Cpu, which: CpuInterrupt, priority: Priority) {
        let plic_mxint_pri_ptr = PLIC_MXINT0_PRI_REG as *mut u32;

        let cpu_interrupt_number = which as isize;
        plic_mxint_pri_ptr
            .offset(cpu_interrupt_number)
            .write_volatile(priority as u32);
    }

    /// Clear a CPU interrupt
    #[inline]
    pub fn clear(_core: Cpu, which: CpuInterrupt) {
        unsafe {
            let cpu_interrupt_number = which as isize;
            let intr = PLIC_MXINT_CLEAR_REG as *mut u32;
            intr.write_volatile(1 << cpu_interrupt_number);
        }
    }

    /// Get interrupt priority
    #[inline]
    pub(super) unsafe extern "C" fn get_priority_by_core(
        _core: Cpu,
        cpu_interrupt: CpuInterrupt,
    ) -> Priority {
        unsafe { get_priority(cpu_interrupt) }
    }

    /// Get interrupt priority - called by assembly code
    #[inline]
    pub(super) unsafe extern "C" fn get_priority(cpu_interrupt: CpuInterrupt) -> Priority {
        let plic_mxint_pri_ptr = PLIC_MXINT0_PRI_REG as *mut u32;

        let cpu_interrupt_number = cpu_interrupt as isize;
        let prio = plic_mxint_pri_ptr
            .offset(cpu_interrupt_number)
            .read_volatile();
        core::mem::transmute::<u8, Priority>(prio as u8)
    }
    #[no_mangle]
    #[link_section = ".trap"]
    pub(super) unsafe extern "C" fn _handle_priority() -> u32 {
        use super::mcause;
        let plic_mxint_pri_ptr = PLIC_MXINT0_PRI_REG as *mut u32;
        let interrupt_id: isize = unwrap!(mcause::read().code().try_into()); // MSB is whether its exception or interrupt.
        let interrupt_priority = plic_mxint_pri_ptr.offset(interrupt_id).read_volatile();

        let thresh_reg = PLIC_MXINT_THRESH_REG as *mut u32;
        let prev_interrupt_priority = thresh_reg.read_volatile() & 0x000000FF;
        // this is a u8 according to esp-idf, so mask everything else.
        if interrupt_priority < 15 {
            // leave interrupts disabled if interrupt is of max priority.
            thresh_reg.write_volatile(interrupt_priority + 1);
            unsafe {
                riscv::interrupt::enable();
            }
        }
        prev_interrupt_priority
    }
    #[no_mangle]
    #[link_section = ".trap"]
    pub(super) unsafe extern "C" fn _restore_priority(stored_prio: u32) {
        riscv::interrupt::disable();
        let thresh_reg = PLIC_MXINT_THRESH_REG as *mut u32;
        thresh_reg.write_volatile(stored_prio);
    }
}

#[cfg(clic)]
mod clic {
    use super::{CpuInterrupt, InterruptKind, Priority};
    use crate::Cpu;

    pub(super) const DISABLED_CPU_INTERRUPT: u32 = 0;

    pub(super) const EXTERNAL_INTERRUPT_OFFSET: u32 = 16;

    pub(super) const PRIORITY_TO_INTERRUPT: &[usize] =
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

    pub(super) const INTERRUPT_TO_PRIORITY: &[usize] =
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

    // The memory map for interrupt registers is on a per-core basis,
    // base points to the current core interrupt register,
    // whereas base + DUALCORE_CLIC_CTRL_OFF points to the other
    // core registers, regardless of the core we are currently running on.

    const DR_REG_CLIC_CTRL_BASE: u32 = 0x20801000;
    const DUALCORE_CLIC_CTRL_OFF: u32 = 0x10000;

    const CLIC_EXT_INTR_NUM_OFFSET: usize = 16;

    bitfield::bitfield! {
        #[derive(Clone, Copy, Default)]
        pub struct InterruptControl(u32);

        bool,pending, set_pending: 0;
        bool,enabled, set_enabled: 8;
        bool,vectored, set_vectored: 16;
        u8,trigger, set_trigger: 18, 17;
        u8,mode, _: 23, 22;
        u8,priority, set_priority: 31, 29;
    }

    /// Get pointer to interrupt control register for the given core and CPU
    /// interrupt number
    fn intr_cntrl(core: Cpu, cpu_interrupt_number: usize) -> *mut u32 {
        let offset = if core == crate::get_core() {
            0
        } else {
            DUALCORE_CLIC_CTRL_OFF
        };
        unsafe {
            ((DR_REG_CLIC_CTRL_BASE + offset) as *mut u32)
                .add(CLIC_EXT_INTR_NUM_OFFSET + cpu_interrupt_number)
        }
    }

    /// Enable a CPU interrupt on current core
    ///
    /// # Safety
    ///
    /// Make sure there is an interrupt handler registered.
    pub unsafe fn enable_cpu_interrupt(which: CpuInterrupt) {
        let cpu_interrupt_number = which as usize;
        let intr_cntrl = intr_cntrl(crate::get_core(), cpu_interrupt_number);
        unsafe {
            let mut val = InterruptControl(intr_cntrl.read_volatile());
            val.set_enabled(true);
            intr_cntrl.write_volatile(val.0);
        }
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// The vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    pub fn set_kind(core: Cpu, which: CpuInterrupt, kind: InterruptKind) {
        let cpu_interrupt_number = which as usize;
        unsafe {
            let intr_cntrl = intr_cntrl(core, cpu_interrupt_number);
            let mut val = InterruptControl(intr_cntrl.read_volatile());
            val.set_trigger(match kind {
                InterruptKind::Level => 0b00,
                InterruptKind::Edge => 0b10,
            });
            intr_cntrl.write_volatile(val.0);
        }
    }

    /// Set the priority level of an CPU interrupt
    ///
    /// # Safety
    ///
    /// Great care must be taken when using this function; avoid changing the
    /// priority of interrupts 1 - 15.
    pub unsafe fn set_priority(core: Cpu, which: CpuInterrupt, priority: Priority) {
        let cpu_interrupt_number = which as usize;
        let intr_cntrl = intr_cntrl(core, cpu_interrupt_number);
        unsafe {
            let mut val = InterruptControl(intr_cntrl.read_volatile());
            val.set_priority(priority as u8);
            intr_cntrl.write_volatile(val.0);
        }
    }

    /// Clear a CPU interrupt
    #[inline]
    pub fn clear(core: Cpu, which: CpuInterrupt) {
        let cpu_interrupt_number = which as usize;
        unsafe {
            let intr_cntrl = intr_cntrl(core, cpu_interrupt_number);
            let mut val = InterruptControl(intr_cntrl.read_volatile());
            val.set_pending(false);
            intr_cntrl.write_volatile(val.0);
        }
    }

    /// Get interrupt priority
    #[inline]
    pub(super) fn get_priority_by_core(core: Cpu, cpu_interrupt: CpuInterrupt) -> Priority {
        let cpu_interrupt_number = cpu_interrupt as usize;
        unsafe {
            let intr_cntrl = intr_cntrl(core, cpu_interrupt_number);
            let val = InterruptControl(intr_cntrl.read_volatile());
            core::mem::transmute::<u8, Priority>(val.priority())
        }
    }
}
