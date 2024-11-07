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

#[cfg(not(plic))]
pub use self::classic::*;
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
pub enum CpuInterrupt {
    /// Interrupt number 1.
    Interrupt1 = 1,
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
}

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
    Priority9,
    /// Priority level 10.
    Priority10,
    /// Priority level 11.
    Priority11,
    /// Priority level 12.
    Priority12,
    /// Priority level 13.
    Priority13,
    /// Priority level 14.
    Priority14,
    /// Priority level 15.
    Priority15,
}

impl Priority {
    /// Maximum interrupt priority
    pub const fn max() -> Priority {
        Priority::Priority15
    }

    /// Minimum interrupt priority
    pub const fn min() -> Priority {
        Priority::Priority1
    }
}

/// The interrupts reserved by the HAL
#[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
pub static RESERVED_INTERRUPTS: &[usize] = INTERRUPT_TO_PRIORITY;

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

    /// Binds the given interrupt to the given handler.
    ///
    /// # Safety
    ///
    /// This will replace any previously bound interrupt handler
    pub unsafe fn bind_interrupt(interrupt: Interrupt, handler: unsafe extern "C" fn()) {
        let ptr = &peripherals::__EXTERNAL_INTERRUPTS[interrupt as usize]._handler as *const _
            as *mut unsafe extern "C" fn();
        ptr.write_volatile(handler);
    }

    /// Returns the currently bound interrupt handler.
    pub fn bound_handler(interrupt: Interrupt) -> Option<unsafe extern "C" fn()> {
        unsafe {
            let addr = peripherals::__EXTERNAL_INTERRUPTS[interrupt as usize]._handler;
            if addr as usize == 0 {
                return None;
            }

            Some(addr)
        }
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

#[cfg(not(plic))]
mod classic {
    use super::{CpuInterrupt, InterruptKind, Priority};
    use crate::Cpu;

    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static DISABLED_CPU_INTERRUPT: u32 = 0;

    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static EXTERNAL_INTERRUPT_OFFSET: u32 = 0;

    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static PRIORITY_TO_INTERRUPT: &[usize] =
        &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static INTERRUPT_TO_PRIORITY: &[usize] =
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
        intr.cpu_int_pri(which as usize)
            .write(|w| w.map().bits(priority as u8));
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
        core::mem::transmute::<u8, Priority>(
            intr.cpu_int_pri(cpu_interrupt as usize).read().map().bits(),
        )
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

    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static DISABLED_CPU_INTERRUPT: u32 = 31;

    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static EXTERNAL_INTERRUPT_OFFSET: u32 = 0;

    // don't use interrupts reserved for CLIC (0,3,4,7)
    // for some reason also CPU interrupt 8 doesn't work by default since it's
    // disabled after reset - so don't use that, too
    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static PRIORITY_TO_INTERRUPT: &[usize] =
        &[1, 2, 5, 6, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19];

    #[cfg_attr(place_switch_tables_in_ram, link_section = ".rwtext")]
    pub(super) static INTERRUPT_TO_PRIORITY: &[usize] = &[
        1, 2, 0, 0, 3, 4, 0, 0, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
    ];

    /// Enable a CPU interrupt
    ///
    /// # Safety
    ///
    /// Make sure there is an interrupt handler registered.
    pub unsafe fn enable_cpu_interrupt(which: CpuInterrupt) {
        unsafe {
            let plic = &*crate::peripherals::PLIC_MX::PTR;
            plic.mxint_enable().modify(|r, w| {
                let old = r.cpu_mxint_enable().bits();
                let new = old | 1 << (which as isize);
                w.cpu_mxint_enable().bits(new)
            });
        }
    }

    /// Set the interrupt kind (i.e. level or edge) of an CPU interrupt
    ///
    /// The vectored interrupt handler will take care of clearing edge interrupt
    /// bits.
    pub fn set_kind(_core: Cpu, which: CpuInterrupt, kind: InterruptKind) {
        unsafe {
            let plic = &*crate::peripherals::PLIC_MX::PTR;
            let interrupt_type = match kind {
                InterruptKind::Level => 0,
                InterruptKind::Edge => 1,
            };

            plic.mxint_type().modify(|r, w| {
                let old = r.cpu_mxint_type().bits();
                let new = old & !(1 << (which as isize)) | (interrupt_type << (which as isize));
                w.cpu_mxint_type().bits(new)
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
        unsafe {
            let plic = &*crate::peripherals::PLIC_MX::PTR;
            plic.mxint_pri(which as usize)
                .modify(|_, w| w.cpu_mxint_pri().bits(priority as u8));
        }
    }

    /// Clear a CPU interrupt
    #[inline]
    pub fn clear(_core: Cpu, which: CpuInterrupt) {
        unsafe {
            let plic = &*crate::peripherals::PLIC_MX::PTR;
            plic.mxint_clear().modify(|r, w| {
                let old = r.cpu_mxint_clear().bits();
                let new = old | (1 << (which as isize));
                w.cpu_mxint_clear().bits(new)
            });
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
        let plic = &*crate::peripherals::PLIC_MX::PTR;
        let prio = plic
            .mxint_pri(cpu_interrupt as usize)
            .read()
            .cpu_mxint_pri()
            .bits();
        core::mem::transmute::<u8, Priority>(prio)
    }
    #[no_mangle]
    #[link_section = ".trap"]
    pub(super) unsafe extern "C" fn _handle_priority() -> u32 {
        use super::mcause;
        let plic = &*crate::peripherals::PLIC_MX::PTR;

        let interrupt_id: usize = mcause::read().code(); // MSB is whether its exception or interrupt.
        let interrupt_priority = plic.mxint_pri(interrupt_id).read().cpu_mxint_pri().bits();

        let prev_interrupt_priority = plic.mxint_thresh().read().cpu_mxint_thresh().bits();
        if interrupt_priority < 15 {
            // leave interrupts disabled if interrupt is of max priority.
            plic.mxint_thresh()
                .write(|w| w.cpu_mxint_thresh().bits(interrupt_priority + 1));
            unsafe {
                riscv::interrupt::enable();
            }
        }
        prev_interrupt_priority as u32
    }
    #[no_mangle]
    #[link_section = ".trap"]
    pub(super) unsafe extern "C" fn _restore_priority(stored_prio: u32) {
        riscv::interrupt::disable();
        let plic = &*crate::peripherals::PLIC_MX::PTR;
        plic.mxint_thresh()
            .write(|w| w.cpu_mxint_thresh().bits(stored_prio as u8));
    }
}
