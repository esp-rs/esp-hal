#[cfg(feature = "esp-radio")]
use core::ffi::c_void;

use esp_hal::{interrupt::software::SoftwareInterrupt, riscv::register, system::Cpu};
use portable_atomic::Ordering;

use crate::SCHEDULER;

unsafe extern "C" {
    fn sys_switch();
}

static _CURRENT_CTX_PTR: portable_atomic::AtomicPtr<CpuContext> =
    portable_atomic::AtomicPtr::new(core::ptr::null_mut());

static _NEXT_CTX_PTR: portable_atomic::AtomicPtr<CpuContext> =
    portable_atomic::AtomicPtr::new(core::ptr::null_mut());

/// Registers saved / restored
#[derive(Debug, Default, Clone)]
#[repr(C)]
pub struct CpuContext {
    /// Return address, stores the address to return to after a function call or
    /// interrupt.
    pub ra: usize,
    /// Temporary register t0, used for intermediate values.
    pub t0: usize,
    /// Temporary register t1, used for intermediate values.
    pub t1: usize,
    /// Temporary register t2, used for intermediate values.
    pub t2: usize,
    /// Temporary register t3, used for intermediate values.
    pub t3: usize,
    /// Temporary register t4, used for intermediate values.
    pub t4: usize,
    /// Temporary register t5, used for intermediate values.
    pub t5: usize,
    /// Temporary register t6, used for intermediate values.
    pub t6: usize,
    /// Argument register a0, typically used to pass the first argument to a
    /// function.
    pub a0: usize,
    /// Argument register a1, typically used to pass the second argument to a
    /// function.
    pub a1: usize,
    /// Argument register a2, typically used to pass the third argument to a
    /// function.
    pub a2: usize,
    /// Argument register a3, typically used to pass the fourth argument to a
    /// function.
    pub a3: usize,
    /// Argument register a4, typically used to pass the fifth argument to a
    /// function.
    pub a4: usize,
    /// Argument register a5, typically used to pass the sixth argument to a
    /// function.
    pub a5: usize,
    /// Argument register a6, typically used to pass the seventh argument to a
    /// function.
    pub a6: usize,
    /// Argument register a7, typically used to pass the eighth argument to a
    /// function.
    pub a7: usize,
    /// Saved register s0, used to hold values across function calls.
    pub s0: usize,
    /// Saved register s1, used to hold values across function calls.
    pub s1: usize,
    /// Saved register s2, used to hold values across function calls.
    pub s2: usize,
    /// Saved register s3, used to hold values across function calls.
    pub s3: usize,
    /// Saved register s4, used to hold values across function calls.
    pub s4: usize,
    /// Saved register s5, used to hold values across function calls.
    pub s5: usize,
    /// Saved register s6, used to hold values across function calls.
    pub s6: usize,
    /// Saved register s7, used to hold values across function calls.
    pub s7: usize,
    /// Saved register s8, used to hold values across function calls.
    pub s8: usize,
    /// Saved register s9, used to hold values across function calls.
    pub s9: usize,
    /// Saved register s10, used to hold values across function calls.
    pub s10: usize,
    /// Saved register s11, used to hold values across function calls.
    pub s11: usize,
    /// Global pointer register, holds the address of the global data area.
    pub gp: usize,
    /// Thread pointer register, holds the address of the thread-local storage
    /// area.
    pub tp: usize,
    /// Stack pointer register, holds the address of the top of the stack.
    pub sp: usize,
    /// Program counter, stores the address of the next instruction to be
    /// executed.
    pub pc: usize,

    /// The mstatus which will be loaded before MRET
    pub mstatus: usize,
}

impl CpuContext {
    /// Creates a new, zeroed out context.
    pub const fn new() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

pub(crate) extern "C" fn idle_hook() -> ! {
    loop {
        unsafe { core::arch::asm!("wfi") };
    }
}

pub(crate) fn set_idle_hook_entry(idle_context: &mut CpuContext, hook_fn: super::IdleFn) {
    // Point idle context PC at the assembly that calls the idle hook. We need a new stack
    // frame for the idle task on the main stack.
    idle_context.pc = hook_fn as usize;
}

#[cfg(feature = "esp-radio")]
pub(crate) fn new_task_context(
    task: extern "C" fn(*mut c_void),
    param: *mut c_void,
    stack_top: *mut (),
) -> CpuContext {
    let stack_top = stack_top as usize;
    let stack_top = stack_top - (stack_top % 16);

    CpuContext {
        pc: super::task_wrapper as usize,
        a0: task as usize,
        a1: param as usize,
        sp: stack_top,
        ..Default::default()
    }
}

/// Switch to next task
///
/// *MUST* be called from inside an ISR without interrupt nesting.
///
/// The task switching relies on MEPC and MSTATUS to not get clobbered.
/// We save MEPC as the current task's PC and change MEPC to an assembly function
/// which will save the current CPU state for the current task (excluding PC) and
/// restoring the CPU state from the next task.
pub fn task_switch(old_ctx: *mut CpuContext, new_ctx: *mut CpuContext) {
    debug_assert!(_NEXT_CTX_PTR.load(Ordering::SeqCst).is_null());
    _CURRENT_CTX_PTR.store(old_ctx, Ordering::SeqCst);
    _NEXT_CTX_PTR.store(new_ctx, Ordering::SeqCst);

    if !old_ctx.is_null() {
        unsafe {
            (*old_ctx).pc = register::mepc::read();
        }
    }

    // set MSTATUS for the switched to task
    // MIE will be set from MPIE
    // MPP will be used to determine the privilege-level
    let mstatus = register::mstatus::read().bits();
    unsafe {
        (*new_ctx).mstatus = mstatus;
    }

    unsafe {
        // set MPIE in MSTATUS to 0 to disable interrupts while task switching
        register::mstatus::write(register::mstatus::Mstatus::from_bits(mstatus & !(1 << 7)));

        // load address of sys_switch into MEPC - will run after all registers are restored
        register::mepc::write(sys_switch as usize);
    }
}

core::arch::global_asm!(
    r#"
.section .trap, "ax"

.globl sys_switch
.align 4
sys_switch:
    # put some regs on the stack since we will need those regs
    addi sp, sp, -16
    sw t0, 0(sp)
    sw t1, 4(sp)

    # t0 => current context
    la t0, {_CURRENT_CTX_PTR}
    lw t0, 0(t0)

    # skip storing context if current task is null (deleted)
    beqz t0, _restore_context

    # store registers to old context - PC needs to be set by the "caller"
    sw ra, 0*4(t0)

    lw t1, 0(sp)
    sw t1, 1*4(t0)

    lw t1, 4(sp)
    sw t1, 2*4(t0)

    sw t2, 3*4(t0)
    sw t3, 4*4(t0)
    sw t4, 5*4(t0)
    sw t5, 6*4(t0)
    sw t6, 7*4(t0)
    sw a0, 8*4(t0)
    sw a1, 9*4(t0)
    sw a2, 10*4(t0)
    sw a3, 11*4(t0)
    sw a4, 12*4(t0)
    sw a5, 13*4(t0)
    sw a6, 14*4(t0)
    sw a7, 15*4(t0)
    sw s0, 16*4(t0)
    sw s1, 17*4(t0)
    sw s2, 18*4(t0)
    sw s3, 19*4(t0)
    sw s4, 20*4(t0)
    sw s5, 21*4(t0)
    sw s6, 22*4(t0)
    sw s7, 23*4(t0)
    sw s8, 24*4(t0)
    sw s9, 25*4(t0)
    sw s10, 26*4(t0)
    sw s11, 27*4(t0)
    sw gp, 28*4(t0)
    sw tp, 29*4(t0)

    addi t1, sp, 16
    sw t1, 30*4(t0)

_restore_context:
    # t0 => next context
    la t1, {_NEXT_CTX_PTR}
    lw t0, 0(t1)

    # signal that the task switch is done - safe to do it already now - interrupts are disabled
    sw x0, 0(t1)

    # set the next task's PC as MEPC
    lw t1, 31*4(t0)
    csrrw x0, mepc, t1

    # set MSTATUS from next context
    lw t1, 32*4(t0)
    csrrw x0, mstatus, t1

    # restore registers from old context
    lw ra, 0*4(t0)
    lw t1, 2*4(t0)
    lw t2, 3*4(t0)
    lw t3, 4*4(t0)
    lw t4, 5*4(t0)
    lw t5, 6*4(t0)
    lw t6, 7*4(t0)
    lw a0, 8*4(t0)
    lw a1, 9*4(t0)
    lw a2, 10*4(t0)
    lw a3, 11*4(t0)
    lw a4, 12*4(t0)
    lw a5, 13*4(t0)
    lw a6, 14*4(t0)
    lw a7, 15*4(t0)
    lw s0, 16*4(t0)
    lw s1, 17*4(t0)
    lw s2, 18*4(t0)
    lw s3, 19*4(t0)
    lw s4, 20*4(t0)
    lw s5, 21*4(t0)
    lw s6, 22*4(t0)
    lw s7, 23*4(t0)
    lw s8, 24*4(t0)
    lw s9, 25*4(t0)
    lw s10, 26*4(t0)
    lw s11, 27*4(t0)
    lw gp, 28*4(t0)
    lw tp, 29*4(t0)
    lw sp, 30*4(t0)
    lw t0, 1*4(t0)


    # jump to next task's PC
    mret

    "#,
    _CURRENT_CTX_PTR = sym _CURRENT_CTX_PTR,
    _NEXT_CTX_PTR = sym _NEXT_CTX_PTR,
);

pub(crate) fn setup_multitasking<const IRQ: u8>(mut irq: SoftwareInterrupt<'static, IRQ>) {
    // Register the interrupt handler without nesting to satisfy the requirements of the task
    // switching code
    let swint_handler = esp_hal::interrupt::InterruptHandler::new_not_nested(
        unsafe { core::mem::transmute::<*const (), extern "C" fn()>(swint_handler as *const ()) },
        esp_hal::interrupt::Priority::Priority1,
    );

    irq.set_interrupt_handler(swint_handler);
}

#[cfg(multi_core)]
pub(crate) fn setup_smp<const IRQ: u8>(irq: SoftwareInterrupt<'static, IRQ>) {
    setup_multitasking(irq);
}

#[esp_hal::ram]
extern "C" fn swint_handler() {
    match Cpu::current() {
        Cpu::ProCpu => unsafe { SoftwareInterrupt::<'static, 0>::steal() }.reset(),
        #[cfg(multi_core)]
        Cpu::AppCpu => unsafe { SoftwareInterrupt::<'static, 1>::steal() }.reset(),
    }

    SCHEDULER.with(|scheduler| scheduler.switch_task());
}

#[inline]
pub(crate) fn yield_task() {
    match Cpu::current() {
        Cpu::ProCpu => unsafe { SoftwareInterrupt::<'static, 0>::steal() }.raise(),
        #[cfg(multi_core)]
        Cpu::AppCpu => unsafe { SoftwareInterrupt::<'static, 1>::steal() }.raise(),
    }

    // It takes a bit for the software interrupt to be serviced.
    esp_hal::riscv::asm::nop();
    esp_hal::riscv::asm::nop();
    esp_hal::riscv::asm::nop();
    esp_hal::riscv::asm::nop();
}
