#[cfg(feature = "esp-radio")]
use core::ffi::c_void;

use esp_hal::{
    interrupt::{self, software::SoftwareInterrupt},
    system::Cpu,
};

#[cfg(feature = "rtos-trace")]
use crate::TraceEvents;
use crate::{
    SCHEDULER,
    task::{IdleFn, Task},
};

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
}

impl CpuContext {
    /// Creates a new, zeroed out context.
    pub const fn new() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

pub(crate) fn set_idle_hook_entry(idle_context: &mut CpuContext, hook_fn: IdleFn) {
    // Point idle context PC at the assembly that calls the idle hook. We need a new stack
    // frame for the idle task on the main stack.
    idle_context.pc = hook_fn as usize;

    // The idle context has no thread-local data
    idle_context.tp = 0;
}

#[inline(always)]
pub(crate) fn read_thread_pointer() -> *mut Task {
    let tp: *mut Task;
    unsafe { core::arch::asm!("c.mv {0}, tp", out(reg) tp, options(nostack)) };
    tp
}

#[inline(always)]
pub(crate) fn write_thread_pointer(task: *mut Task) {
    unsafe { core::arch::asm!("c.mv tp, {0}", in(reg) task, options(nostack)) };
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
        pc: super::task_wrapper as *const () as usize,
        a0: task as usize,
        a1: param as usize,
        sp: stack_top,
        ..Default::default()
    }
}

/// Select the new task
///
/// Task switching happens when exiting the interrupt handler. The handler detects that `tp` has
/// changed, and saves/restores registers accordingly.
pub fn task_switch(_old_ctx: *mut CpuContext, new_ctx: *mut CpuContext) {
    unsafe {
        core::arch::asm!("mv tp, {}", in(reg) new_ctx as *const CpuContext as usize);
    }
}

pub(crate) fn setup_multitasking<const IRQ: u8>(_irq: SoftwareInterrupt<'static, IRQ>) {
    // Register a direct-bound interrupt handler, so that we don't have to worry about other
    // interrupt handlers interfering.

    let interrupt = match IRQ {
        0 => esp_hal::peripherals::Interrupt::FROM_CPU_INTR0,
        _ => panic!("Invalid IRQ number"),
    };

    interrupt::enable_direct(
        interrupt,
        interrupt::Priority::min(),
        interrupt::DirectBindableCpuInterrupt::Interrupt0,
        swint_handler_trampoline,
    );
}

#[cfg(multi_core)]
pub(crate) fn setup_smp<const IRQ: u8>(irq: SoftwareInterrupt<'static, IRQ>) {
    setup_multitasking(irq);
}

// We need to place this close to the trap handler for the jump to be resolved properly
/// Task switch wrapper
///
/// This function is the direct interrupt handler for the context switch software interrupt.
/// It stores context into the Context behind the current thread pointer, calls the scheduler,
/// and restores context from the Context behind the next thread pointer.
///
/// The scheduler itself only changes the thread pointer. The new thread pointer is guaranteed to be non-zero.
///
/// The current task's thread pointer can be zero, if and only if the current task is the
/// idle "task", or the current task is being deleted. The assembly contains jumps to avoid
/// storing these contexts.
///
/// We must not save the state of the idle task, or we'll risk running code with an incorrectly
/// set stack pointer inherited from the last scheduled task. We must not save the state of deleted
/// tasks, or we'll write some of the registers to memory that has been freed.
#[unsafe(link_section = ".trap.rust")]
#[unsafe(no_mangle)]
#[unsafe(naked)]
#[rustfmt::skip]
unsafe extern "C" fn swint_handler_trampoline() {
    core::arch::naked_asm! {"
        .cfi_startproc
        # https://github.com/riscv-non-isa/riscv-elf-psabi-doc/blob/139d8d8e1d8ee8c0c3ee150de709ceaab5c08417/riscv-dwarf.adoc
        # .cfi_register ra, 0x1341 # Unwind with MEPC as return address, crashes probe-rs

        # Save registers
        addi sp, sp, -16 # allocate 16 bytes for saving regs (RISC-V requires 16-byte alignment)

        # Store the thread pointer on the stack. We'll use it to check what needs to be restored
        sw tp, 0*4(sp)

        # Skip storing context for the idle context or deleted tasks (no thread pointer)
        beqz tp, 1f # Skip to calling the interrupt handler

        sw ra, 0*4(tp)
        sw t0, 1*4(tp)
        sw t1, 2*4(tp)
        sw t2, 3*4(tp)
        sw t3, 4*4(tp)
        sw t4, 5*4(tp)
        sw t5, 6*4(tp)
        sw t6, 7*4(tp)
        sw a0, 8*4(tp)
        sw a1, 9*4(tp)
        sw a2, 10*4(tp)
        sw a3, 11*4(tp)
        sw a4, 12*4(tp)
        sw a5, 13*4(tp)
        sw a6, 14*4(tp)
        sw a7, 15*4(tp)

1:
        # Let's run the interrupt handler, which runs the scheduler. If the scheduler
        # decides we need to switch context, it will change the thread pointer to the new context.
        la t0, {scheduler_interrupt_handler}
        jalr ra, t0, 0

        # Load old thread pointer and free up stack. This way we store/reload the unmodified stack pointer.
        lw t0, 0*4(sp)
        addi sp, sp, 16

        # If the thread pointer has not changed, just restore caller-saved registers
        beq t0, tp, 3f # Skip to restoring caller-saved registers in the new context

        # Skip storing context for the idle context or deleted tasks (no thread pointer)
        beqz t0, 2f # Skip to loading registers for the new context

        # If the thread pointer has changed, switch context
        # First, save registers to the old context
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
      # sw tp, 29*4(t0) # No need to save TP, it's set up when the task is created.
        sw sp, 30*4(t0)
        # mepc -> pc
        csrr t1, mepc
        sw t1, 31*4(t0)

2:
        # Next, load registers from the new context
        lw s0, 16*4(tp)
        lw s1, 17*4(tp)
        lw s2, 18*4(tp)
        lw s3, 19*4(tp)
        lw s4, 20*4(tp)
        lw s5, 21*4(tp)
        lw s6, 22*4(tp)
        lw s7, 23*4(tp)
        lw s8, 24*4(tp)
        lw s9, 25*4(tp)
        lw s10, 26*4(tp)
        lw s11, 27*4(tp)
        lw gp, 28*4(tp)
        # TP will be restored last.
        lw sp, 30*4(tp)

        lw t1, 31*4(tp)
        csrw mepc, t1

3:
        lw ra, 0*4(tp)
        lw t0, 1*4(tp)
        lw t1, 2*4(tp)
        lw t2, 3*4(tp)
        lw t3, 4*4(tp)
        lw t4, 5*4(tp)
        lw t5, 6*4(tp)
        lw t6, 7*4(tp)
        lw a0, 8*4(tp)
        lw a1, 9*4(tp)
        lw a2, 10*4(tp)
        lw a3, 11*4(tp)
        lw a4, 12*4(tp)
        lw a5, 13*4(tp)
        lw a6, 14*4(tp)
        lw a7, 15*4(tp)

        # Restore TP last. For the idle hook, this should write 0, which prevents saving its state.
        lw tp, 29*4(tp)

        mret
        .cfi_endproc
        ",
        scheduler_interrupt_handler = sym swint_handler,
    }
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
    #[cfg(feature = "rtos-trace")]
    {
        rtos_trace::trace::marker_begin(TraceEvents::YieldTask as u32);
        rtos_trace::trace::marker_end(TraceEvents::YieldTask as u32);
    }

    match Cpu::current() {
        Cpu::ProCpu => unsafe { SoftwareInterrupt::<'static, 0>::steal() }.raise(),
        #[cfg(multi_core)]
        Cpu::AppCpu => unsafe { SoftwareInterrupt::<'static, 1>::steal() }.raise(),
    }
}
