//! Xtensa context switching implementation.
//!
//! Context switching is implemented via interrupt handlers. Servicing the interrupt request
//! saves context to the stack. The OS copies the state to memory, replaces it with the new task's
//! state, then returns from the interrupt handler.
//!
//! Same-core yields and cross-core yields both use inter-processor call (IPC). Context switching
//! must happen at the lowest interrupt priority. This ensures that context switching does not
//! interfere with other interrupts, so an interrupt handler is not left only partially executed.

#[cfg(feature = "esp-radio")]
use core::ffi::c_void;
use core::sync::atomic::Ordering;

pub(crate) use esp_hal::trapframe::TrapFrame as CpuContext;
use esp_hal::{
    interrupt::ipc::__rtos_implementation::set_context_switch_handler,
    ram,
    system::Cpu,
};
use portable_atomic::AtomicPtr;

#[cfg(feature = "rtos-trace")]
use crate::TraceEvents;
use crate::{
    SCHEDULER,
    task::{IdleFn, Task},
};

static IDLE_HOOK: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

#[unsafe(naked)]
extern "C" fn idle_entry() -> ! {
    core::arch::naked_asm!("
        .literal idle_hook_fn, {idle_hook_fn}

        l32r   a2, idle_hook_fn
        l32i.n a2, a2, 0
        callx4 a2
    ", idle_hook_fn = sym IDLE_HOOK);
}

// Exception mode. Setting this bit prevents interrupts below EXCMLEVEL. Cleared by `rfe` at the end
// of the Level 1 interrupt handler.
const PS_EXCM: u32 = 1 << 4;
// User mode. Selects the user exception vector, instead of the kernel one. Both vectors point at
// the same handler, but tasks must run with this bit set, because the interrupt handlers do, too.
const PS_UM: u32 = 1 << 5;
// Windowed mode.
const PS_WOE: u32 = 1 << 18;
// CALLINC field value for call4 instruction.
#[cfg(feature = "esp-radio")]
const PS_CALLINC_CALL4: u32 = 1 << 16;

pub(crate) fn set_idle_hook_entry(idle_context: &mut CpuContext, hook_fn: IdleFn) {
    IDLE_HOOK.store(hook_fn as *mut (), Ordering::Relaxed);

    // Point idle context PC at the assembly that calls the idle hook. We need a new stack
    // frame for the idle task on the main stack.
    idle_context.PC = idle_entry as *const () as u32;
    // Set a valid processor status value, that will not end up spilling registers into the main
    // task's stack. Here we jump to a naked function so we can omit the CALLINC bits.
    idle_context.PS = PS_EXCM | PS_UM | PS_WOE;

    // The idle context has no thread-local data
    idle_context.THREADPTR = 0;
}

#[inline(always)]
pub(crate) fn read_thread_pointer() -> *mut Task {
    let tp: *mut Task;
    unsafe { core::arch::asm!("rur.threadptr {0}", out(reg) tp, options(nostack)) };
    tp
}

#[inline(always)]
pub(crate) fn write_thread_pointer(task: *mut Task) {
    unsafe { core::arch::asm!("wur.threadptr {0}", in(reg) task, options(nostack)) };
}

#[cfg(feature = "esp-radio")]
pub(crate) fn new_task_context(
    task_fn: extern "C" fn(*mut c_void),
    param: *mut c_void,
    stack_top: *mut (),
) -> CpuContext {
    // stack must be aligned by 16
    let stack_top = stack_top as u32;
    let stack_top = stack_top - (stack_top % 16);

    unsafe {
        *((stack_top - 4) as *mut u32) = 0;
        *((stack_top - 8) as *mut u32) = 0;
        *((stack_top - 12) as *mut u32) = stack_top;
        *((stack_top - 16) as *mut u32) = 0;
    }

    CpuContext {
        PC: super::task_wrapper as *const () as u32,
        A0: 0,
        A1: stack_top,
        A6: task_fn as usize as u32,
        A7: param as usize as u32,

        // For windowed ABI set WOE, UM, EXCM and CALLINC = call4 (pretend task was 'call4'd)
        // UM and EXCM are important, so that restoring context will correctly restore an exception
        // context (where the context switch happens).
        PS: PS_EXCM | PS_UM | PS_WOE | PS_CALLINC_CALL4,

        ..Default::default()
    }
}

#[inline]
pub(crate) fn task_switch(
    current_context: *mut CpuContext,
    next_context: *mut CpuContext,
    trap_frame: &mut CpuContext,
) {
    if !current_context.is_null() {
        unsafe { core::ptr::copy_nonoverlapping(trap_frame, current_context, 1) };
    }
    unsafe { core::ptr::copy_nonoverlapping(next_context, trap_frame, 1) };
}

pub(crate) fn setup_multitasking() {
    // The IPC interrupt is the lowest-priority interrupt of this CPU, and runs its context
    // switch. Same-core yields and cross-core yields both use it.
    unsafe {
        set_context_switch_handler(Cpu::current(), trigger_task_switch);
    }
}

#[ram]
extern "C" fn trigger_task_switch(context: &mut CpuContext) {
    SCHEDULER.with(|scheduler| scheduler.switch_task(context));
}
