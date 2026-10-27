//! Xtensa context switching implementation.
//!
//! The interrupt entry of esp-hal saves the context of the interrupted task as a frame on the
//! stack of that task. The context switch handler stores the address of that frame in the task,
//! and returns the frame of the task to resume.

use core::{ffi::c_void, sync::atomic::Ordering};

use esp_hal::interrupt::__rtos_implementation::TASK_FRAME_SIZE;
pub(crate) use esp_hal::interrupt::__rtos_implementation::TaskFrame;
use portable_atomic::AtomicPtr;

use crate::task::{IdleFn, Task};

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
const PS_CALLINC_CALL4: u32 = 1 << 16;

pub(crate) fn set_idle_hook(hook_fn: IdleFn) {
    IDLE_HOOK.store(hook_fn as *mut (), Ordering::Relaxed);
}

/// Writes a frame that enters the idle hook with its stack pointer at `stack_top`, and returns
/// the frame.
pub(crate) fn new_idle_frame(stack_top: *mut TaskFrame) -> *mut TaskFrame {
    let frame = stack_top.wrapping_byte_sub(TASK_FRAME_SIZE);

    let context = TaskFrame {
        PC: idle_entry as *const () as u32,
        A1: stack_top as u32,
        // A valid processor status that does not spill registers above `stack_top`. `idle_entry`
        // is naked, so the CALLINC bits can stay 0.
        PS: PS_EXCM | PS_UM | PS_WOE,
        // The idle context has no thread-local data.
        THREADPTR: 0,
        ..Default::default()
    };
    unsafe { frame.write(context) };

    frame
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

/// Writes the first frame of a task at the top of its stack, and returns the frame.
pub(crate) fn new_task_context(
    task_fn: extern "C" fn(*mut c_void),
    param: *mut c_void,
    stack_top: *mut (),
) -> *mut TaskFrame {
    // stack must be aligned by 16
    let stack_top = stack_top as u32;
    let stack_top = stack_top - (stack_top % 16);

    // The base save area of the task entry. The frame below leaves these words alone.
    unsafe {
        *((stack_top - 4) as *mut u32) = 0;
        *((stack_top - 8) as *mut u32) = 0;
        *((stack_top - 12) as *mut u32) = stack_top;
        *((stack_top - 16) as *mut u32) = 0;
    }

    let frame = (stack_top as usize - TASK_FRAME_SIZE) as *mut TaskFrame;
    let context = TaskFrame {
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
    };
    unsafe { frame.write(context) };

    frame
}
