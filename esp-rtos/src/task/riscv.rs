use core::{ffi::c_void, sync::atomic::Ordering};

use esp_hal::interrupt::__rtos_implementation::TASK_FRAME_SIZE;
pub(crate) use esp_hal::interrupt::__rtos_implementation::TaskFrame;
use portable_atomic::AtomicPtr;

use crate::task::{IdleFn, Task};

static IDLE_HOOK: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

pub(crate) fn set_idle_hook(hook_fn: IdleFn) {
    IDLE_HOOK.store(hook_fn as *mut (), Ordering::Relaxed);
}

/// Writes a frame that enters the idle hook with its stack pointer at `stack_top`, and returns
/// the frame.
pub(crate) fn new_idle_frame(stack_top: *mut TaskFrame) -> *mut TaskFrame {
    let frame = stack_top.wrapping_byte_sub(TASK_FRAME_SIZE);

    // The idle context has no thread-local data, so `tp` stays 0.
    let mut context = TaskFrame::default();
    context.pc = IDLE_HOOK.load(Ordering::Relaxed) as usize;
    unsafe { frame.write(context) };

    frame
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

/// Writes the first frame of a task at the top of its stack, and returns the frame.
pub(crate) fn new_task_context(
    task: extern "C" fn(*mut c_void),
    param: *mut c_void,
    stack_top: *mut (),
) -> *mut TaskFrame {
    let stack_top = stack_top as usize;
    let stack_top = stack_top - (stack_top % 16);
    let frame = (stack_top - TASK_FRAME_SIZE) as *mut TaskFrame;

    let mut context = TaskFrame::default();
    context.pc = super::task_wrapper as *const () as usize;
    context.a0 = task as usize;
    context.a1 = param as usize;
    unsafe { frame.write(context) };

    frame
}
