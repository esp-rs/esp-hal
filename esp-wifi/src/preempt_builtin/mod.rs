#[cfg_attr(target_arch = "riscv32", path = "preempt_riscv.rs")]
#[cfg_attr(target_arch = "xtensa", path = "preempt_xtensa.rs")]
mod arch_specific;
pub mod timer;

use core::{ffi::c_void, mem::MaybeUninit};

use allocator_api2::boxed::Box;
use arch_specific::*;
pub(crate) use timer::setup_timer;
use timer::{disable_multitasking, setup_multitasking};

use crate::{
    compat::malloc::InternalMemory,
    hal::{sync::Locked, trapframe::TrapFrame},
    preempt::Scheduler,
    preempt_builtin::timer::disable_timebase,
};

struct Context {
    trap_frame: TrapFrame,
    pub thread_semaphore: u32,
    pub next: *mut Context,
    pub _allocated_stack: Box<[MaybeUninit<u8>], InternalMemory>,
}

impl Context {
    pub(crate) fn new(
        task_fn: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
    ) -> Self {
        trace!("task_create {:?} {:?} {}", task_fn, param, task_stack_size);

        let mut stack = Box::<[u8], _>::new_uninit_slice_in(task_stack_size, InternalMemory);

        let stack_top = unsafe { stack.as_mut_ptr().add(task_stack_size).cast() };

        Context {
            trap_frame: new_task_context(task_fn, param, stack_top),
            thread_semaphore: 0,
            next: core::ptr::null_mut(),
            _allocated_stack: stack,
        }
    }
}

struct SchedulerState {
    /// Pointer to the current task.
    ///
    /// Tasks are stored in a circular linked list. CTX_NOW points to the
    /// current task.
    current_task: *mut Context,

    /// Pointer to the task that is scheduled for deletion.
    to_delete: *mut Context,
}

impl SchedulerState {
    const fn new() -> Self {
        Self {
            current_task: core::ptr::null_mut(),
            to_delete: core::ptr::null_mut(),
        }
    }

    fn delete_task(&mut self, task: *mut Context) {
        let mut current_task = self.current_task;
        // Save the first pointer so we can prevent an accidental infinite loop.
        let initial = current_task;
        loop {
            // We don't have the previous pointer, so we need to walk forward in the circle
            // even if we need to delete the first task.

            // If the next task is the one we want to delete, we need to remove it from the
            // list, then drop it.
            let next_task = unsafe { (*current_task).next };
            if core::ptr::eq(next_task, task) {
                unsafe {
                    (*current_task).next = (*next_task).next;

                    core::ptr::drop_in_place(task);
                    break;
                }
            }

            // If the next task is the first task, we can stop. If we needed to delete the
            // first task, we have already handled it in the above case. If we needed to
            // delete another task, it has already been deleted in a previous iteration.
            if core::ptr::eq(next_task, initial) {
                break;
            }

            // Move to the next task.
            current_task = next_task;
        }
    }

    fn switch_task(&mut self, trap_frame: &mut TrapFrame) {
        save_task_context(unsafe { &mut *self.current_task }, trap_frame);

        if !self.to_delete.is_null() {
            let task_to_delete = core::mem::replace(&mut self.to_delete, core::ptr::null_mut());
            self.delete_task(task_to_delete);
        }

        unsafe { self.current_task = (*self.current_task).next };

        restore_task_context(unsafe { &mut *self.current_task }, trap_frame);
    }

    fn schedule_task_deletion(&mut self, task: *mut Context) -> bool {
        self.to_delete = task;
        core::ptr::eq(task, self.current_task)
    }
}

static SCHEDULER_STATE: Locked<SchedulerState> = Locked::new(SchedulerState::new());

struct BuiltinScheduler {}

crate::scheduler_impl!(static SCHEDULER: BuiltinScheduler = BuiltinScheduler {});

impl Scheduler for BuiltinScheduler {
    fn enable(&self) {
        // allocate the main task
        allocate_main_task();
        setup_multitasking();
    }

    fn disable(&self) {
        disable_timebase();
        disable_multitasking();
        delete_all_tasks();
    }

    fn yield_task(&self) {
        timer::yield_task()
    }

    fn task_create(
        &self,
        task: extern "C" fn(*mut c_void),
        param: *mut c_void,
        task_stack_size: usize,
    ) -> *mut c_void {
        let task = Box::new_in(Context::new(task, param, task_stack_size), InternalMemory);
        let task_ptr = Box::into_raw(task);

        SCHEDULER_STATE.with(|state| unsafe {
            let current_task = state.current_task;
            debug_assert!(
                !current_task.is_null(),
                "Tried to allocate a task before allocating the main task"
            );
            // Insert the new task at the next position.
            let next = (*current_task).next;
            (*task_ptr).next = next;
            (*current_task).next = task_ptr;
        });

        task_ptr as *mut c_void
    }

    fn current_task(&self) -> *mut c_void {
        current_task() as *mut c_void
    }

    fn schedule_task_deletion(&self, task_handle: *mut c_void) {
        schedule_task_deletion(task_handle as *mut Context)
    }

    fn current_task_thread_semaphore(&self) -> *mut crate::binary::c_types::c_void {
        unsafe {
            &mut ((*current_task()).thread_semaphore) as *mut _
                as *mut crate::binary::c_types::c_void
        }
    }
}

fn allocate_main_task() {
    // This context will be filled out by the first context switch.
    let context = Box::new_in(
        Context {
            trap_frame: TrapFrame::default(),
            thread_semaphore: 0,
            next: core::ptr::null_mut(),
            _allocated_stack: Box::<[u8], _>::new_uninit_slice_in(0, InternalMemory),
        },
        InternalMemory,
    );

    let context_ptr = Box::into_raw(context);
    unsafe {
        // The first task loops back to itself.
        (*context_ptr).next = context_ptr;
    }

    SCHEDULER_STATE.with(|state| {
        debug_assert!(
            state.current_task.is_null(),
            "Tried to allocate main task multiple times"
        );
        state.current_task = context_ptr;
    })
}

fn delete_all_tasks() {
    let first_task = SCHEDULER_STATE.with(|state| {
        // Remove all tasks from the list. We will drop them outside of the critical
        // section.
        core::mem::replace(&mut state.current_task, core::ptr::null_mut())
    });

    if first_task.is_null() {
        return;
    }

    let mut task_to_delete = first_task;

    loop {
        let next_task = unsafe {
            // SAFETY: Tasks are in a circular linked list. We are guaranteed that the next
            // task is a valid pointer, or the first task that may already have been
            // deleted. In the second case, we will not move on to the next
            // iteration, so the loop will not try to free a task twice.
            let next_task = (*task_to_delete).next;
            core::ptr::drop_in_place(task_to_delete);
            next_task
        };

        if core::ptr::eq(next_task, first_task) {
            break;
        }

        task_to_delete = next_task;
    }
}

fn current_task() -> *mut Context {
    SCHEDULER_STATE.with(|state| state.current_task)
}

fn schedule_task_deletion(task: *mut Context) {
    let deleting_current = SCHEDULER_STATE.with(|state| state.schedule_task_deletion(task));

    // Tasks are deleted during context switches, so we need to yield if we are
    // deleting the current task.
    if deleting_current {
        loop {
            timer::yield_task();
        }
    }
}

pub(crate) fn task_switch(trap_frame: &mut TrapFrame) {
    SCHEDULER_STATE.with(|state| state.switch_task(trap_frame));
}
