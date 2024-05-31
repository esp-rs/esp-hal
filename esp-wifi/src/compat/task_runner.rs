use super::queue::SimpleQueue;
use crate::{binary::c_types, compat::common::str_from_c, timer::yield_task};

type TaskFunc = (extern "C" fn(*mut c_types::c_void), *mut c_types::c_void);

static mut TASK_SPAWN_QUEUE: SimpleQueue<TaskFunc, 4> = SimpleQueue::new();

pub fn spawn_task(
    task_func: *mut c_types::c_void,
    name: *const c_types::c_char,
    _stack_depth: u32,
    param: *mut c_types::c_void,
    prio: u32,
    _task_handle: *mut c_types::c_void,
    _core_id: u32,
) -> bool {
    debug!(
        "spawning task {}: {:?} param {:?} prio {}",
        unsafe { str_from_c(name.cast()) },
        task_func,
        param,
        prio
    );

    // TODO: allocate a stack and insert into the task queue

    critical_section::with(|_| unsafe {
        if TASK_SPAWN_QUEUE
            .enqueue((
                core::mem::transmute::<*mut c_types::c_void, extern "C" fn(*mut c_types::c_void)>(
                    task_func,
                ),
                param,
            ))
            .is_ok()
        {
            true
        } else {
            warn!("task spawn queue full");
            false
        }
    })
}

/// This function runs a single C task started by the wifi stack.
pub(crate) extern "C" fn run_c_task() {
    loop {
        // Take a task and run it.
        if let Some((f, p)) = critical_section::with(|_| unsafe { TASK_SPAWN_QUEUE.dequeue() }) {
            debug!("task started: {:?} {:?}", f, p);
            f(p);
            debug!("task finished: {:?} {:?}", f, p);
        }
        yield_task();
    }
}
