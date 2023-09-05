use super::queue::SimpleQueue;

static mut WORKER_HIGH: Option<
    SimpleQueue<
        (
            extern "C" fn(*mut crate::binary::c_types::c_void),
            *mut crate::binary::c_types::c_void,
        ),
        10,
    >,
> = None;

pub fn queue_work(
    task_func: *mut crate::binary::c_types::c_void,
    _name: *const crate::binary::c_types::c_char,
    _stack_depth: u32,
    param: *mut crate::binary::c_types::c_void,
    prio: u32,
    _task_handle: *mut crate::binary::c_types::c_void,
    _core_id: u32,
) {
    trace!(
        "work_queue task {:?} param {:?} prio {}",
        task_func,
        param,
        prio
    );

    critical_section::with(|_| unsafe {
        if WORKER_HIGH.is_none() {
            WORKER_HIGH = Some(SimpleQueue::new());
        }

        let _ = &unwrap!(WORKER_HIGH.as_mut()).enqueue((core::mem::transmute(task_func), param));
    });
}

pub fn do_work() {
    unsafe {
        let mut todo: [Option<(
            extern "C" fn(*mut crate::binary::c_types::c_void),
            *mut crate::binary::c_types::c_void,
        )>; 10] = [None; 10];

        critical_section::with(|_| {
            if WORKER_HIGH.is_none() {
                return;
            }

            todo.iter_mut().for_each(|e| {
                let work = &unwrap!(WORKER_HIGH.as_mut()).dequeue();

                match work {
                    Some(worker) => {
                        e.replace(*worker);
                    }
                    None => {}
                }
            });
        });

        for worker in todo.iter() {
            match worker {
                core::option::Option::Some((f, p)) => {
                    trace!("before worker {:?} {:?}", f, *p);

                    f(*p);

                    trace!("after worker");
                }
                core::option::Option::None => {}
            }
        }
    }
}
