mod interrupt;
mod thread;

pub use interrupt::*;
pub use thread::*;

#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    use crate::system::SoftwareInterrupt;

    let context = (context as usize).to_le_bytes();

    match context[0] {
<<<<<<< HEAD
        // For interrupt executors, the context value is the
        // software interrupt number
        0 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
        1 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
        2 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
        3 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
        other => {
            assert_eq!(other, THREAD_MODE_CONTEXT);
            // THREAD_MODE_CONTEXT id is reserved for thread mode executors
            thread::pend_thread_mode(context[1] as usize)
        }
=======
        // 0 is reserved for thread mode executors
        0 => thread::pend_thread_mode(context[1] as usize),
        // For interrupt executors, the context value is the
        // software interrupt number + `SW_OFFSET`
        16 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
        17 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
        18 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
        19 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
        _ => {}
>>>>>>> 2d5150dc (Reserve sw interrupt 3 (4) instead of 0 for multicore systems with the embassy feature enabled)
    }
}
