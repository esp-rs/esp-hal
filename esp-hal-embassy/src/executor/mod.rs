pub use self::{interrupt::*, thread::*};

mod interrupt;
mod thread;

#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    use esp_hal::interrupt::software::SoftwareInterrupt;

    let context = (context as usize).to_le_bytes();

    match context[0] {
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
    }
}
