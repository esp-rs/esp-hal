pub use self::{interrupt::*, thread::*};

mod interrupt;
mod thread;

#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    use esp_hal::interrupt::software::SoftwareInterrupt;

    match context as usize {
        // For interrupt executors, the context value is the
        // software interrupt number
        0 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
        1 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
        2 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
        #[cfg(not(multi_core))]
        3 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
        // THREAD_MODE_CONTEXT + core ID
        16 => thread::pend_thread_mode(0),
        #[cfg(multi_core)]
        17 => thread::pend_thread_mode(1),
        _ => unreachable!(),
    }
}
