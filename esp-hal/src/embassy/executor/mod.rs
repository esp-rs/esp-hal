pub mod thread;

pub use thread::*;

pub mod interrupt;

pub use interrupt::*;

#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    use crate::system::SoftwareInterrupt;

    let context = (context as usize).to_le_bytes();

    match context[0] {
        // 0 is reserved for thread mode executors
        0 => thread::pend_thread_mode(context[1] as usize),
        // For interrupt executors, the context value is the 
        // software interrupt number + `SW_OFFSET`
        16 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
        17 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
        18 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
        19 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
        _ => {}
    }
}
