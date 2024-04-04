#[cfg(feature = "embassy-executor-thread")]
pub mod thread;

#[cfg(feature = "embassy-executor-thread")]
pub use thread::*;

#[cfg(feature = "embassy-executor-interrupt")]
pub mod interrupt;

#[cfg(feature = "embassy-executor-interrupt")]
pub use interrupt::*;

#[cfg(any(
    feature = "embassy-executor-thread",
    feature = "embassy-executor-interrupt",
))]
#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    #[cfg(feature = "embassy-executor-interrupt")]
    use crate::system::SoftwareInterrupt;

    let context = (context as usize).to_le_bytes();

    cfg_if::cfg_if! {
        if #[cfg(feature = "embassy-executor-interrupt")] {
            match context[0] {
                #[cfg(feature = "embassy-executor-thread")]
                0 => thread::pend_thread_mode(context[1] as usize),

                #[cfg(not(feature = "embassy-executor-thread"))]
                0 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
                1 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
                2 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
                3 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
                _ => {}
            }
        } else {
            pend_thread_mode(context[1] as usize);
        }
    }
}
