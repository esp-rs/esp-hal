#[cfg(feature = "executor-interrupt")]
pub use self::interrupt::*;
#[cfg(feature = "executor-thread")]
pub use self::thread::*;

#[cfg(feature = "executor-interrupt")]
mod interrupt;
#[cfg(feature = "executor-thread")]
mod thread;

#[cfg(any(feature = "executor-interrupt", feature = "executor-thread"))]
#[export_name = "__pender"]
fn __pender(context: *mut ()) {
    let context = (context as usize).to_le_bytes();

    cfg_if::cfg_if! {
        if #[cfg(feature = "executor-interrupt")] {
            match context[0] {
                #[cfg(feature = "executor-thread")]
                0 => thread::pend_thread_mode(context[1] as usize),
                1 => FromCpu1::pend(),
                2 => FromCpu2::pend(),
                3 => FromCpu3::pend(),
                _ => {}
            }
        } else {
            pend_thread_mode(context[1] as usize);
        }
    }
}
