#[cfg(riscv)]
pub use riscv::*;
#[cfg(xtensa)]
pub use xtensa::*;

#[cfg(riscv)]
mod riscv;

#[cfg(xtensa)]
mod xtensa;

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
    let context = (context as usize).to_le_bytes();

    cfg_if::cfg_if! {
        if #[cfg(feature = "embassy-executor-interrupt")] {
            match context[0] {
                #[cfg(feature = "embassy-executor-thread")]
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
