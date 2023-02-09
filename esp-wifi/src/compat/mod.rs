pub mod common;
pub mod malloc;
pub mod timer_compat;
pub mod work_queue;

pub mod queue {
    pub use heapless::spsc::Queue as SimpleQueue;
}
