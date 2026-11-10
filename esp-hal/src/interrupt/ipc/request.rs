//! The part of the IPC that every esp-hal copy shares.
//!
//! This file uses `core` and `portable_atomic` only, because the host tests include it.
// esp-hal copies without `rt` read requests that the copy with `rt` writes. The layout of
// `IpcRequest`, the encodings below and `seq_reached` must stay stable.

use core::ptr;

use portable_atomic::{AtomicPtr, AtomicU32, AtomicUsize, Ordering};

/// The position of the sequence number in `status` and in the result of a post.
pub(crate) const SEQ_SHIFT: u32 = 5;

/// The outcome bits of `status`.
pub(crate) const OUTCOME_MASK: u32 = 0b11;
/// The run ended. Each other outcome means that the target CPU stopped before the run.
pub(crate) const DONE: u32 = 0;

/// The post queued a run, or joined a pending run.
pub(crate) const POST_OK: u32 = 0;
/// The target CPU does not take IPC interrupts. Each other result means that the request is busy.
pub(crate) const POST_OFFLINE: u32 = 1;
/// The result bits of the return value of a post.
pub(crate) const POST_RESULT_MASK: u32 = (1 << SEQ_SHIFT) - 1;

/// A function that [`Ipc::post`][super::Ipc::post] runs on a CPU, and the state of its runs.
///
/// The IPC interrupt of the target CPU calls `func` at
/// [`Priority::min`][crate::interrupt::Priority::min] with the argument of the post. A request must
/// be `'static`, because the IPC can read and write it at any time after the first post. A request
/// has at most one pending run. Two concurrent runs need two requests.
///
/// `func` must:
///
/// - return,
/// - not use the floating-point registers,
/// - not enable interrupts, and not lower the run level,
/// - not wait for a run on its own CPU.
///
/// A caller can stop its wait before the run starts, so `func` can run late. `func` must then find
/// its protocol state stale, and do nothing.
#[repr(C)]
#[derive(Debug)]
pub struct IpcRequest {
    pub(super) next: AtomicPtr<IpcRequest>,
    /// The state of the pending and the running run. Only the copy with `rt` reads the bits. A
    /// request that no post took holds 0.
    pub(super) word: AtomicU32,
    pub(super) status: AtomicU32,
    pub(super) func: extern "C" fn(usize),
    pub(super) arg: AtomicUsize,
}

impl IpcRequest {
    /// Creates a request that runs `func` with the argument of the post.
    pub const fn new(func: extern "C" fn(usize)) -> Self {
        Self {
            next: AtomicPtr::new(ptr::null_mut()),
            word: AtomicU32::new(0),
            status: AtomicU32::new(0),
            func,
            arg: AtomicUsize::new(0),
        }
    }
}

/// Returns `true` if the sequence number in the `seq` bits of `current` reached the one of
/// `target`.
///
/// Sequence numbers have 27 bits and wrap. The bits of `target` below the `seq` bits are 0.
#[inline(always)]
pub(crate) fn seq_reached(current: u32, target: u32) -> bool {
    current.wrapping_sub(target) as i32 >= 0
}

/// Returns the outcome of the run of `request` with the `seq` bits `seq`, or `None` if the run
/// did not end.
pub(crate) fn poll(request: &IpcRequest, seq: u32) -> Option<u32> {
    let status = request.status.load(Ordering::Acquire);
    seq_reached(status, seq).then_some(status & OUTCOME_MASK)
}
