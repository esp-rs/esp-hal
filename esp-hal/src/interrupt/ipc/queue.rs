//! The request queue of the IPC.
//!
//! This file uses `core` and `portable_atomic` only, because the host tests include it.

use core::ptr;

use portable_atomic::{AtomicPtr, Ordering};

pub(crate) use super::request::*;

/// A run is pending. Its sequence number is in the `seq` bits.
pub(crate) const QUEUED: u32 = 1 << 0;
/// `func` runs on the CPU in the `CORE` bit.
pub(crate) const RUNNING: u32 = 1 << 1;
/// The CPU of the pending or running run.
pub(crate) const CORE: u32 = 1 << 2;

/// The target CPU stopped before the run.
pub(crate) const OFFLINE: u32 = 1;

/// The request belongs to the other CPU.
pub(crate) const POST_BUSY: u32 = 2;

/// The bits of the sequence number in `word` and in `status`.
const SEQ_FIELD: u32 = !((1 << SEQ_SHIFT) - 1);

/// The request lists of one CPU.
#[derive(Debug)]
pub(crate) struct Lists {
    /// The requests that posts pushed, last post first.
    pub(crate) head: AtomicPtr<IpcRequest>,
    /// The requests that the handler took and did not run yet, in post order.
    pub(crate) drain: AtomicPtr<IpcRequest>,
    /// The request that runs now.
    pub(crate) current: AtomicPtr<IpcRequest>,
}

impl Lists {
    pub(crate) const fn new() -> Self {
        Self {
            head: AtomicPtr::new(ptr::null_mut()),
            drain: AtomicPtr::new(ptr::null_mut()),
            current: AtomicPtr::new(ptr::null_mut()),
        }
    }
}

#[inline(always)]
fn core_bit(core: u32) -> u32 {
    if core == 0 { 0 } else { CORE }
}

/// Queues a run of `request` on the CPU `core`, or joins its pending run.
///
/// Returns the `seq` bits of the run on success, [`POST_BUSY`] otherwise. Calls `raise` after it
/// queued a run.
///
/// # Safety
///
/// The caller masks the interrupts of the current CPU, and checks that `core` takes IPC
/// interrupts. `request` stays valid as long as the runtime can reach it.
pub(crate) unsafe fn post(
    lists: &Lists,
    core: u32,
    request: &IpcRequest,
    raise: impl FnOnce(),
) -> u32 {
    let core = core_bit(core);
    let mut word = request.word.load(Ordering::Acquire);
    loop {
        let same_core = word & CORE == core;

        if word & QUEUED == 0 {
            // After the store of `status`, the other CPU does not use the request again.
            let running = if same_core { word & RUNNING } else { 0 };
            if word & RUNNING != 0 && !same_core && poll(request, word & SEQ_FIELD).is_none() {
                return POST_BUSY;
            }

            let seq = (word & SEQ_FIELD).wrapping_add(1 << SEQ_SHIFT);
            let queued = seq | QUEUED | core | running;
            if let Err(actual) = request.word.compare_exchange_weak(
                word,
                queued,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                word = actual;
                continue;
            }

            unsafe { push(lists, request) };
            raise();

            return seq;
        }

        return if same_core {
            word & SEQ_FIELD
        } else {
            POST_BUSY
        };
    }
}

unsafe fn push(lists: &Lists, request: &IpcRequest) {
    let request = ptr::from_ref(request).cast_mut();
    let mut head = lists.head.load(Ordering::Acquire);
    loop {
        unsafe { (*request).next.store(head, Ordering::Relaxed) };
        match lists
            .head
            .compare_exchange_weak(head, request, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => return,
            Err(actual) => head = actual,
        }
    }
}

/// Runs the requests that posts pushed to `lists`, in post order.
///
/// # Safety
///
/// Only the CPU `core`, which owns `lists`, calls this function, from its IPC interrupt, with
/// masked interrupts. The CPU `core` stops only inside `func`.
// The IPC handler runs from RAM, and the flash can be off while the handler runs.
#[inline(always)]
pub(crate) unsafe fn drain(lists: &Lists, core: u32) {
    let mut taken = lists.head.swap(ptr::null_mut(), Ordering::AcqRel);
    let mut ordered = ptr::null_mut();
    while let Some(request) = unsafe { taken.as_ref() } {
        let next = request.next.load(Ordering::Relaxed);
        request.next.store(ordered, Ordering::Relaxed);
        ordered = taken;
        taken = next;
    }
    lists.drain.store(ordered, Ordering::Release);

    loop {
        let ptr = lists.drain.load(Ordering::Acquire);
        let Some(request) = (unsafe { ptr.as_ref() }) else {
            return;
        };

        // Read `next` before clearing `queued`. A post can take the request again after that.
        let next = request.next.load(Ordering::Relaxed);

        // `fail_stranded` reads `current` and `drain` after a stop inside `func`.
        lists.current.store(ptr, Ordering::Release);
        lists.drain.store(next, Ordering::Release);

        let mut word = request.word.load(Ordering::Acquire);
        let seq = loop {
            let running = (word & !QUEUED) | RUNNING;
            match request.word.compare_exchange_weak(
                word,
                running,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => break word & SEQ_FIELD,
                Err(actual) => word = actual,
            }
        };

        (request.func)();

        complete(request, seq, DONE);
        clear_running(request, core);
        lists.current.store(ptr::null_mut(), Ordering::Release);
    }
}

/// Stores the outcome of the run of `request` with the `seq` bits `seq`.
///
/// Only the CPU that the run belongs to calls this function, before it gives up the request.
#[inline(always)]
fn complete(request: &IpcRequest, seq: u32, outcome: u32) {
    request.status.store(seq | outcome, Ordering::Release);
}

/// Clears the `running` bit of `request`, if the request still belongs to the CPU `core`.
///
/// After [`complete`], a post to the other CPU can take the request and set its own `running`
/// bit. A post during `func` to the CPU `core` queued a new run, so only `running` is cleared.
#[inline(always)]
pub(crate) fn clear_running(request: &IpcRequest, core: u32) {
    let core = core_bit(core);
    let mut word = request.word.load(Ordering::Acquire);
    while word & CORE == core && word & RUNNING != 0 {
        match request.word.compare_exchange_weak(
            word,
            word & !RUNNING,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => return,
            Err(actual) => word = actual,
        }
    }
}

/// Fails each request on `lists` with [`OFFLINE`], and empties the lists.
///
/// # Safety
///
/// The CPU `core` owns `lists`, and stopped inside `func` or outside [`drain`]. No post adds to
/// `lists` during the call.
pub(crate) unsafe fn fail_stranded(lists: &Lists, core: u32) {
    let core = core_bit(core);
    for list in [&lists.drain, &lists.head] {
        let mut ptr = list.load(Ordering::Acquire);
        while let Some(request) = unsafe { ptr.as_ref() } {
            // Read `next` before clearing `queued`. A post to the other CPU can take the request
            // after that.
            ptr = request.next.load(Ordering::Relaxed);
            fail(request, core);
        }
        list.store(ptr::null_mut(), Ordering::Release);
    }

    // `current` is no list: its `next` can belong to a list of the other CPU.
    if let Some(request) = unsafe { lists.current.load(Ordering::Acquire).as_ref() } {
        fail(request, core);
    }
    lists.current.store(ptr::null_mut(), Ordering::Release);
}

fn fail(request: &IpcRequest, core: u32) {
    let mut word = request.word.load(Ordering::Acquire);
    let seq = loop {
        // A request that posted itself during `func` is in `current` and in `head`. After the
        // first fail, it can belong to the other CPU.
        if word & CORE != core || word & (QUEUED | RUNNING) == 0 {
            return;
        }

        // A post to the other CPU takes a pending request after the store of `status`. The
        // request must thus be running at that store, as in `drain`.
        match request.word.compare_exchange_weak(
            word,
            (word & !QUEUED) | RUNNING,
            Ordering::AcqRel,
            Ordering::Acquire,
        ) {
            Ok(_) => break word & SEQ_FIELD,
            Err(actual) => word = actual,
        }
    };

    complete(request, seq, OFFLINE);
    clear_running(request, core);
}
