//! Asynchronous utilities.
use core::{
    future::poll_fn,
    task::{Context, Poll, Waker},
};

use embassy_sync::waitqueue::GenericAtomicWaker;
use esp_sync::RawMutex;

use crate::time::Instant;

/// Arranges for the task behind `cx` to be polled again no later than `deadline`.
///
/// With the `embassy-time` feature this schedules a timer wake-up. Without it, the task is woken
/// immediately, so it keeps busy-polling until the deadline.
pub(crate) fn wake_at(deadline: Instant, cx: &mut Context<'_>) {
    cfg_select! {
        feature = "embassy-time" => {
            use core::future::Future;

            let now = Instant::now();
            let remaining = if deadline > now {
                (deadline - now).as_micros()
            } else {
                0
            };

            let mut timer = embassy_time::Timer::after_micros(remaining);
            let _ = core::pin::Pin::new(&mut timer).poll(cx);
        }
        _ => {
            let _ = deadline;
            cx.waker().wake_by_ref();
        }
    }
}

/// Waits until `deadline`. See [`wake_at`].
pub(crate) async fn wait_until(deadline: Instant) {
    poll_fn(|cx| {
        if Instant::now() >= deadline {
            Poll::Ready(())
        } else {
            wake_at(deadline, cx);
            Poll::Pending
        }
    })
    .await
}

/// Utility struct to register and wake a waker.
pub struct AtomicWaker {
    waker: GenericAtomicWaker<RawMutex>,
}

impl AtomicWaker {
    /// Creates a new `AtomicWaker`.
    #[allow(clippy::new_without_default)]
    pub const fn new() -> Self {
        Self {
            waker: GenericAtomicWaker::new(RawMutex::new()),
        }
    }

    /// Registers a waker. Overwrites the previous waker, if any.
    #[inline]
    pub fn register(&self, w: &Waker) {
        self.waker.register(w);
    }

    /// Wakes the registered waker, if any.
    #[crate::ram]
    pub fn wake(&self) {
        self.waker.wake();
    }
}
