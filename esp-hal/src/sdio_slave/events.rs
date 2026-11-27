// SLC0 event classification. SLC1 and host-vector bits are never acknowledged.

/// SLC0 TX is host-to-slave; SLC0 RX is slave-to-host (TRM 34.6.2).
pub(super) const RECEIVE_EVENTS: u32 = (1 << 15) | (1 << 19) | (1 << 11);
pub(super) const TRANSMIT_EVENTS: u32 = (1 << 17) | (1 << 20) | (1 << 10);
pub(super) const DMA_EVENTS: u32 = RECEIVE_EVENTS | TRANSMIT_EVENTS;

#[derive(Debug, PartialEq, Eq)]
pub(super) enum Completion {
    Pending,
    Draining,
    Complete,
    Fault,
}

pub(super) fn completion(raw: u32, transmitting: bool, parked: bool) -> Completion {
    let (eof, faults) = if transmitting {
        (1 << 17, (1 << 20) | (1 << 10))
    } else {
        (1 << 15, (1 << 19) | (1 << 11))
    };
    if raw & faults != 0 {
        Completion::Fault
    } else if raw & eof == 0 {
        Completion::Pending
    } else if !parked {
        Completion::Draining
    } else {
        Completion::Complete
    }
}

/// Register access stays in the driver; the actual future is also host-tested.
pub(super) trait CompletionSource {
    fn register(&self, waker: &core::task::Waker);
    fn poll_completion(&mut self) -> Result<Option<usize>, super::state::Error>;
    fn pending_event(&self) -> bool;
    fn arm_notification(&self);
    fn mask_notification(&self);
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(super) struct Wait<'a, S: CompletionSource> {
    source: &'a mut S,
    result: Option<Result<usize, super::state::Error>>,
}

impl<'a, S: CompletionSource> Wait<'a, S> {
    pub(super) fn new(source: &'a mut S) -> Self {
        Self {
            source,
            result: None,
        }
    }
}

impl<S: CompletionSource> core::future::Future for Wait<'_, S> {
    type Output = Result<usize, super::state::Error>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        use core::task::Poll;
        let this = self.get_mut();
        if let Some(result) = this.result {
            return Poll::Ready(result);
        }
        this.source.register(cx.waker());
        let result = match this.source.poll_completion() {
            Ok(Some(length)) => Ok(length),
            Err(error) => Err(error),
            Ok(None) => {
                if this.source.pending_event() {
                    // EOF can precede PARK without another interrupt following it.
                    this.source.mask_notification();
                    cx.waker().wake_by_ref();
                } else {
                    // An event in this window stays latched in RAW and fires
                    // when ENA is set. The waker is already registered.
                    this.source.arm_notification();
                }
                return Poll::Pending;
            }
        };
        this.source.mask_notification();
        this.result = Some(result);
        Poll::Ready(result)
    }
}

impl<S: CompletionSource> Drop for Wait<'_, S> {
    fn drop(&mut self) {
        // Notification cancellation must never return or repurpose DMA storage.
        self.source.mask_notification();
    }
}
