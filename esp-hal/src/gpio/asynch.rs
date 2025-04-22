use core::{
    future::poll_fn,
    sync::atomic::Ordering,
    task::{Context, Poll},
};

use procmacros::ram;

use crate::{
    asynch::AtomicWaker,
    gpio::{Event, Flex, GpioBank, Input, NUM_PINS, set_int_enable},
};

#[ram]
pub(super) static PIN_WAKERS: [AtomicWaker; NUM_PINS] = [const { AtomicWaker::new() }; NUM_PINS];

impl Flex<'_> {
    /// Wait until the pin experiences a particular [`Event`].
    ///
    /// The GPIO driver will disable listening for the event once it occurs,
    /// or if the `Future` is dropped.
    ///
    /// Note that calling this function will overwrite previous
    /// [`listen`][Self::listen] operations for this pin.
    #[inline]
    #[instability::unstable]
    pub async fn wait_for(&mut self, event: Event) {
        // We construct the Future first, because its `Drop` implementation
        // is load-bearing if `wait_for` is dropped during the initialization.
        let future = PinFuture { pin: self };

        // Make sure this pin is not being processed by an interrupt handler.
        if future.pin.is_listening() {
            set_int_enable(
                future.pin.number(),
                None, // Do not disable handling pending interrupts.
                0,    // Disable generating new events
                false,
            );
            poll_fn(|cx| {
                if future.pin.is_interrupt_set() {
                    cx.waker().wake_by_ref();
                    Poll::Pending
                } else {
                    Poll::Ready(())
                }
            })
            .await;
        }

        // At this point the pin is no longer listening, we can safely
        // do our setup.

        // Mark pin as async.
        future
            .bank()
            .async_operations()
            .fetch_or(future.mask(), Ordering::Relaxed);

        future.pin.listen(event);

        future.await
    }

    /// Wait until the pin is high.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    #[instability::unstable]
    pub async fn wait_for_high(&mut self) {
        self.wait_for(Event::HighLevel).await
    }

    /// Wait until the pin is low.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    #[instability::unstable]
    pub async fn wait_for_low(&mut self) {
        self.wait_for(Event::LowLevel).await
    }

    /// Wait for the pin to undergo a transition from low to high.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    #[instability::unstable]
    pub async fn wait_for_rising_edge(&mut self) {
        self.wait_for(Event::RisingEdge).await
    }

    /// Wait for the pin to undergo a transition from high to low.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    #[instability::unstable]
    pub async fn wait_for_falling_edge(&mut self) {
        self.wait_for(Event::FallingEdge).await
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high
    /// to low.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    #[instability::unstable]
    pub async fn wait_for_any_edge(&mut self) {
        self.wait_for(Event::AnyEdge).await
    }
}

impl Input<'_> {
    /// Wait until the pin experiences a particular [`Event`].
    ///
    /// The GPIO driver will disable listening for the event once it occurs,
    /// or if the `Future` is dropped.
    ///
    /// Note that calling this function will overwrite previous
    /// [`listen`][Self::listen] operations for this pin.
    #[inline]
    pub async fn wait_for(&mut self, event: Event) {
        self.pin.wait_for(event).await
    }

    /// Wait until the pin is high.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    pub async fn wait_for_high(&mut self) {
        self.pin.wait_for_high().await
    }

    /// Wait until the pin is low.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    pub async fn wait_for_low(&mut self) {
        self.pin.wait_for_low().await
    }

    /// Wait for the pin to undergo a transition from low to high.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    pub async fn wait_for_rising_edge(&mut self) {
        self.pin.wait_for_rising_edge().await
    }

    /// Wait for the pin to undergo a transition from high to low.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    pub async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await
    }

    /// Wait for the pin to undergo any transition, i.e low to high OR high
    /// to low.
    ///
    /// See [Self::wait_for] for more information.
    #[inline]
    pub async fn wait_for_any_edge(&mut self) {
        self.pin.wait_for_any_edge().await
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct PinFuture<'f, 'd> {
    pin: &'f mut Flex<'d>,
}

impl PinFuture<'_, '_> {
    fn number(&self) -> u8 {
        self.pin.number()
    }

    fn bank(&self) -> GpioBank {
        self.pin.pin.bank()
    }

    fn mask(&self) -> u32 {
        self.pin.pin.mask()
    }

    fn is_done(&self) -> bool {
        // Only the interrupt handler should clear the async bit, and only if the
        // specific pin is handling an interrupt.
        self.bank().async_operations().load(Ordering::Acquire) & self.mask() == 0
    }
}

impl core::future::Future for PinFuture<'_, '_> {
    type Output = ();

    fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        PIN_WAKERS[self.number() as usize].register(cx.waker());

        if self.is_done() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Drop for PinFuture<'_, '_> {
    fn drop(&mut self) {
        // If the pin isn't listening, the future has either been dropped before setup,
        // or the interrupt has already been handled.
        if self.pin.is_listening() {
            // Make sure the future isn't dropped while the interrupt is being handled.
            // This prevents tricky drop-and-relisten scenarios.

            set_int_enable(
                self.number(),
                None, // Do not disable handling pending interrupts.
                0,    // Disable generating new events
                false,
            );

            while self.pin.is_interrupt_set() {}

            // Unmark pin as async
            self.bank()
                .async_operations()
                .fetch_and(!self.mask(), Ordering::Relaxed);
        }
    }
}
