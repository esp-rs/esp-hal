use core::{
    sync::atomic::Ordering,
    task::{Context, Poll},
};

use procmacros::ram;

use crate::{
    asynch::AtomicWaker,
    gpio::{Event, Flex, GpioBank, Input, NUM_PINS},
};

#[ram]
pub(super) static PIN_WAKERS: [AtomicWaker; NUM_PINS] = [const { AtomicWaker::new() }; NUM_PINS];

impl Flex<'_> {
    /// Wait until the pin experiences a particular [`Event`].
    ///
    /// The GPIO driver will disable listening for the event once it occurs,
    /// or if the `Future` is dropped - which also means this method is **not**
    /// cancellation-safe, it will always wait for a future event.
    ///
    /// Note that calling this function will overwrite previous
    /// [`listen`][Self::listen] operations for this pin.
    #[inline]
    #[instability::unstable]
    pub async fn wait_for(&mut self, event: Event) {
        // Make sure this pin is not being processed by an interrupt handler. We need to
        // always take a critical section even if the pin is not listening, because the
        // interrupt handler may be running on another core and the interrupt handler
        // may be in the process of processing the pin if the interrupt status is set -
        // regardless of the pin actually listening or not.
        if self.is_listening() || self.is_interrupt_set() {
            self.unlisten_and_clear();
        }

        // At this point the pin is no longer listening, and not being processed, so we
        // can safely do our setup.

        // Mark pin as async. The interrupt handler clears this bit before processing a
        // pin and unlistens it, so this call will not race with the interrupt
        // handler (because it must have finished before `unlisten` above, or the
        // handler no longer )
        self.pin
            .bank()
            .async_operations()
            .fetch_or(self.pin.mask(), Ordering::Relaxed);

        // Start listening for the event. We only need to do this once, as disabling
        // the interrupt will signal the future to complete.
        self.listen(event);

        PinFuture { pin: self }.await
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
    /// or if the `Future` is dropped - which also means this method is **not**
    /// cancellation-safe, it will always wait for a future event.
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
        // specific pin is handling an interrupt. This way the user may clear the
        // interrupt status without worrying about the async bit being cleared.
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
        // If the future has completed, unlistening and removing the async bit will have
        // been done by the interrupt handler.

        if !self.is_done() {
            self.pin.unlisten_and_clear();

            // Unmark pin as async so that a future listen call doesn't wake a waker for no
            // reason.
            self.bank()
                .async_operations()
                .fetch_and(!self.mask(), Ordering::Relaxed);
        }
    }
}
