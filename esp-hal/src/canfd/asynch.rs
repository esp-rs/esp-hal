//! Interrupt handling for the async CAN FD driver.
//!
//! Received frames stay in the hardware RX buffer until a task reads them. The
//! handler therefore only masks the interrupt that fired and wakes the waiting
//! task, rather than draining frames into a software queue. That keeps the
//! driver free of a fixed-size queue and of a policy for what to drop when such
//! a queue overflows: once the hardware RX buffer is full, no software buffer
//! could have helped anyway.

use embassy_sync::waitqueue::AtomicWaker;

use super::{CanFdInterrupt, Info, ll::Ll};

/// Wakers for the tasks waiting on one controller.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    /// Woken when a frame arrives.
    pub rx_waker: AtomicWaker,
    /// Woken when a TX buffer finishes.
    pub tx_waker: AtomicWaker,
}

impl State {
    pub(super) const fn new() -> Self {
        Self {
            rx_waker: AtomicWaker::new(),
            tx_waker: AtomicWaker::new(),
        }
    }
}

/// Interrupt sources that wake a receiving task.
const RX_SOURCES: u32 = CanFdInterrupt::RxNotEmpty.bit();

/// Interrupt sources that wake a transmitting task.
const TX_SOURCES: u32 = CanFdInterrupt::TxDone.bit();

pub(super) fn handle(info: &Info, state: &State) {
    let ll = Ll::new(info.register_block);
    let status = ll.interrupt_status();

    // Disable whatever fired before waking. The futures re-enable the sources
    // they still need, so a level-triggered source cannot re-enter the handler
    // in a loop while the task has not run yet.
    ll.disable_interrupts(status);
    ll.clear_interrupts(status);

    if status & RX_SOURCES != 0 {
        state.rx_waker.wake();
    }
    if status & TX_SOURCES != 0 {
        state.tx_waker.wake();
    }
}
