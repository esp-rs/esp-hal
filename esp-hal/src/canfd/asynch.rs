//! Interrupt handling for the async CAN FD driver.
//!
//! Received frames stay in the hardware RX buffer until a task reads them, so
//! the handler only masks the source that fired and wakes the waiting task.

use super::{CanFdInterrupt, Info, ll::Driver};
use crate::asynch::AtomicWaker;

/// Wakers for the tasks waiting on one controller.
#[doc(hidden)]
#[non_exhaustive]
pub struct State {
    pub rx_waker: AtomicWaker,
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

const RX_SOURCES: u32 = CanFdInterrupt::RxNotEmpty.bit();
const TX_SOURCES: u32 = CanFdInterrupt::TxDone.bit();

pub(super) fn handle(info: &Info, state: &State) {
    let driver = Driver::new(info.register_block);
    let status = driver.interrupt_status();

    // The futures re-enable the sources they still need, so a level-triggered
    // source cannot re-enter the handler before the task has run.
    driver.disable_interrupts(status);
    driver.clear_interrupts(status);

    if status & RX_SOURCES != 0 {
        state.rx_waker.wake();
    }
    if status & TX_SOURCES != 0 {
        state.tx_waker.wake();
    }
}
