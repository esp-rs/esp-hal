include!("../../../esp-hal/src/sdio_slave/events.rs");

#[cfg(test)]
mod tests {
    use core::{
        cell::{Cell, RefCell},
        future::Future,
        pin::Pin,
        task::{Context, Poll, Waker},
    };
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };

    use super::*;

    #[derive(Default)]
    struct WakeCount(AtomicUsize);
    impl std::task::Wake for WakeCount {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
        fn wake_by_ref(self: &Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }

    #[derive(Default)]
    struct Source {
        event: Cell<bool>,
        parked: Cell<bool>,
        enabled: Cell<bool>,
        event_during_arm: Cell<bool>,
        failure: Cell<Option<super::super::state::Error>>,
        waker: RefCell<Option<Waker>>,
        polls: usize,
    }

    impl CompletionSource for Source {
        fn register(&self, waker: &Waker) {
            *self.waker.borrow_mut() = Some(waker.clone());
        }
        fn poll_completion(&mut self) -> Result<Option<usize>, super::super::state::Error> {
            self.polls += 1;
            if let Some(error) = self.failure.get() {
                return Err(error);
            }
            Ok((self.event.get() && self.parked.get()).then_some(64))
        }
        fn pending_event(&self) -> bool {
            self.event.get()
        }
        fn arm_notification(&self) {
            self.enabled.set(true);
            if self.event_during_arm.replace(false) {
                self.event.set(true);
                // Model the level ISR: mask, keep RAW, wake the registered task.
                self.enabled.set(false);
                self.waker.borrow().as_ref().unwrap().wake_by_ref();
            }
        }
        fn mask_notification(&self) {
            self.enabled.set(false);
        }
    }

    #[test]
    fn cancellation_masks_notification_but_does_not_complete_or_discard_event() {
        let mut source = Source::default();
        let waker = Waker::noop();
        let mut cx = Context::from_waker(waker);
        {
            let mut wait = Wait::new(&mut source);
            assert_eq!(Pin::new(&mut wait).poll(&mut cx), Poll::Pending);
            assert!(wait.source.enabled.get());
        }
        assert!(!source.enabled.get());
        assert!(!source.event.get());
        // A transfer can finish after the wait was cancelled. A resumed wait
        // observes RAW without requiring the hardware to produce another edge.
        source.event.set(true);
        source.parked.set(true);
        let mut wait = Wait::new(&mut source);
        assert_eq!(Pin::new(&mut wait).poll(&mut cx), Poll::Ready(Ok(64)));
        let polls = wait.source.polls;
        assert_eq!(Pin::new(&mut wait).poll(&mut cx), Poll::Ready(Ok(64)));
        assert_eq!(wait.source.polls, polls);
        assert!(!wait.source.enabled.get());
    }

    #[test]
    fn event_between_raw_check_and_enable_is_not_lost() {
        let mut source = Source::default();
        source.event_during_arm.set(true);
        let count = Arc::new(WakeCount::default());
        let waker = Waker::from(count.clone());
        let mut cx = Context::from_waker(&waker);
        let mut wait = Wait::new(&mut source);
        assert_eq!(Pin::new(&mut wait).poll(&mut cx), Poll::Pending);
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert!(wait.source.event.get());
        assert!(!wait.source.enabled.get());
        // EOF before PARK schedules a retry even though the ISR is masked.
        assert_eq!(Pin::new(&mut wait).poll(&mut cx), Poll::Pending);
        assert_eq!(count.0.load(Ordering::SeqCst), 2);
        wait.source.parked.set(true);
        assert_eq!(Pin::new(&mut wait).poll(&mut cx), Poll::Ready(Ok(64)));
        assert_eq!(count.0.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn a_wait_error_masks_notification_without_another_poll_or_wake() {
        let mut source = Source::default();
        source
            .failure
            .set(Some(super::super::state::Error::DmaFault));
        source.enabled.set(true);
        let count = Arc::new(WakeCount::default());
        let waker = Waker::from(count.clone());
        let mut cx = Context::from_waker(&waker);
        let mut wait = Wait::new(&mut source);
        assert_eq!(
            Pin::new(&mut wait).poll(&mut cx),
            Poll::Ready(Err(super::super::state::Error::DmaFault))
        );
        assert!(!wait.source.enabled.get());
        assert_eq!(count.0.load(Ordering::SeqCst), 0);
        assert_eq!(wait.source.polls, 1);
    }

    #[test]
    fn completion_needs_eof_and_park_and_faults_win() {
        for transmitting in [false, true] {
            let events = if transmitting {
                TRANSMIT_EVENTS
            } else {
                RECEIVE_EVENTS
            };
            let eof = if transmitting { 1 << 17 } else { 1 << 15 };
            assert_eq!(completion(0, transmitting, true), Completion::Pending);
            assert_eq!(completion(eof, transmitting, false), Completion::Draining);
            assert_eq!(completion(eof, transmitting, true), Completion::Complete);
            assert_eq!(completion(events, transmitting, true), Completion::Fault);
            for bit in 0..32 {
                if events & (1 << bit) == 0 {
                    assert_eq!(
                        completion(1 << bit, transmitting, true),
                        Completion::Pending
                    );
                }
            }
        }
    }
}
