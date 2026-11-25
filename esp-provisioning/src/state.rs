//! Pure `no_std` multi-attempt provisioning state machine.
//!
//! Separated from [`crate::session`] so every state transition compiles and is
//! unit-tested on the host under `--no-default-features`.

/// Why a station join or DHCP verification attempt failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum StaFailReason {
    /// Wrong passphrase, incompatible security mode, handshake failure, or DHCP timeout.
    AuthError,
    /// Access point SSID was not found.
    NetworkNotFound,
}

/// Phase of the multi-attempt provisioning lifecycle.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Phase<C, I> {
    /// Waiting for `SetConfig` + `ApplyConfig` (or reset after a failed attempt).
    Idle,
    /// Credentials submitted by `ApplyConfig`; waiting for the application to pick them up.
    Pending(C),
    /// Application took the credentials and is joining the AP / acquiring DHCP.
    Connecting,
    /// Join or DHCP failed; waiting for `prov-ctrl` `CtrlReset` to return to [`Self::Idle`].
    Failed(StaFailReason),
    /// Station associated and acquired an IPv4 address.
    Connected(I),
}

/// Pure state transitions backing [`crate::Session`].
#[derive(Debug)]
pub(crate) struct SessionCore<C, I> {
    phase: Phase<C, I>,
}

impl<C, I> Default for SessionCore<C, I> {
    fn default() -> Self {
        Self::new()
    }
}

impl<C, I> SessionCore<C, I> {
    pub(crate) const fn new() -> Self {
        Self { phase: Phase::Idle }
    }

    pub(crate) fn phase(&self) -> &Phase<C, I> {
        &self.phase
    }

    /// Accepts credentials only in [`Phase::Idle`].
    pub(crate) fn submit(&mut self, creds: C) -> bool {
        if matches!(self.phase, Phase::Idle) {
            self.phase = Phase::Pending(creds);
            true
        } else {
            false
        }
    }

    /// Transitions [`Phase::Pending`] to [`Phase::Connecting`] and yields the credentials.
    pub(crate) fn take_pending(&mut self) -> Option<C> {
        if matches!(self.phase, Phase::Pending(_)) {
            let prev = core::mem::replace(&mut self.phase, Phase::Connecting);
            match prev {
                Phase::Pending(creds) => Some(creds),
                _ => unreachable!(),
            }
        } else {
            None
        }
    }

    pub(crate) fn report_connecting(&mut self) {
        if !matches!(self.phase, Phase::Connected(_)) {
            self.phase = Phase::Connecting;
        }
    }

    pub(crate) fn report_failed(&mut self, reason: StaFailReason) {
        self.phase = Phase::Failed(reason);
    }

    pub(crate) fn report_connected(&mut self, info: I) {
        self.phase = Phase::Connected(info);
    }

    /// Resets [`Phase::Failed`] (or idempotently [`Phase::Idle`]) back to [`Phase::Idle`].
    ///
    /// Returns `false` in [`Phase::Pending`], [`Phase::Connecting`], or [`Phase::Connected`].
    pub(crate) fn reset_on_failure(&mut self) -> bool {
        match self.phase {
            Phase::Failed(_) | Phase::Idle => {
                self.phase = Phase::Idle;
                true
            }
            Phase::Pending(_) | Phase::Connecting | Phase::Connected(_) => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn multi_attempt_failure_reset_and_success() {
        let mut core = SessionCore::<&str, u32>::new();
        assert_eq!(core.phase(), &Phase::Idle);
        assert!(core.take_pending().is_none());

        // First attempt: submit -> take -> fail.
        assert!(core.submit("wrong-pass"));
        assert!(!core.submit("second-ignored"));
        assert_eq!(core.take_pending(), Some("wrong-pass"));
        assert_eq!(core.phase(), &Phase::Connecting);
        core.report_connecting();
        assert_eq!(core.phase(), &Phase::Connecting);
        assert!(core.take_pending().is_none());

        // CtrlReset rejected while Connecting.
        assert!(!core.reset_on_failure());
        assert_eq!(core.phase(), &Phase::Connecting);

        core.report_failed(StaFailReason::AuthError);
        assert_eq!(core.phase(), &Phase::Failed(StaFailReason::AuthError));
        // ApplyConfig rejected while Failed until CtrlReset arrives.
        assert!(!core.submit("before-reset"));
        assert!(core.take_pending().is_none());

        // CtrlReset in Failed returns to Idle.
        assert!(core.reset_on_failure());
        assert_eq!(core.phase(), &Phase::Idle);
        // CtrlReset in Idle is idempotent.
        assert!(core.reset_on_failure());
        assert_eq!(core.phase(), &Phase::Idle);

        // Second attempt: submit -> take -> connect.
        assert!(core.submit("good-pass"));
        // CtrlReset rejected while Pending.
        assert!(!core.reset_on_failure());
        assert_eq!(core.take_pending(), Some("good-pass"));
        core.report_connected(42);
        assert_eq!(core.phase(), &Phase::Connected(42));

        // CtrlReset and submit rejected once Connected.
        assert!(!core.reset_on_failure());
        assert!(!core.submit("after-connected"));
        assert_eq!(core.phase(), &Phase::Connected(42));
    }
}
