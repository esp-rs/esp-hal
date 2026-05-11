//! `embassy-net` driver integration for the EMAC Ethernet peripheral.
//!
//! This module provides an [`embassy_net_driver_02::Driver`] implementation
//! for [`Ethernet`] operating in async mode, enabling the
//! Ethernet peripheral to be used as a network interface with
//! [`embassy-net`](https://crates.io/crates/embassy-net).
//!
//! # Usage
//!
//! After obtaining an `Ethernet<'_, Async, P>` instance, pass it directly to
//! `embassy_net::new()` — the [`Driver`] impl is inherent on the type.

use core::task::Context;

use embassy_net_driver_02::{Capabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken};

use super::{Ethernet, RX_WAKER, TX_WAKER, mac::EmacRegs};
use crate::{
    Async,
    ethernet::{
        dma::{RDesRing, TDesRing},
        phy::Phy,
    },
};

/// Maximum Ethernet frame size (header + payload, no FCS).
const MTU: usize = 1514;

// ── Token types ───────────────────────────────────────────────────────────────

/// Received-frame token.
///
/// Holds a mutable borrow of the RX ring and a shared borrow of the MAC
/// register block needed to resume the RX DMA after releasing the descriptor.
/// The closure passed to [`consume`][RxToken::consume] receives a `&mut [u8]`
/// pointing directly into the DMA RX buffer — no copy is performed.
pub struct EthernetRxToken<'a, 'd> {
    rx: &'a mut RDesRing<'d>,
}

/// Transmit token.
///
/// Holds a mutable borrow of the TX ring and a shared borrow of the MAC
/// register block needed to trigger a TX poll after committing the frame.
/// The closure passed to [`consume`][TxToken::consume] receives a `&mut [u8]`
/// pointing directly into the DMA TX buffer — no copy is performed.
pub struct EthernetTxToken<'a, 'd> {
    tx: &'a mut TDesRing<'d>,
}

impl<'a, 'd> RxToken for EthernetRxToken<'a, 'd> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        // receive() loops past error frames and returns a direct reference into
        // the DMA buffer. The descriptor is still CPU-owned while f runs.
        // NOTE: unwrap is safe — Driver::receive() verified a valid frame exists
        // and we hold exclusive access to the ring via &'a mut.
        let pkt = unwrap!(self.rx.receive(), "RX packet vanished");
        let r = f(pkt);
        // After f returns, the &mut [u8] borrow on the ring ends (NLL),
        // so we can recycle the descriptor.
        self.rx.pop();
        // Poke the RX DMA in case it suspended waiting for a CPU-owned descriptor.
        EmacRegs.demand_rx_poll();
        r
    }
}

impl<'a, 'd> TxToken for EthernetTxToken<'a, 'd> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let capped = len.min(MTU);
        // Get a direct mutable reference into the DMA TX buffer — no copy.
        // NOTE: unwrap is safe — Driver::transmit/receive() verified capacity.
        let buf = unwrap!(self.tx.available_buf(), "TX slot vanished");
        let r = f(&mut buf[..capped]);
        // After f returns the &mut borrow on buf ends, we can commit.
        self.tx.commit(capped);
        EmacRegs.demand_tx_poll();
        r
    }
}

// ── Driver impl ───────────────────────────────────────────────────────────────

impl<'d, P: Phy> Driver for Ethernet<'d, Async, P> {
    type RxToken<'a>
        = EthernetRxToken<'a, 'd>
    where
        Self: 'a;

    type TxToken<'a>
        = EthernetTxToken<'a, 'd>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context<'_>) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        RX_WAKER.register(cx.waker());
        TX_WAKER.register(cx.waker());

        // Check availability as plain booleans so the borrows end before we
        // split &mut self into the two ring references for the tokens.
        let rx_ready = self.rx.receive().is_some();
        // Poke the RX DMA unconditionally: receive() may have recycled error
        // frames back to DMA ownership without a poll-demand write, which
        // would leave the GMAC RX channel suspended.
        EmacRegs.demand_rx_poll();
        let tx_ready = self.tx.available_buf().is_some();

        if rx_ready && tx_ready {
            Some((
                EthernetRxToken { rx: &mut self.rx },
                EthernetTxToken { tx: &mut self.tx },
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, cx: &mut Context<'_>) -> Option<Self::TxToken<'_>> {
        TX_WAKER.register(cx.waker());
        if self.tx.available_buf().is_some() {
            Some(EthernetTxToken { tx: &mut self.tx })
        } else {
            None
        }
    }

    fn link_state(&mut self, cx: &mut Context<'_>) -> LinkState {
        let state = self.poll_link(Some(cx));
        if state.up {
            self.set_speed(state.speed);
            self.set_duplex(state.duplex);
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = MTU;
        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac_addr())
    }
}
