//! `xarxa` driver integration for the EMAC Ethernet peripheral.
//!
//! This module provides a [`xarxa_driver::Driver`] implementation for
//! [`Ethernet`] operating in async mode, enabling the Ethernet peripheral to be
//! used as a network interface with [`xarxa`](https://crates.io/crates/xarxa).
//!
//! ## Usage
//!
//! After obtaining an `Ethernet<'_, Async, P>` instance, pass it directly to the
//! stack — the [`Driver`] impl is inherent on the type.
//!
//! Unlike the `embassy-net` driver, this one copies each frame between the DMA
//! ring and a pool buffer, because `xarxa` passes ownership of packet buffers
//! instead of lending the driver's own memory.

use core::{
    any::TypeId,
    task::{Context, Waker},
};

use xarxa_driver::{
    Capabilities,
    ChecksumOffload,
    Driver,
    HardwareAddress,
    LinkState,
    NotSupported,
    PacketBuf,
    config::PACKET_BUF_SIZE,
};

use super::{Ethernet, RX_WAKER, TX_WAKER, mac::EmacRegs};
use crate::{Async, DriverMode, ethernet::phy::Phy};

/// Maximum Ethernet frame size (header + payload, no FCS).
///
/// The packet pool has a fixed buffer size, which caps the frames this driver
/// can pass on to the stack.
const MTU: usize = if 1514 < PACKET_BUF_SIZE {
    1514
} else {
    PACKET_BUF_SIZE
};

impl<'d, Dm: DriverMode + 'static, P: Phy> Driver for Ethernet<'d, Dm, P> {
    fn capabilities(&self) -> Capabilities {
        let mut caps = Capabilities::default();
        caps.max_transmission_unit = MTU;
        // `Checksum` names the direction(s) computed in software. Hardware
        // offload is advertised so the stack skips that side.
        let checksum = ChecksumOffload {
            tx: property!("ethernet.tx_checksum_offload"),
            rx: property!("ethernet.rx_checksum_offload"),
        };
        caps.checksum.ipv4 = checksum;
        caps.checksum.tcp = checksum;
        caps.checksum.udp = checksum;
        caps.checksum.icmpv4 = checksum;
        caps.checksum.icmpv6 = checksum;
        caps
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac_addr())
    }

    fn link_state(&mut self) -> LinkState {
        let state = self.poll_link(None);
        if state.up {
            self.set_speed(state.speed);
            self.set_duplex(state.duplex);
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    fn register_waker(&mut self, waker: &Waker) -> Result<(), NotSupported> {
        if TypeId::of::<Dm>() == TypeId::of::<Async>() {
            // The driver has one waker per event, the stack has one waker for all of them.
            RX_WAKER.register(waker);
            TX_WAKER.register(waker);
            // The PHY keeps the waker to report a link change. A PHY that cannot do so wakes the
            // stack at once, which makes it poll the link state instead.
            self.poll_link(Some(&mut Context::from_waker(waker)));
            Ok(())
        } else {
            Err(NotSupported)
        }
    }

    fn receive(&mut self) -> Option<PacketBuf> {
        let Some(frame) = self.rx.receive() else {
            // receive() may have recycled error frames back to DMA ownership without a
            // poll-demand write, which would leave the GMAC RX channel suspended.
            EmacRegs.demand_rx_poll();
            return None;
        };

        let buffer = if frame.len() > PACKET_BUF_SIZE {
            warn!(
                "Dropping a {} byte frame, the packet buffer holds {} bytes",
                frame.len(),
                PACKET_BUF_SIZE
            );
            None
        } else if let Some(mut buffer) = PacketBuf::try_new() {
            buffer.set_len(frame.len());
            buffer.copy_from_slice(frame);
            Some(buffer)
        } else {
            // The pool is shared with the rest of the stack and can be empty. Dropping the
            // frame is the only option, the descriptor must go back to the DMA.
            None
        };

        self.rx.pop();
        EmacRegs.demand_rx_poll();

        buffer
    }

    fn can_transmit(&mut self) -> bool {
        self.tx.available_buf().is_some()
    }

    fn transmit(&mut self, buffer: PacketBuf) -> Result<(), PacketBuf> {
        let Some(slot) = self.tx.available_buf() else {
            return Err(buffer);
        };

        let len = buffer.len().min(MTU);
        slot[..len].copy_from_slice(&buffer[..len]);

        self.tx.commit(len);
        EmacRegs.demand_tx_poll();

        Ok(())
    }
}
