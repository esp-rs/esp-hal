//! Low-level [IEEE 802.15.4] driver for the ESP32-C6 and ESP32-H2.
//!
//! Implements the PHY/MAC layers of the IEEE 802.15.4 protocol stack, and
//! supports sending and receiving of raw frames.
//!
//! This library is intended to be used to implement support for higher-level
//! communication protocols, for example [esp-openthread].
//!
//! Note that this crate currently requires you to enable the `unstable` feature
//! on `esp-hal`.
//!
//! [IEEE 802.15.4]: https://en.wikipedia.org/wiki/IEEE_802.15.4
//! [esp-openthread]: https://github.com/esp-rs/esp-openthread
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![no_std]

use core::{cell::RefCell, marker::PhantomData};

use byte::{BytesExt, TryRead};
use critical_section::Mutex;
use esp_config::*;
use esp_hal::peripherals::{IEEE802154, RADIO_CLK};
use heapless::Vec;
use ieee802154::mac::{self, FooterMode, FrameSerDesContext};

use self::{
    frame::FRAME_SIZE,
    pib::{CONFIG_IEEE802154_CCA_THRESHOLD, IEEE802154_FRAME_EXT_ADDR_SIZE},
    raw::*,
};
pub use self::{
    frame::{Frame, ReceivedFrame},
    pib::{CcaMode, PendingMode},
    raw::RawReceived,
};

mod fmt;
mod frame;
mod hal;
mod pib;
mod raw;

/// IEEE 802.15.4 errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The requested data is bigger than available range, and/or the offset is
    /// invalid
    Incomplete,
    /// The requested data content is invalid
    BadInput,
}

impl From<byte::Error> for Error {
    fn from(err: byte::Error) -> Self {
        match err {
            byte::Error::Incomplete | byte::Error::BadOffset(_) => Error::Incomplete,
            byte::Error::BadInput { .. } => Error::BadInput,
        }
    }
}

struct QueueConfig {
    rx_queue_size: usize,
}

pub(crate) const CONFIG: QueueConfig = QueueConfig {
    rx_queue_size: esp_config_int!(usize, "ESP_IEEE802154_CONFIG_RX_QUEUE_SIZE"),
};

/// IEEE 802.15.4 driver configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub auto_ack_tx: bool,
    pub auto_ack_rx: bool,
    pub enhance_ack_tx: bool,
    pub promiscuous: bool,
    pub coordinator: bool,
    pub rx_when_idle: bool,
    pub txpower: i8,
    pub channel: u8,
    pub cca_threshold: i8,
    pub cca_mode: CcaMode,
    pub pan_id: Option<u16>,
    pub short_addr: Option<u16>,
    pub ext_addr: Option<u64>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            auto_ack_tx: Default::default(),
            auto_ack_rx: Default::default(),
            enhance_ack_tx: Default::default(),
            promiscuous: Default::default(),
            coordinator: Default::default(),
            rx_when_idle: Default::default(),
            txpower: 10,
            channel: 15,
            cca_threshold: CONFIG_IEEE802154_CCA_THRESHOLD,
            cca_mode: CcaMode::Ed,
            pan_id: None,
            short_addr: None,
            ext_addr: None,
        }
    }
}

/// IEEE 802.15.4 driver
#[derive(Debug)]
pub struct Ieee802154<'a> {
    _align: u32,
    transmit_buffer: [u8; FRAME_SIZE],
    _phantom1: PhantomData<&'a ()>,
}

impl<'a> Ieee802154<'a> {
    /// Construct a new driver, enabling the IEEE 802.15.4 radio in the process
    pub fn new(_radio: IEEE802154, mut radio_clocks: RADIO_CLK) -> Self {
        esp_ieee802154_enable(&mut radio_clocks);

        Self {
            _align: 0,
            transmit_buffer: [0u8; FRAME_SIZE],
            _phantom1: PhantomData,
        }
    }

    /// Set the configuration for the driver
    pub fn set_config(&mut self, cfg: Config) {
        set_auto_ack_tx(cfg.auto_ack_tx);
        set_auto_ack_rx(cfg.auto_ack_rx);
        set_enhance_ack_tx(cfg.enhance_ack_tx);
        set_promiscuous(cfg.promiscuous);
        set_coordinator(cfg.coordinator);
        set_rx_when_idle(cfg.rx_when_idle);
        set_tx_power(cfg.txpower);
        set_channel(cfg.channel);
        set_cca_theshold(cfg.cca_threshold);
        set_cca_mode(cfg.cca_mode);

        if let Some(pan_id) = cfg.pan_id {
            set_panid(0, pan_id);
        }

        if let Some(short_addr) = cfg.short_addr {
            set_short_address(0, short_addr);
        }

        if let Some(ext_addr) = cfg.ext_addr {
            let mut address = [0u8; IEEE802154_FRAME_EXT_ADDR_SIZE];
            address.copy_from_slice(&ext_addr.to_be_bytes()); // LE or BE?

            set_extended_address(0, address);
        }
    }

    /// Start receiving frames
    pub fn start_receive(&mut self) {
        ieee802154_receive();
    }

    /// Return the raw data of a received frame
    pub fn raw_received(&mut self) -> Option<RawReceived> {
        ieee802154_poll()
    }

    /// Get a received frame, if available
    pub fn received(&mut self) -> Option<Result<ReceivedFrame, Error>> {
        if let Some(raw) = ieee802154_poll() {
            let maybe_decoded = if raw.data[0] as usize > raw.data.len() {
                // try to decode up to data.len()
                mac::Frame::try_read(&raw.data[1..][..raw.data.len()], FooterMode::Explicit)
            } else {
                mac::Frame::try_read(&raw.data[1..][..raw.data[0] as usize], FooterMode::Explicit)
            };

            let result = match maybe_decoded {
                Ok((decoded, _)) => {
                    // crc is not written to rx buffer
                    let rssi = if (raw.data[0] as usize > raw.data.len()) || (raw.data[0] == 0) {
                        raw.data[raw.data.len() - 1] as i8
                    } else {
                        raw.data[raw.data[0] as usize - 1] as i8
                    };

                    Ok(ReceivedFrame {
                        frame: Frame {
                            header: decoded.header,
                            content: decoded.content,
                            payload: Vec::from_slice(decoded.payload).unwrap(),
                            footer: decoded.footer,
                        },
                        channel: raw.channel,
                        rssi,
                        lqi: rssi_to_lqi(rssi),
                    })
                }
                Err(err) => Err(err.into()),
            };

            Some(result)
        } else {
            None
        }
    }

    /// Transmit a frame
    pub fn transmit(&mut self, frame: &Frame) -> Result<(), Error> {
        let frm = mac::Frame {
            header: frame.header,
            content: frame.content,
            payload: &frame.payload,
            footer: frame.footer,
        };

        let mut offset = 1usize;
        self.transmit_buffer
            .write_with(
                &mut offset,
                frm,
                &mut FrameSerDesContext::no_security(FooterMode::Explicit),
            )
            .unwrap();
        self.transmit_buffer[0] = (offset - 1) as u8;

        ieee802154_transmit(self.transmit_buffer.as_ptr(), false); // what about CCA?

        Ok(())
    }

    /// Transmit a raw frame
    pub fn transmit_raw(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.transmit_buffer[1..][..frame.len()].copy_from_slice(frame);
        self.transmit_buffer[0] = frame.len() as u8;

        ieee802154_transmit(self.transmit_buffer.as_ptr(), false); // what about CCA?

        Ok(())
    }

    /// Set the transmit done callback function.
    pub fn set_tx_done_callback(&mut self, callback: &'a mut (dyn FnMut() + Send)) {
        critical_section::with(|cs| {
            let mut tx_done_callback = TX_DONE_CALLBACK.borrow_ref_mut(cs);
            let cb: &'static mut (dyn FnMut() + Send) = unsafe { core::mem::transmute(callback) };
            tx_done_callback.replace(cb);
        });
    }

    /// Clear the transmit done callback function.
    pub fn clear_tx_done_callback(&mut self) {
        critical_section::with(|cs| {
            let mut tx_done_callback = TX_DONE_CALLBACK.borrow_ref_mut(cs);
            tx_done_callback.take();
        });
    }

    /// Set the receive available callback function.
    pub fn set_rx_available_callback(&mut self, callback: &'a mut (dyn FnMut() + Send)) {
        critical_section::with(|cs| {
            let mut rx_available_callback = RX_AVAILABLE_CALLBACK.borrow_ref_mut(cs);
            let cb: &'static mut (dyn FnMut() + Send) = unsafe { core::mem::transmute(callback) };
            rx_available_callback.replace(cb);
        });
    }

    /// Clear the receive available callback function.
    pub fn clear_rx_available_callback(&mut self) {
        critical_section::with(|cs| {
            let mut rx_available_callback = RX_AVAILABLE_CALLBACK.borrow_ref_mut(cs);
            rx_available_callback.take();
        });
    }

    /// Set the transmit done callback function.
    pub fn set_tx_done_callback_fn(&mut self, callback: fn()) {
        critical_section::with(|cs| {
            let mut tx_done_callback_fn = TX_DONE_CALLBACK_FN.borrow_ref_mut(cs);
            tx_done_callback_fn.replace(callback);
        });
    }

    /// Clear the transmit done callback function.
    pub fn clear_tx_done_callback_fn(&mut self) {
        critical_section::with(|cs| {
            let mut tx_done_callback_fn = TX_DONE_CALLBACK_FN.borrow_ref_mut(cs);
            tx_done_callback_fn.take();
        });
    }

    /// Set the receive available callback function.
    pub fn set_rx_available_callback_fn(&mut self, callback: fn()) {
        critical_section::with(|cs| {
            let mut rx_available_callback_fn = RX_AVAILABLE_CALLBACK_FN.borrow_ref_mut(cs);
            rx_available_callback_fn.replace(callback);
        });
    }

    /// Clear the receive available callback function.
    pub fn clear_rx_available_callback_fn(&mut self) {
        critical_section::with(|cs| {
            let mut rx_available_callback_fn = RX_AVAILABLE_CALLBACK_FN.borrow_ref_mut(cs);
            rx_available_callback_fn.take();
        });
    }
}

impl Drop for Ieee802154<'_> {
    fn drop(&mut self) {
        self.clear_tx_done_callback();
        self.clear_tx_done_callback_fn();
        self.clear_rx_available_callback();
        self.clear_rx_available_callback_fn();
    }
}

/// Convert from RSSI (Received Signal Strength Indicator) to LQI (Link Quality
/// Indication)
///
/// RSSI is a measure of incoherent (raw) RF power in a channel. LQI is a
/// cumulative value used in multi-hop networks to assess the cost of a link.
pub fn rssi_to_lqi(rssi: i8) -> u8 {
    if rssi < -80 {
        0
    } else if rssi > -30 {
        0xff
    } else {
        let lqi_convert = ((rssi as u32).wrapping_add(80)) * 255;
        (lqi_convert / 50) as u8
    }
}

static TX_DONE_CALLBACK: Mutex<RefCell<Option<&'static mut (dyn FnMut() + Send)>>> =
    Mutex::new(RefCell::new(None));

static RX_AVAILABLE_CALLBACK: Mutex<RefCell<Option<&'static mut (dyn FnMut() + Send)>>> =
    Mutex::new(RefCell::new(None));

#[allow(clippy::type_complexity)]
static TX_DONE_CALLBACK_FN: Mutex<RefCell<Option<fn()>>> = Mutex::new(RefCell::new(None));

#[allow(clippy::type_complexity)]
static RX_AVAILABLE_CALLBACK_FN: Mutex<RefCell<Option<fn()>>> = Mutex::new(RefCell::new(None));

fn tx_done() {
    trace!("tx_done callback");

    critical_section::with(|cs| {
        let mut tx_done_callback = TX_DONE_CALLBACK.borrow_ref_mut(cs);
        let tx_done_callback = tx_done_callback.as_mut();

        if let Some(tx_done_callback) = tx_done_callback {
            tx_done_callback();
        }

        let mut tx_done_callback_fn = TX_DONE_CALLBACK_FN.borrow_ref_mut(cs);
        let tx_done_callback_fn = tx_done_callback_fn.as_mut();

        if let Some(tx_done_callback_fn) = tx_done_callback_fn {
            tx_done_callback_fn();
        }
    });
}

fn rx_available() {
    trace!("rx available callback");

    critical_section::with(|cs| {
        let mut rx_available_callback = RX_AVAILABLE_CALLBACK.borrow_ref_mut(cs);
        let rx_available_callback = rx_available_callback.as_mut();

        if let Some(rx_available_callback) = rx_available_callback {
            rx_available_callback();
        }

        let mut rx_available_callback_fn = RX_AVAILABLE_CALLBACK_FN.borrow_ref_mut(cs);
        let rx_available_callback_fn = rx_available_callback_fn.as_mut();

        if let Some(rx_available_callback_fn) = rx_available_callback_fn {
            rx_available_callback_fn();
        }
    });
}
