//! Low-level [IEEE 802.15.4] driver for the ESP32-C6 and ESP32-H2.
//!
//! Implements the PHY/MAC layers of the IEEE 802.15.4 protocol stack, and
//! supports sending and receiving of raw frames.
//!
//! This module is intended to be used to implement support for higher-level
//! communication protocols, for example [esp-openthread].
//!
//! Note that this module currently requires you to enable the `unstable` feature
//! on `esp-hal`.
//!
//! NOTE: Coexistence with Wi-Fi or Bluetooth is currently not possible. If you do it anyway,
//! things will break.
//!
//! [IEEE 802.15.4]: https://en.wikipedia.org/wiki/IEEE_802.15.4
//! [esp-openthread]: https://github.com/esp-rs/esp-openthread

use byte::{BytesExt, TryRead};
use esp_hal::{clock::PhyClockGuard, peripherals::IEEE802154};
use esp_phy::PhyInitGuard;
use esp_sync::NonReentrantMutex;
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

mod frame;
mod hal;
mod pib;
mod raw;

/// IEEE 802.15.4 errors
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The requested data is bigger than available range, and/or the offset is
    /// invalid.
    Incomplete,

    /// The requested data content is invalid.
    BadInput,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Incomplete => write!(f, "Incomplete data."),
            Error::BadInput => write!(f, "Bad input data."),
        }
    }
}

impl core::error::Error for Error {}

impl From<byte::Error> for Error {
    fn from(err: byte::Error) -> Self {
        match err {
            byte::Error::Incomplete | byte::Error::BadOffset(_) => Error::Incomplete,
            byte::Error::BadInput { .. } => Error::BadInput,
        }
    }
}

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
    pub rx_queue_size: usize,
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
            rx_queue_size: 10,
        }
    }
}

/// IEEE 802.15.4 driver
#[derive(Debug)]
pub struct Ieee802154<'a> {
    _align: u32,
    transmit_buffer: [u8; FRAME_SIZE],
    _phy_clock_guard: PhyClockGuard<'a>,
    _phy_init_guard: PhyInitGuard<'a>,
}

impl<'a> Ieee802154<'a> {
    /// Construct a new driver, enabling the IEEE 802.15.4 radio in the process
    ///
    /// NOTE: Coexistence with Wi-Fi or Bluetooth is currently not possible. If you do it anyway,
    /// things will break.
    pub fn new(radio: IEEE802154<'a>) -> Self {
        let (_phy_clock_guard, _phy_init_guard) = esp_ieee802154_enable(radio);
        Self {
            _align: 0,
            transmit_buffer: [0u8; FRAME_SIZE],
            _phy_clock_guard,
            _phy_init_guard,
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

        raw::set_queue_size(cfg.rx_queue_size);
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
                            payload: decoded.payload.to_vec(),
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
        CALLBACKS.with(|cbs| {
            let cb: &'static mut (dyn FnMut() + Send) = unsafe { core::mem::transmute(callback) };
            cbs.tx_done = Some(cb);
        });
    }

    /// Clear the transmit done callback function.
    pub fn clear_tx_done_callback(&mut self) {
        CALLBACKS.with(|cbs| cbs.tx_done = None);
    }

    /// Set the receive available callback function.
    pub fn set_rx_available_callback(&mut self, callback: &'a mut (dyn FnMut() + Send)) {
        CALLBACKS.with(|cbs| {
            let cb: &'static mut (dyn FnMut() + Send) = unsafe { core::mem::transmute(callback) };
            cbs.rx_available = Some(cb);
        });
    }

    /// Clear the receive available callback function.
    pub fn clear_rx_available_callback(&mut self) {
        CALLBACKS.with(|cbs| cbs.rx_available = None);
    }

    /// Set the transmit done callback function.
    pub fn set_tx_done_callback_fn(&mut self, callback: fn()) {
        CALLBACKS.with(|cbs| cbs.tx_done_fn = Some(callback));
    }

    /// Clear the transmit done callback function.
    pub fn clear_tx_done_callback_fn(&mut self) {
        CALLBACKS.with(|cbs| cbs.tx_done_fn = None);
    }

    /// Set the receive available callback function.
    pub fn set_rx_available_callback_fn(&mut self, callback: fn()) {
        CALLBACKS.with(|cbs| cbs.rx_available_fn = Some(callback));
    }

    /// Clear the receive available callback function.
    pub fn clear_rx_available_callback_fn(&mut self) {
        CALLBACKS.with(|cbs| cbs.rx_available_fn = None);
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

struct Callbacks {
    tx_done: Option<&'static mut (dyn FnMut() + Send)>,
    rx_available: Option<&'static mut (dyn FnMut() + Send)>,
    // TODO: remove these - Box<dyn FnMut> should be good enough
    tx_done_fn: Option<fn()>,
    rx_available_fn: Option<fn()>,
}

impl Callbacks {
    fn call_tx_done(&mut self) {
        if let Some(cb) = self.tx_done.as_mut() {
            cb();
        }
        if let Some(cb) = self.tx_done_fn.as_mut() {
            cb();
        }
    }

    fn call_rx_available(&mut self) {
        if let Some(cb) = self.rx_available.as_mut() {
            cb();
        }
        if let Some(cb) = self.rx_available_fn.as_mut() {
            cb();
        }
    }
}

static CALLBACKS: NonReentrantMutex<Callbacks> = NonReentrantMutex::new(Callbacks {
    tx_done: None,
    rx_available: None,
    tx_done_fn: None,
    rx_available_fn: None,
});

fn tx_done() {
    trace!("tx_done callback");

    CALLBACKS.with(|cbs| cbs.call_tx_done());
}

fn rx_available() {
    trace!("rx available callback");

    CALLBACKS.with(|cbs| cbs.call_rx_available());
}
