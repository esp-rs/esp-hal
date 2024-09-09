use core::cell::RefCell;

use critical_section::Mutex;

use crate::hal::{
    set_cca_mode,
    set_cca_threshold,
    set_coordinator,
    set_freq,
    set_multipan_enable_mask,
    set_multipan_ext_addr,
    set_multipan_panid,
    set_multipan_short_addr,
    set_pending_mode,
    set_power,
    set_promiscuous,
    set_rx_auto_ack,
    set_tx_auto_ack,
    set_tx_enhance_ack,
};

pub(crate) const CONFIG_IEEE802154_CCA_THRESHOLD: i8 = -60;
pub(crate) const IEEE802154_FRAME_EXT_ADDR_SIZE: usize = 8;

const IEEE802154_MULTIPAN_0: u8 = 0;
const IEEE802154_MULTIPAN_MAX: usize = 4;

static PIB: Mutex<RefCell<Option<Pib>>> = Mutex::new(RefCell::new(None));

/// Frame pending mode
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum PendingMode {
    /// Frame pending bit always set to 1 in the ack to Data Request
    #[default]
    Disable  = 0,
    /// Frame pending bit set to 1 if src address matches, in the ack to Data
    /// Request
    Enable   = 1,
    /// Frame pending bit set to 1 if src address matches, in all ack frames
    Enhanced = 2,
    /// Frame pending bit set to 0 only if src address is short address and
    /// matches in table, in the ack to Data Request
    Zigbee   = 3,
}

/// CCA mode
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum CcaMode {
    /// Carrier only
    #[default]
    Carrier      = 0x00,
    /// Energy Detect only
    Ed           = 0x01,
    /// Carrier or Energy Detect
    CarrierOrEd  = 0x02,
    /// Carrier and Energy Detect
    CarrierAndEd = 0x03,
}

#[derive(Debug, Default, Clone, Copy)]
struct Pib {
    auto_ack_tx: bool,
    auto_ack_rx: bool,
    enhance_ack_tx: bool,
    promiscuous: bool,
    coordinator: bool,
    rx_when_idle: bool,
    txpower: i8,
    channel: u8,
    pending_mode: PendingMode,
    multipan_mask: u8,
    panid: [u16; IEEE802154_MULTIPAN_MAX],
    short_addr: [u16; IEEE802154_MULTIPAN_MAX],
    ext_addr: [[u8; IEEE802154_FRAME_EXT_ADDR_SIZE]; IEEE802154_MULTIPAN_MAX],
    cca_threshold: i8,
    cca_mode: CcaMode,
}

pub(crate) fn ieee802154_pib_init() {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).replace(Pib {
            auto_ack_tx: true,
            auto_ack_rx: true,
            enhance_ack_tx: true,
            coordinator: false,
            promiscuous: true,
            rx_when_idle: false,
            txpower: 10,
            channel: 11,
            pending_mode: PendingMode::Disable,
            multipan_mask: 1 << IEEE802154_MULTIPAN_0,
            panid: [0u16; 4],
            short_addr: [0u16; IEEE802154_MULTIPAN_MAX],
            ext_addr: [[0xffu8; IEEE802154_FRAME_EXT_ADDR_SIZE]; IEEE802154_MULTIPAN_MAX],
            cca_threshold: CONFIG_IEEE802154_CCA_THRESHOLD,
            cca_mode: CcaMode::Ed,
        });
    });
}

pub(crate) fn ieee802154_pib_set_panid(index: u8, panid: u16) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().panid[index as usize] = panid;
    });
}

pub(crate) fn ieee802154_pib_set_promiscuous(enable: bool) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().promiscuous = enable;
    });
}

pub(crate) fn ieee802154_pib_set_auto_ack_tx(enable: bool) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().auto_ack_tx = enable;
    });
}

pub(crate) fn ieee802154_pib_set_auto_ack_rx(enable: bool) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().auto_ack_rx = enable;
    });
}

pub(crate) fn ieee802154_pib_set_enhance_ack_tx(enable: bool) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().enhance_ack_tx = enable;
    });
}

pub(crate) fn ieee802154_pib_set_coordinator(enable: bool) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().coordinator = enable;
    });
}

pub(crate) fn ieee802154_pib_set_rx_when_idle(enable: bool) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().rx_when_idle = enable;
    });
}

pub(crate) fn ieee802154_pib_get_rx_when_idle() -> bool {
    critical_section::with(|cs| PIB.borrow_ref_mut(cs).as_mut().unwrap().rx_when_idle)
}

pub(crate) fn ieee802154_pib_set_tx_power(power: i8) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().txpower = power;
    });
}

pub(crate) fn ieee802154_pib_set_channel(channel: u8) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().channel = channel;
    });
}

pub(crate) fn ieee802154_pib_set_pending_mode(mode: PendingMode) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().pending_mode = mode;
    });
}

pub(crate) fn ieee802154_pib_set_short_address(index: u8, address: u16) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().short_addr[index as usize] = address;
    });
}

pub(crate) fn ieee802154_pib_set_extended_address(
    index: u8,
    address: [u8; IEEE802154_FRAME_EXT_ADDR_SIZE],
) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().ext_addr[index as usize] = address;
    });
}

pub(crate) fn ieee802154_pib_set_cca_theshold(cca_threshold: i8) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().cca_threshold = cca_threshold;
    });
}

pub(crate) fn ieee802154_pib_set_cca_mode(mode: CcaMode) {
    critical_section::with(|cs| {
        PIB.borrow_ref_mut(cs).as_mut().unwrap().cca_mode = mode;
    });
}

pub(crate) fn ieee802154_pib_update() {
    critical_section::with(|cs| {
        let mut pib = PIB.borrow_ref_mut(cs);
        let pib = pib.as_mut().unwrap();

        set_freq(channel_to_freq(pib.channel));
        set_power(ieee802154_txpower_convert(pib.txpower));

        set_multipan_enable_mask(pib.multipan_mask);
        ieee802154_set_multipan_hal(pib);

        set_cca_mode(pib.cca_mode);
        set_cca_threshold(pib.cca_threshold);

        set_tx_auto_ack(pib.auto_ack_tx);
        set_rx_auto_ack(pib.auto_ack_rx);
        set_tx_enhance_ack(pib.enhance_ack_tx);

        set_coordinator(pib.coordinator);
        set_promiscuous(pib.promiscuous);
        set_pending_mode(pib.pending_mode == PendingMode::Enhanced);
    });
}

fn channel_to_freq(channel: u8) -> u8 {
    (channel - 11) * 5 + 3
}

fn ieee802154_set_multipan_hal(pib: &Pib) {
    for index in 0..IEEE802154_MULTIPAN_MAX {
        if (pib.multipan_mask & (1 << index)) != 0 {
            set_multipan_panid(index.into(), pib.panid[index]);
            set_multipan_short_addr(index.into(), pib.short_addr[index]);
            set_multipan_ext_addr(index.into(), pib.ext_addr[index].as_ptr());
        }
    }
}

// https://github.com/espressif/esp-idf/blob/release/v5.3/components/ieee802154/driver/esp_ieee802154_pib.c#L48
fn ieee802154_txpower_convert(txpower: i8) -> u8 {
    cfg_if::cfg_if! {
        if #[cfg(feature="esp32h2")] {
            // https://github.com/espressif/esp-idf/blob/release/v5.3/components/hal/esp32h2/include/hal/ieee802154_ll.h
            const IEEE802154_TXPOWER_VALUE_MAX: i8 = 20;
            const IEEE802154_TXPOWER_VALUE_MIN: i8 = -24;
            const IEEE802154_TXPOWER_INDEX_MIN: i8 = 0;
        } else if #[cfg(feature="esp32c6")]{
            // https://github.com/espressif/esp-idf/blob/release/v5.3/components/hal/esp32c6/include/hal/ieee802154_ll.h
            const IEEE802154_TXPOWER_VALUE_MAX: i8 = 20;
            const IEEE802154_TXPOWER_VALUE_MIN: i8 = -15;
            const IEEE802154_TXPOWER_INDEX_MIN: i8 = 3;
        }
    }
    if txpower > IEEE802154_TXPOWER_VALUE_MAX {
        15
    } else if txpower <= IEEE802154_TXPOWER_VALUE_MIN {
        IEEE802154_TXPOWER_INDEX_MIN as u8
    } else {
        (((txpower - IEEE802154_TXPOWER_VALUE_MIN) / 3) + IEEE802154_TXPOWER_INDEX_MIN) as u8
    }
}
