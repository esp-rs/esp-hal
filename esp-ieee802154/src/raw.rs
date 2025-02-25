use core::{cell::RefCell, ptr::addr_of};

use critical_section::Mutex;
use esp_hal::{clock::RadioClockController, handler, interrupt::Priority, peripherals::RADIO_CLK};
use esp_wifi_sys::include::{
    esp_phy_calibration_data_t,
    esp_phy_calibration_mode_t_PHY_RF_CAL_FULL,
    ieee802154_coex_event_t,
    ieee802154_coex_event_t_IEEE802154_IDLE,
    ieee802154_coex_event_t_IEEE802154_LOW,
    ieee802154_coex_event_t_IEEE802154_MIDDLE,
    register_chipv7_phy,
};
use heapless::spsc::Queue;

use crate::{
    frame::{
        frame_get_version,
        frame_is_ack_required,
        FRAME_SIZE,
        FRAME_VERSION_1,
        FRAME_VERSION_2,
    },
    hal::*,
    pib::*,
};

const PHY_ENABLE_VERSION_PRINT: u32 = 1;

static mut RX_BUFFER: [u8; FRAME_SIZE] = [0u8; FRAME_SIZE];
static RX_QUEUE: Mutex<RefCell<Queue<RawReceived, { crate::CONFIG.rx_queue_size }>>> =
    Mutex::new(RefCell::new(Queue::new()));
static STATE: Mutex<RefCell<Ieee802154State>> = Mutex::new(RefCell::new(Ieee802154State::Idle));

extern "C" {
    fn bt_bb_v2_init_cmplx(print_version: u32); // from libbtbb.a

    fn bt_bb_set_zb_tx_on_delay(time: u16); // from libbtbb.a

    fn esp_coex_ieee802154_ack_pti_set(event: ieee802154_coex_event_t); // from ???

    fn esp_coex_ieee802154_txrx_pti_set(event: ieee802154_coex_event_t); // from ???

    fn phy_version_print(); // from libphy.a
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Ieee802154State {
    Idle,
    Receive,
    Transmit,
    TxAck,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy, PartialEq)]
enum Ieee802154TxRxScene {
    Idle,
    Tx,
    Rx,
    TxAt,
    RxAt,
}

/// A raw payload received on some channel
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawReceived {
    /// Payload
    pub data: [u8; FRAME_SIZE],
    /// Receiver channel
    pub channel: u8,
}

pub(crate) fn esp_ieee802154_enable(radio_clock_control: &mut RADIO_CLK) {
    let mut radio_clock_control = RadioClockController::new(radio_clock_control);
    radio_clock_control.init_clocks();
    radio_clock_control.enable_phy(true);
    radio_clock_control.enable_ieee802154(true);

    esp_phy_enable();
    esp_btbb_enable();
    ieee802154_mac_init();

    unsafe { phy_version_print() }; // libphy.a
    info!("date={:x}", mac_date());
}

fn esp_phy_enable() {
    unsafe {
        let mut calibration_data = esp_phy_calibration_data_t {
            version: [0u8; 4],
            mac: [0u8; 6],
            opaque: [0u8; 1894],
        };

        register_chipv7_phy(
            core::ptr::null(),
            &mut calibration_data as *mut esp_phy_calibration_data_t,
            esp_phy_calibration_mode_t_PHY_RF_CAL_FULL,
        );
    }
}

fn esp_btbb_enable() {
    unsafe { bt_bb_v2_init_cmplx(PHY_ENABLE_VERSION_PRINT) };
}

fn ieee802154_mac_init() {
    #[cfg(feature = "esp32c6")]
    unsafe {
        extern "C" {
            static mut coex_pti_tab_ptr: u32;
            static coex_pti_tab: u8;
        }

        // Manually set `coex_pti_tab_ptr` pointing to `coex_pti_tab`
        core::ptr::addr_of_mut!(coex_pti_tab_ptr).write_volatile(&coex_pti_tab as *const _ as u32);
    }

    ieee802154_pib_init();

    enable_events(Event::mask());
    disable_events(Event::Timer0Overflow | Event::Timer1Overflow);

    enable_tx_abort_events(
        TxAbortReason::RxAckTimeout
            | TxAbortReason::TxCoexBreak
            | TxAbortReason::TxSecurityError
            | TxAbortReason::CcaFailed
            | TxAbortReason::CcaBusy
            | TxAbortReason::TxStop,
    );
    enable_rx_abort_events(
        RxAbortReason::TxAckTimeout | RxAbortReason::TxAckCoexBreak | RxAbortReason::RxStop,
    );

    set_ed_sample_mode(EdSampleMode::Avg);

    unsafe { esp_coex_ieee802154_ack_pti_set(ieee802154_coex_event_t_IEEE802154_MIDDLE) };
    ieee802154_set_txrx_pti(Ieee802154TxRxScene::Idle);

    unsafe {
        bt_bb_set_zb_tx_on_delay(50); // set tx on delay for libbtbb.a
    }
    set_rx_on_delay(50);

    // memset(s_rx_frame, 0, sizeof(s_rx_frame));
    // s_ieee802154_state = IEEE802154_STATE_IDLE;

    unsafe {
        esp_hal::interrupt::bind_interrupt(
            esp_hal::peripherals::Interrupt::ZB_MAC,
            ZB_MAC.handler(),
        );
    }
    esp_hal::interrupt::enable(esp_hal::peripherals::Interrupt::ZB_MAC, ZB_MAC.priority()).unwrap();
}

fn ieee802154_set_txrx_pti(txrx_scene: Ieee802154TxRxScene) {
    match txrx_scene {
        Ieee802154TxRxScene::Idle => {
            unsafe { esp_coex_ieee802154_txrx_pti_set(ieee802154_coex_event_t_IEEE802154_IDLE) };
        }
        Ieee802154TxRxScene::Tx | Ieee802154TxRxScene::Rx => {
            unsafe { esp_coex_ieee802154_txrx_pti_set(ieee802154_coex_event_t_IEEE802154_LOW) };
        }
        Ieee802154TxRxScene::TxAt | Ieee802154TxRxScene::RxAt => {
            unsafe { esp_coex_ieee802154_txrx_pti_set(ieee802154_coex_event_t_IEEE802154_MIDDLE) };
        }
    }
}

pub fn tx_init(frame: *const u8) {
    let tx_frame = frame;
    stop_current_operation();
    ieee802154_pib_update();
    ieee802154_sec_update();

    set_tx_addr(tx_frame);

    if true
    // ieee802154_frame_is_ack_required(frame)
    {
        // set rx pointer for ack frame
        set_next_rx_buffer();
    }
}

pub fn ieee802154_transmit(frame: *const u8, cca: bool) -> i32 {
    critical_section::with(|cs| {
        tx_init(frame);

        ieee802154_set_txrx_pti(Ieee802154TxRxScene::Tx);

        if cca {
            // disable_events(IEEE802154_EVENT_ED_DONE);
            // set_cmd(IEEE802154_CMD_CCA_TX_START);
            // ieee802154_state = IEEE802154_STATE_TX_CCA;
        } else {
            set_cmd(Command::TxStart);
            // if (ieee802154_frame_get_type(frame) == IEEE802154_FRAME_TYPE_ACK
            //     && ieee802154_frame_get_version(frame) == IEEE802154_FRAME_VERSION_2)
            // {
            //     ieee802154_state = IEEE802154_STATE_TX_ENH_ACK;
            // } else {
            *STATE.borrow_ref_mut(cs) = Ieee802154State::Transmit;
            // }
        }
    });

    0 // ESP_OK
}

pub fn ieee802154_receive() -> i32 {
    critical_section::with(|cs| {
        if *STATE.borrow_ref(cs) == Ieee802154State::Receive {
            return;
        }

        rx_init();
        enable_rx();

        *STATE.borrow_ref_mut(cs) = Ieee802154State::Receive;
    });

    0 // ESP-OK
}

pub fn ieee802154_poll() -> Option<RawReceived> {
    critical_section::with(|cs| {
        let mut queue = RX_QUEUE.borrow_ref_mut(cs);
        queue.dequeue()
    })
}

fn rx_init() {
    stop_current_operation();
    ieee802154_pib_update();
}

fn enable_rx() {
    set_next_rx_buffer();
    ieee802154_set_txrx_pti(Ieee802154TxRxScene::Rx);

    set_cmd(Command::RxStart);

    // ieee802154_state = IEEE802154_STATE_RX;
}

fn stop_current_operation() {
    let events = events();
    set_cmd(Command::Stop);
    clear_events(events);
}

fn set_next_rx_buffer() {
    #[allow(unused_unsafe)] // stable compiler needs unsafe, nightly complains about it
    unsafe {
        set_rx_addr(core::ptr::addr_of_mut!(RX_BUFFER).cast());
    }
}

pub fn set_promiscuous(enable: bool) {
    ieee802154_pib_set_promiscuous(enable);
}

pub fn set_auto_ack_tx(enable: bool) {
    ieee802154_pib_set_auto_ack_tx(enable);
}

pub fn set_auto_ack_rx(enable: bool) {
    ieee802154_pib_set_auto_ack_rx(enable);
}

pub fn set_enhance_ack_tx(enable: bool) {
    ieee802154_pib_set_enhance_ack_tx(enable);
}

pub fn set_coordinator(enable: bool) {
    ieee802154_pib_set_coordinator(enable);
}

pub fn set_rx_when_idle(enable: bool) {
    ieee802154_pib_set_rx_when_idle(enable);
}

pub fn set_tx_power(power: i8) {
    ieee802154_pib_set_tx_power(power);
}

pub fn set_channel(channel: u8) {
    ieee802154_pib_set_channel(channel);
}

#[allow(unused)]
pub fn set_pending_mode(mode: PendingMode) {
    ieee802154_pib_set_pending_mode(mode);
}

#[allow(unused)]
pub fn set_multipan_enable(mask: u8) {
    set_multipan_enable_mask(mask);
}

pub fn set_short_address(index: u8, address: u16) {
    ieee802154_pib_set_short_address(index, address);
}

pub fn set_extended_address(index: u8, address: [u8; IEEE802154_FRAME_EXT_ADDR_SIZE]) {
    ieee802154_pib_set_extended_address(index, address);
}

pub fn set_cca_theshold(cca_threshold: i8) {
    ieee802154_pib_set_cca_theshold(cca_threshold);
}

pub fn set_cca_mode(mode: CcaMode) {
    ieee802154_pib_set_cca_mode(mode);
}

pub fn set_panid(index: u8, id: u16) {
    ieee802154_pib_set_panid(index, id);
}

#[inline(always)]
fn ieee802154_sec_update() {
    let is_security = false;
    set_transmit_security(is_security);
    // ieee802154_sec_clr_transmit_security();
}

fn next_operation() {
    let previous_operation = critical_section::with(|cs| {
        let state = *STATE.borrow_ref(cs);

        if ieee802154_pib_get_rx_when_idle() {
            enable_rx();
            *STATE.borrow_ref_mut(cs) = Ieee802154State::Receive;
        } else {
            *STATE.borrow_ref_mut(cs) = Ieee802154State::Idle;
        }

        state
    });

    match previous_operation {
        Ieee802154State::Receive => crate::rx_available(),
        Ieee802154State::Transmit => crate::tx_done(),
        Ieee802154State::TxAck => crate::tx_done(),
        _ => (),
    }
}

#[handler(priority = "Priority::Priority1")]
fn ZB_MAC() {
    trace!("ZB_MAC interrupt");

    let events = events();
    clear_events(events);

    trace!("events = {:032b}", events);

    if events & Event::RxSfdDone != 0 {
        // IEEE802154_STATE_TX && IEEE802154_STATE_TX_CCA && IEEE802154_STATE_TX_ENH_ACK
        // for isr processing delay
        trace!("rx sfd done");
    }

    if events & Event::TxSfdDone != 0 {
        // IEEE802154_STATE_RX for isr processing delay, only 821
        // IEEE802154_STATE_TX_ACK for workaround jira ZB-81.
        trace!("tx sfd done");
    }

    if events & Event::TxDone != 0 {
        trace!("tx done");
        next_operation();
    }

    if events & Event::RxDone != 0 {
        trace!("rx done");
        unsafe {
            trace!("Received raw {:x?}", &*addr_of!(RX_BUFFER));
            critical_section::with(|cs| {
                let mut queue = RX_QUEUE.borrow_ref_mut(cs);
                if !queue.is_full() {
                    let item = RawReceived {
                        data: RX_BUFFER,
                        channel: freq_to_channel(freq()),
                    };
                    queue.enqueue(item).ok();
                } else {
                    warn!("Receive queue full");
                }

                let frm = if RX_BUFFER[0] >= FRAME_SIZE as u8 {
                    warn!("RX_BUFFER[0] {:} is larger than frame size", RX_BUFFER[0]);
                    &RX_BUFFER[1..][..FRAME_SIZE - 1]
                } else {
                    &RX_BUFFER[1..][..RX_BUFFER[0] as usize]
                };
                if will_auto_send_ack(frm) {
                    *STATE.borrow_ref_mut(cs) = Ieee802154State::TxAck;
                } else if should_send_enhanced_ack(frm) {
                    // TODO
                } else {
                    // esp_ieee802154_coex_pti_set(IEEE802154_IDLE_RX);
                    next_operation();
                }
            });
        }
    }

    if events & Event::AckRxDone != 0 {
        info!("EventAckRxDone");
    }

    if events & Event::AckTxDone != 0 {
        trace!("EventAckTxDone");
        next_operation();
    }

    if events & Event::TxAbort != 0 {
        trace!("TxAbort");
        abort_tx();
    }

    if events & Event::RxAbort != 0 {
        trace!("RxAbort");
        abort_rx();
    }
}

fn freq_to_channel(freq: u8) -> u8 {
    (freq - 3) / 5 + 11
}

fn will_auto_send_ack(frame: &[u8]) -> bool {
    frame_is_ack_required(frame) && frame_get_version(frame) <= FRAME_VERSION_1 && tx_auto_ack()
}

fn should_send_enhanced_ack(frame: &[u8]) -> bool {
    frame_is_ack_required(frame) && frame_get_version(frame) <= FRAME_VERSION_2 && tx_enhance_ack()
}
