use alloc::collections::VecDeque as Queue;

use esp_hal::{
    clock::{ModemClockController, PhyClockGuard, init_radio_clocks},
    handler,
    interrupt::Priority,
    peripherals::IEEE802154,
};
use esp_phy::{PhyController, PhyInitGuard};
use esp_sync::NonReentrantMutex;
use esp_wifi_sys::include::{
    ieee802154_coex_event_t,
    ieee802154_coex_event_t_IEEE802154_IDLE,
    ieee802154_coex_event_t_IEEE802154_LOW,
    ieee802154_coex_event_t_IEEE802154_MIDDLE,
};

use super::{
    frame::{
        FRAME_SIZE,
        FRAME_VERSION_1,
        FRAME_VERSION_2,
        frame_get_version,
        frame_is_ack_required,
    },
    hal::*,
    pib::*,
};

const PHY_ENABLE_VERSION_PRINT: u8 = 1;

static mut RX_BUFFER: [u8; FRAME_SIZE] = [0u8; FRAME_SIZE];

struct IeeeState {
    state: Ieee802154State,
    rx_queue: Queue<RawReceived>,
    rx_queue_size: usize,
}

static STATE: NonReentrantMutex<IeeeState> = NonReentrantMutex::new(IeeeState {
    state: Ieee802154State::Idle,
    rx_queue: Queue::new(),
    rx_queue_size: 10,
});

unsafe extern "C" {
    fn bt_bb_v2_init_cmplx(print_version: u8); // from libbtbb.a

    fn bt_bb_set_zb_tx_on_delay(time: u16); // from libbtbb.a

    fn esp_coex_ieee802154_ack_pti_set(event: ieee802154_coex_event_t); // from ???

    fn esp_coex_ieee802154_txrx_pti_set(event: ieee802154_coex_event_t); // from ???
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

pub(crate) fn esp_ieee802154_enable(
    mut radio: IEEE802154<'_>,
) -> (PhyClockGuard<'_>, PhyInitGuard<'_>) {
    init_radio_clocks();
    let phy_clock_guard = radio.enable_phy_clock();
    radio.enable_modem_clock(true);

    let phy_init_guard = radio.enable_phy();

    esp_btbb_enable();
    ieee802154_mac_init();

    info!("date={:x}", mac_date());
    (phy_clock_guard, phy_init_guard)
}

fn esp_btbb_enable() {
    unsafe { bt_bb_v2_init_cmplx(PHY_ENABLE_VERSION_PRINT) };
}

fn ieee802154_mac_init() {
    #[cfg(feature = "esp32c6")]
    unsafe {
        unsafe extern "C" {
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
            zb_mac_handler.handler(),
        );
    }
    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::ZB_MAC,
        zb_mac_handler.priority(),
    )
    .unwrap();
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

pub(crate) fn set_queue_size(rx_queue_size: usize) {
    STATE.with(|state| {
        state.rx_queue_size = rx_queue_size;
    });
}

pub fn ieee802154_transmit(frame: *const u8, cca: bool) -> i32 {
    STATE.with(|state| {
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
            state.state = Ieee802154State::Transmit;
            // }
        }
    });

    0 // ESP_OK
}

pub fn ieee802154_receive() -> i32 {
    STATE.with(|state| {
        if state.state == Ieee802154State::Receive {
            return;
        }

        rx_init();
        enable_rx();

        state.state = Ieee802154State::Receive;
    });

    0 // ESP-OK
}

pub fn ieee802154_poll() -> Option<RawReceived> {
    STATE.with(|state| state.rx_queue.pop_front())
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
    set_rx_addr(core::ptr::addr_of_mut!(RX_BUFFER).cast());
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

fn next_operation_inner(state: &mut IeeeState) -> Ieee802154State {
    let prev_state = state.state;
    state.state = if ieee802154_pib_get_rx_when_idle() {
        enable_rx();
        Ieee802154State::Receive
    } else {
        Ieee802154State::Idle
    };

    prev_state
}

fn notify_state(state: Ieee802154State) {
    match state {
        Ieee802154State::Receive => super::rx_available(),
        Ieee802154State::Transmit => super::tx_done(),
        Ieee802154State::TxAck => super::tx_done(),
        _ => (),
    }
}

fn next_operation() {
    let previous_operation = STATE.with(next_operation_inner);

    notify_state(previous_operation)
}

#[handler(priority = Priority::Priority1)]
fn zb_mac_handler() {
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
            trace!(
                "Received raw {:?}",
                crate::fmt::Bytes(&*core::ptr::addr_of!(RX_BUFFER))
            );
            let mut state_for_notify = Ieee802154State::Idle;
            STATE.with(|state| {
                if state.rx_queue.len() <= state.rx_queue_size {
                    let item = RawReceived {
                        data: RX_BUFFER,
                        channel: freq_to_channel(freq()),
                    };
                    state.rx_queue.push_back(item);
                } else {
                    warn!("Receive queue full");
                }

                let frm = if RX_BUFFER[0] >= FRAME_SIZE as u8 {
                    warn!("RX_BUFFER[0] {} is larger than frame size", RX_BUFFER[0]);
                    &RX_BUFFER[1..][..FRAME_SIZE - 1]
                } else {
                    &RX_BUFFER[1..][..RX_BUFFER[0] as usize]
                };
                if will_auto_send_ack(frm) {
                    state.state = Ieee802154State::TxAck;
                } else if should_send_enhanced_ack(frm) {
                    // TODO
                } else {
                    state_for_notify = next_operation_inner(state)
                    // esp_ieee802154_coex_pti_set(IEEE802154_IDLE_RX);
                }
            });

            notify_state(state_for_notify)
        }
    }

    if events & Event::AckRxDone != 0 {
        trace!("EventAckRxDone");
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
