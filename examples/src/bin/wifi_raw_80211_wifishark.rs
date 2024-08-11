//! Barebones promiscuous 802.11 frame dumping.
//! Best used with the 'extras/esp-wifishark' extcap plugin for Wireshark.
//!
//! Currently hard-coded for 2.4GHz Channel 1.

//% FEATURES: esp-wifi-sys esp-wifi esp-wifi/wifi-default esp-wifi/wifi esp-wifi/utils esp-wifi/esp-now esp-wifi/phy-enable-usb
//% CHIPS: esp32 esp32s2 esp32s3 esp32c2 esp32c3 esp32c6

#![no_std]
#![no_main]

use embedded_io::*;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, PeriodicTimer}
};
use esp_println::{print, println};
use esp_wifi::{
    current_millis,
    initialize,
    wifi::{
        utils::create_network_interface,
        AccessPointInfo,
        ClientConfiguration,
        Configuration,
        WifiError,
        WifiStaDevice,
    },
    esp_now::RxControlInfo,
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use esp_wifi_sys::include;
use smoltcp::{
    iface::SocketStorage,
    wire::{IpAddress, Ipv4Address},
};

#[derive(Debug, Clone, Copy)]
pub enum WiFiPromiscuousPacketType {
    #[doc = "< Management frame, indicates 'buf' argument is wifi_promiscuous_pkt_t"]
    WIFI_PKT_MGMT = 0,
    #[doc = "< Control frame, indicates 'buf' argument is wifi_promiscuous_pkt_t"]
    WIFI_PKT_CTRL = 1,
    #[doc = "< Data frame, indiciates 'buf' argument is wifi_promiscuous_pkt_t"]
    WIFI_PKT_DATA = 2,
    #[doc = "< Other type, such as MIMO etc. 'buf' argument is wifi_promiscuous_pkt_t but the payload is zero length."]
    WIFI_PKT_MISC = 3,
}

impl From<u32> for WiFiPromiscuousPacketType {
    fn from(item: u32) -> Self {
      match item {
        0 => WiFiPromiscuousPacketType::WIFI_PKT_MGMT,
        1 => WiFiPromiscuousPacketType::WIFI_PKT_CTRL,
        2 => WiFiPromiscuousPacketType::WIFI_PKT_DATA,
        3 => WiFiPromiscuousPacketType::WIFI_PKT_MISC,
        _ => panic!("Unexpected packet type")
      }
    }
}

// Take inspiration from esp-now, to unpack this data
// https://github.com/esp-rs/esp-hal/blob/2e8937a90ff1e081a7c4127001900cf3f800abb3/esp-wifi/src/esp_now/mod.rs#L792C1-L887C2
unsafe extern "C" fn recv_cb_prom(
    buffer: *mut esp_wifi_sys::c_types::c_void,
    msgtype: u32,
) -> () {
    // Cast the bugger and msgtype to their relevant structs
    let rx_pkt_type : WiFiPromiscuousPacketType = (msgtype as include::wifi_promiscuous_pkt_type_t).into();
    let rx_pkt = & *(buffer as *mut include::wifi_promiscuous_pkt_t);
    let rx_cntl = rx_pkt.rx_ctrl;

    // rx_ctrl provides the payload length, rx_cntl.sig_len
    // RxControlInfo is being re-used here, from the EspNow module
    let rx_control = RxControlInfo {
        rssi: rx_cntl.rssi(),
        rate: rx_cntl.rate(),
        sig_mode: rx_cntl.sig_mode(),
        mcs: rx_cntl.mcs(),
        cwb: rx_cntl.cwb(),
        smoothing: rx_cntl.smoothing(),
        not_sounding: rx_cntl.not_sounding(),
        aggregation: rx_cntl.aggregation(),
        stbc: rx_cntl.stbc(),
        fec_coding: rx_cntl.fec_coding(),
        sgi: rx_cntl.sgi(),
        ampdu_cnt: rx_cntl.ampdu_cnt(),
        channel: rx_cntl.channel(),
        secondary_channel: rx_cntl.secondary_channel(),
        timestamp: rx_cntl.timestamp(),
        noise_floor: rx_cntl.noise_floor(),
        ant: rx_cntl.ant(),
        sig_len: rx_cntl.sig_len(),
        rx_state: rx_cntl.rx_state(),
    };

    let payload_len : u32 = match (rx_pkt_type,rx_control.sig_len) {
      (WiFiPromiscuousPacketType::WIFI_PKT_MISC,_) => 0, // MISC packets have no payload
      (_,n) => n
    };

    if payload_len > 0 {
      let slice = rx_pkt.payload.as_slice(payload_len as usize);
      // This print format is what the 'esp-wifishark' extcap plugin expects
      println!("@WIFIFRAME {:?}", slice);
    }
}

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let peripherals = Peripherals::take();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let timer0: ErasedTimer = timg0.timer0.into();
    let timer = PeriodicTimer::new(timer0);

    let init = initialize(
        EspWifiInitFor::Wifi,
        timer,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();


    let wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) = create_network_interface(&init, wifi, WifiStaDevice, &mut socket_set_entries).unwrap();
    let wifi_stack = WifiStack::new(iface, device, sockets, current_millis);
    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());
 
    
    // This unsafe block is required,
    // because there is currently no 'proper API' to enable promiscious mode.
    // So we have to do it 'by hand'.
    unsafe {
      let result = include::esp_wifi_set_promiscuous(true);
      println!("esp_wifi_set_promiscuous = {:?}",(result==0));
      let cbresult = include::esp_wifi_set_promiscuous_rx_cb(Some(recv_cb_prom));
      println!("esp_wifi_set_promiscuous_rx_cb = {:?}",(cbresult==0));

      // Now we wait, and let the recv_cb_prom handle everything!
      loop {
      }
    }
}


