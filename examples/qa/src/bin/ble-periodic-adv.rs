//! Regression / verification test for <https://github.com/esp-rs/esp-hal/issues/6506>.
//!
//! Directly writes and reads HCI packets to/from the BLE controller (without a
//! BLE host stack) to verify `LE Set Periodic Advertising Data` (0x203F) with a
//! non-empty payload and `Operation = 0x03` (Complete periodic advertising data).
//!
//! In #6506, `bt-hci`'s `WriteHci::write_hci_async` for `&[u8]` encoded the
//! `Advertising_Data_Length` parameter as `self.size()` (`len + 1`) instead of
//! `self.len()`, causing the controller to reject the command with status `0x12`
//! (`Invalid HCI Command Parameters`) because the total parameter length did not
//! match `3 + Advertising_Data_Length`.

//% FEATURES: esp-radio esp-radio/ble esp-radio/unstable esp-hal/unstable
//% CHIP_FILTER: bt_driver_supported && !esp32

#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, delay::Delay, timer::timg::TimerGroup};
use esp_println::println;
use esp_radio::ble::controller::BleConnector;

esp_bootloader_esp_idf::esp_app_desc!();

const H4_TYPE_COMMAND: u8 = 0x01;
const H4_TYPE_EVENT: u8 = 0x04;
const EVT_COMMAND_COMPLETE: u8 = 0x0E;

const OGF_CONTROLLER_BASEBAND: u8 = 0x03;
const OGF_LE_CONTROLLER: u8 = 0x08;

const OCF_RESET: u16 = 0x0003;
const OCF_LE_SET_ADV_SET_RANDOM_ADDR: u16 = 0x0035;
const OCF_LE_SET_EXT_ADV_PARAMS: u16 = 0x0036;
const OCF_LE_SET_EXT_ADV_DATA: u16 = 0x0037;
const OCF_LE_SET_EXT_ADV_ENABLE: u16 = 0x0039;
const OCF_LE_SET_PERIODIC_ADV_PARAMS: u16 = 0x003E;
const OCF_LE_SET_PERIODIC_ADV_DATA: u16 = 0x003F;
const OCF_LE_SET_PERIODIC_ADV_ENABLE: u16 = 0x0040;

const fn opcode(ogf: u8, ocf: u16) -> [u8; 2] {
    (((ogf as u16) << 10) | ocf).to_le_bytes()
}

fn read_all(connector: &mut BleConnector, mut buf: &mut [u8]) {
    while !buf.is_empty() {
        let len = connector.read(buf).unwrap();
        buf = &mut buf[len..];
    }
}

fn read_packet(connector: &mut BleConnector, buf: &mut [u8]) -> usize {
    read_all(connector, &mut buf[..3]);
    let payload_len = buf[2] as usize;
    read_all(connector, &mut buf[3..][..payload_len]);
    3 + payload_len
}

/// Sends an HCI command and waits for its `Command Complete` event, returning the status byte.
fn send_cmd(connector: &mut BleConnector, name: &str, ogf: u8, ocf: u16, params: &[u8]) -> u8 {
    let op = opcode(ogf, ocf);
    let mut cmd = [0u8; 260];
    cmd[0] = H4_TYPE_COMMAND;
    cmd[1] = op[0];
    cmd[2] = op[1];
    cmd[3] = params.len() as u8;
    cmd[4..][..params.len()].copy_from_slice(params);

    connector.write(&cmd[..4 + params.len()]).unwrap();

    let mut evt = [0u8; 260];
    loop {
        let len = read_packet(connector, &mut evt);
        if len >= 7
            && evt[0] == H4_TYPE_EVENT
            && evt[1] == EVT_COMMAND_COMPLETE
            && evt[4] == op[0]
            && evt[5] == op[1]
        {
            println!(
                "[{}] (0x{:04x}) complete: status=0x{:02x} payload={:02x?}",
                name,
                u16::from_le_bytes(op),
                evt[6],
                &evt[3..len]
            );
            return evt[6];
        }
    }
}

#[esp_hal::main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let config = esp_radio::ble::Config::default().with_ext_adv_max_size(255);
    let mut connector = BleConnector::new(peripherals.BT, config).unwrap();

    // 1. HCI Reset
    let status = send_cmd(
        &mut connector,
        "HCI Reset",
        OGF_CONTROLLER_BASEBAND,
        OCF_RESET,
        &[],
    );
    assert_eq!(status, 0x00, "HCI Reset failed");

    // 2. LE Set Extended Advertising Parameters (7.8.53)
    // Adv_Handle = 0, Adv_Event_Properties = 0x0000 (non-connectable, non-scannable),
    // Primary_Adv_Interval_Min = 0x0000A0 (100ms), Primary_Adv_Interval_Max = 0x000140 (200ms),
    // Primary_Adv_Channel_Map = 0x07 (all), Own_Address_Type = 0x01 (Random),
    // Peer_Address_Type = 0x00, Peer_Address = [0; 6], Adv_Filter_Policy = 0x00,
    // Adv_Tx_Power = 0x7F (no preference), Primary_Adv_PHY = 0x01 (LE 1M),
    // Secondary_Adv_Max_Skip = 0x00, Secondary_Adv_PHY = 0x01 (LE 1M),
    // Adv_SID = 0x00, Scan_Request_Notification_Enable = 0x00
    let ext_adv_params = [
        0x00, // Advertising_Handle
        0x00, 0x00, // Advertising_Event_Properties
        0xA0, 0x00, 0x00, // Primary_Advertising_Interval_Min (160 * 0.625ms = 100ms)
        0x40, 0x01, 0x00, // Primary_Advertising_Interval_Max (320 * 0.625ms = 200ms)
        0x07, // Primary_Advertising_Channel_Map
        0x01, // Own_Address_Type (Random)
        0x00, // Peer_Address_Type
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Peer_Address
        0x00, // Advertising_Filter_Policy
        0x7F, // Advertising_Tx_Power
        0x01, // Primary_Advertising_PHY (LE 1M)
        0x00, // Secondary_Advertising_Max_Skip
        0x01, // Secondary_Advertising_PHY (LE 1M)
        0x00, // Advertising_SID
        0x00, // Scan_Request_Notification_Enable
    ];
    let status = send_cmd(
        &mut connector,
        "LE Set Extended Advertising Parameters",
        OGF_LE_CONTROLLER,
        OCF_LE_SET_EXT_ADV_PARAMS,
        &ext_adv_params,
    );
    assert_eq!(
        status, 0x00,
        "LE Set Extended Advertising Parameters failed"
    );

    // 3. LE Set Advertising Set Random Address (7.8.52)
    let rand_addr_params = [
        0x00, // Advertising_Handle
        0xFF, 0x8F, 0x1A, 0x05, 0xE4, 0xFF, // Random_Address (static random: top 2 bits = 11)
    ];
    let status = send_cmd(
        &mut connector,
        "LE Set Advertising Set Random Address",
        OGF_LE_CONTROLLER,
        OCF_LE_SET_ADV_SET_RANDOM_ADDR,
        &rand_addr_params,
    );
    assert_eq!(status, 0x00, "LE Set Advertising Set Random Address failed");

    // 4. LE Set Extended Advertising Data (7.8.54) so the device shows up with a name in a BLE
    // scanner app ("ESP Periodic Adv").
    // AD Structure 1: Flags (0x01) = LE General Discoverable (0x02) | BR/EDR Not Supported (0x04)
    // AD Structure 2: Complete Local Name (0x09) = "ESP Periodic Adv"
    const LOCAL_NAME: &[u8] = b"ESP Periodic Adv";
    let mut ext_adv_data = [0u8; 4 + 3 + 2 + LOCAL_NAME.len()];
    ext_adv_data[0] = 0x00; // Advertising_Handle
    ext_adv_data[1] = 0x03; // Operation (Complete extended advertising data)
    ext_adv_data[2] = 0x01; // Fragment_Preference (Should not fragment)
    ext_adv_data[3] = (3 + 2 + LOCAL_NAME.len()) as u8; // Advertising_Data_Length
    ext_adv_data[4..9].copy_from_slice(&[
        0x02,
        0x01,
        0x06, // Flags: LE General Discoverable | BR/EDR Not Supported
        (1 + LOCAL_NAME.len()) as u8,
        0x09, // Complete Local Name
    ]);
    ext_adv_data[9..].copy_from_slice(LOCAL_NAME);
    let status = send_cmd(
        &mut connector,
        "LE Set Extended Advertising Data",
        OGF_LE_CONTROLLER,
        OCF_LE_SET_EXT_ADV_DATA,
        &ext_adv_data,
    );
    assert_eq!(status, 0x00, "LE Set Extended Advertising Data failed");

    // 5. LE Set Periodic Advertising Parameters (7.8.61)
    // Adv_Handle = 0, Periodic_Adv_Interval_Min = 0x0050 (100ms),
    // Periodic_Adv_Interval_Max = 0x0050 (100ms), Periodic_Adv_Properties = 0x0000
    let per_adv_params = [
        0x00, // Advertising_Handle
        0x50, 0x00, // Periodic_Advertising_Interval_Min (80 * 1.25ms = 100ms)
        0x50, 0x00, // Periodic_Advertising_Interval_Max (80 * 1.25ms = 100ms)
        0x00, 0x00, // Periodic_Advertising_Properties
    ];
    let status = send_cmd(
        &mut connector,
        "LE Set Periodic Advertising Parameters",
        OGF_LE_CONTROLLER,
        OCF_LE_SET_PERIODIC_ADV_PARAMS,
        &per_adv_params,
    );
    assert_eq!(
        status, 0x00,
        "LE Set Periodic Advertising Parameters failed"
    );

    // 6. LE Set Periodic Advertising Data (7.8.62)
    // AD Structure: Manufacturer Specific Data (0xFF), Company ID 0xFFFF, counter byte
    let adv_payload = [0x04, 0xFF, 0xFF, 0xFF, 0x00];
    let mut per_adv_data = [0u8; 3 + 5];
    per_adv_data[0] = 0x00; // Advertising_Handle
    per_adv_data[1] = 0x03; // Operation (Complete periodic advertising data)
    per_adv_data[2] = adv_payload.len() as u8; // Advertising_Data_Length
    per_adv_data[3..].copy_from_slice(&adv_payload);

    let status = send_cmd(
        &mut connector,
        "LE Set Periodic Advertising Data",
        OGF_LE_CONTROLLER,
        OCF_LE_SET_PERIODIC_ADV_DATA,
        &per_adv_data,
    );
    assert_eq!(status, 0x00, "LE Set Periodic Advertising Data failed");

    // 7. LE Set Periodic Advertising Enable (7.8.63)
    let per_adv_enable = [
        0x01, // Enable = 0x01 (Periodic advertising enabled)
        0x00, // Advertising_Handle
    ];
    let status = send_cmd(
        &mut connector,
        "LE Set Periodic Advertising Enable",
        OGF_LE_CONTROLLER,
        OCF_LE_SET_PERIODIC_ADV_ENABLE,
        &per_adv_enable,
    );
    assert_eq!(status, 0x00, "LE Set Periodic Advertising Enable failed");

    // 8. LE Set Extended Advertising Enable (7.8.56)
    let ext_adv_enable = [
        0x01, // Enable = 0x01
        0x01, // Number_of_Sets = 1
        0x00, // Advertising_Handle[0] = 0
        0x00, 0x00, // Duration[0] = 0 (no duration limit)
        0x00, // Max_Extended_Advertising_Events[0] = 0
    ];
    let status = send_cmd(
        &mut connector,
        "LE Set Extended Advertising Enable",
        OGF_LE_CONTROLLER,
        OCF_LE_SET_EXT_ADV_ENABLE,
        &ext_adv_enable,
    );
    assert_eq!(status, 0x00, "LE Set Extended Advertising Enable failed");

    println!(
        "PASS: Advertising as \"ESP Periodic Adv\" (FF:E4:05:1A:8F:FF), updating periodic payload every second..."
    );

    // 9. Continuously update periodic advertising data while advertising
    let delay = Delay::new();
    let mut counter: u8 = 0;
    loop {
        delay.delay_millis(1000);
        counter = counter.wrapping_add(1);
        per_adv_data[7] = counter;
        let status = send_cmd(
            &mut connector,
            "LE Set Periodic Advertising Data (update)",
            OGF_LE_CONTROLLER,
            OCF_LE_SET_PERIODIC_ADV_DATA,
            &per_adv_data,
        );
        assert_eq!(
            status, 0x00,
            "Updating LE Periodic Advertising Data while enabled failed"
        );
    }
}
