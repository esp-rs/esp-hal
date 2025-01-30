//! Test we can talk to the BLE controller.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32s2 esp32s3
//% FEATURES: unstable esp-wifi esp-alloc esp-wifi/ble

#![no_std]
#![no_main]

use embedded_io::{Read, Write};
use hil_test as _;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{clock::CpuClock, peripherals::Peripherals, rng::Rng, timer::timg::TimerGroup};
    use esp_wifi::ble::controller::BleConnector;

    use super::*;

    const H4_TYPE_COMMAND: u8 = 1;
    const HCI_GRP_HOST_CONT_BASEBAND_CMDS: u16 = 0x03 << 10;

    const HCI_RESET_BYTE0: u8 = ((0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS) & 0xff00 >> 8) as u8;
    const HCI_RESET_BYTE1: u8 = ((0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS) & 0xff) as u8;

    #[init]
    fn init() -> Peripherals {
        esp_alloc::heap_allocator!(72 * 1024);

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[test]
    fn test_controller_comms(mut peripherals: Peripherals) {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let init = esp_wifi::init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap();

        let mut connector = BleConnector::new(&init, &mut peripherals.BT);

        // send reset
        connector
            .write(&[H4_TYPE_COMMAND, HCI_RESET_BYTE0, HCI_RESET_BYTE1, 0])
            .unwrap();

        // wait for an expected response - we might get some other events, too
        // so if we don't get the expected response we'll timeout and fail
        let mut buf = [0u8; 255];
        loop {
            let len = connector.read(&mut buf).unwrap();
            if len == 7 {
                assert_eq!(buf[0], 4);
                assert_eq!(buf[1], 14);
                assert_eq!(buf[2], 4);

                assert_eq!(buf[4], 3);
                assert_eq!(buf[5], 3);
                assert_eq!(buf[6], 1);
                break;
            }
        }
    }
}
