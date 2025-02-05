//! Test we can talk to the BLE controller.
//!
//! We just send a reset command and check that we get a command complete for
//! the command.
//!
//! This is at least enough to know BLE is initialized find and the controller
//! responds as expected.
//!
//! See Bluetooth Core Specification v5.3
//!
//! THIS DOESN'T ACTUALLY TEST THE RADIO HOWEVER.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s3
//% FEATURES: unstable esp-wifi esp-alloc esp-wifi/ble

#![no_std]
#![no_main]

use embedded_io::{Read, Write};
use esp_hal::{clock::CpuClock, peripherals::Peripherals, rng::Rng, timer::timg::TimerGroup};
use esp_wifi::ble::controller::BleConnector;
use hil_test as _;

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    const H4_TYPE_COMMAND: u8 = 1;

    pub const CONTROLLER_OGF: u8 = 0x03;
    pub const RESET_OCF: u16 = 0x03;

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

        // send reset cmd
        pub const fn opcode(ogf: u8, ocf: u16) -> [u8; 2] {
            let opcode = ((ogf as u16) << 10) + ocf as u16;
            opcode.to_le_bytes()
        }

        let opcode = opcode(CONTROLLER_OGF, RESET_OCF);

        let mut buf = [0u8; 4];
        buf[0] = H4_TYPE_COMMAND;
        buf[1] = opcode[0];
        buf[2] = opcode[1];
        buf[3] = 0; // payload length - RESET COMMAND doesn't have any parameters

        connector.write(&buf).unwrap();

        // wait for an expected response - we might get some other events, too, since
        // different chips have different defaults for which events are masked away
        // so if we don't get the expected response we'll timeout and fail
        let mut buf = [0u8; 255];
        loop {
            let len = read_packet(&mut connector, &mut buf);
            if len == 7 {
                assert_eq!(buf[0], 4, "Expected packet type = 4 (EVENT)");
                assert_eq!(buf[1], 14, "Expected event code = 14 (COMMAND_COMPLETE)");
                assert_eq!(buf[2], 4, "Expected payload length = 4");

                // don't care about NUM_HANDLES (different for different chips)

                assert_eq!(buf[4], opcode[0], "Unexpected op-code first byte");
                assert_eq!(buf[5], opcode[1], "Unexpected op-code second byte");
                assert_eq!(
                    buf[6], 0,
                    "Expected RESET COMMAND to succeed (return code = 0)"
                );
                break;
            }
        }
    }
}

fn read_packet(connector: &mut BleConnector, buf: &mut [u8]) -> usize {
    // Read header
    read_all(connector, &mut buf[..3]);

    // Read payload
    let payload_len = buf[2] as usize;
    read_all(connector, &mut buf[3..][..payload_len]);

    3 + payload_len
}

fn read_all(connector: &mut BleConnector, mut buf: &mut [u8]) {
    while !buf.is_empty() {
        let len = connector.read(buf).unwrap();
        buf = &mut buf[len..];
    }
}
