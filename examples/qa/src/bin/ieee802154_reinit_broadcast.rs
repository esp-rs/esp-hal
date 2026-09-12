//! Repeatedly initializes and deinitializes the IEEE 802.15.4 driver and
//! broadcasts data frames. The iterations after the first exercise
//! re-initialization after the modem clocks were torn down.
//!
//! Verify with a sniffer on channel 15 (e.g. a second board running the
//! `ieee802154_sniffer` example) that frames are received during every
//! active phase, with continuously increasing sequence numbers.
//!
//! The test is also suited to verify the power savings of the teardown with
//! a (slow) power meter:
//! - after boot there is a startup delay with the radio never initialized (baseline consumption),
//! - each iteration then has a 10s active phase (radio initialized, sending two frames per second)
//!   followed by a 10s inactive phase (radio deinitialized). The inactive-phase consumption should
//!   match the baseline.
//!
//! Success: the sniffer receives frames during every active phase.
//! Failure: the sniffer stops seeing frames after the first iteration.

//% FEATURES: esp-radio esp-radio/ieee802154 esp-radio/unstable esp-hal/unstable
//% CHIP_FILTER: ieee802154_driver_supported

#![no_std]
#![no_main]

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, main};
use esp_println::println;
use esp_radio::ieee802154::{Config, Frame, Ieee802154};
use ieee802154::mac::{
    Address,
    FrameContent,
    FrameType,
    FrameVersion,
    Header,
    PanId,
    ShortAddress,
};

esp_bootloader_esp_idf::esp_app_desc!();

/// Settle time after boot, with the radio never initialized (baseline power).
const STARTUP_DELAY_MS: u32 = 5000;
/// How long the radio stays initialized per iteration.
const ACTIVE_PHASE_MS: u32 = 10_000;
/// How long the radio stays deinitialized per iteration.
const INACTIVE_PHASE_MS: u32 = 10_000;
/// Frame interval during the active phase.
const FRAME_INTERVAL_MS: u32 = 500;

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 24 * 1024);

    let delay = Delay::new();

    println!(
        "Waiting {}s (baseline power, radio never initialized)",
        STARTUP_DELAY_MS / 1000
    );
    delay.delay_millis(STARTUP_DELAY_MS);

    let mut radio = peripherals.IEEE802154;
    let mut seq_number = 0u8;
    loop {
        {
            let mut ieee802154 = Ieee802154::new(radio.reborrow());

            ieee802154.set_config(Config {
                channel: 15,
                promiscuous: false,
                pan_id: Some(0x4242),
                short_addr: Some(0x2323),
                ..Default::default()
            });

            println!("Active for {}s", ACTIVE_PHASE_MS / 1000);
            for _ in 0..ACTIVE_PHASE_MS / FRAME_INTERVAL_MS {
                seq_number = seq_number.wrapping_add(1);

                match ieee802154.transmit(
                    &Frame {
                        header: Header {
                            frame_type: FrameType::Data,
                            frame_pending: false,
                            ack_request: false,
                            pan_id_compress: false,
                            seq_no_suppress: false,
                            ie_present: false,
                            version: FrameVersion::Ieee802154_2003,
                            seq: seq_number,
                            destination: Some(Address::Short(PanId(0xffff), ShortAddress(0xffff))),
                            source: None,
                            auxiliary_security_header: None,
                        },
                        content: FrameContent::Data,
                        payload: b"re-init broadcast".to_vec(),
                        footer: [0u8; 2],
                    },
                    true,
                ) {
                    Ok(()) => println!("Broadcast frame with seq {seq_number} sent"),
                    Err(e) => println!("Failed to send frame with seq {seq_number}: {e:?}"),
                }

                delay.delay_millis(FRAME_INTERVAL_MS);
            }
        }

        println!(
            "Radio deinitialized, inactive for {}s (measure power now)",
            INACTIVE_PHASE_MS / 1000
        );
        delay.delay_millis(INACTIVE_PHASE_MS);
    }
}
