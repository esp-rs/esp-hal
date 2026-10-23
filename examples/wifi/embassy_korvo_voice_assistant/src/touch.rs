//! Polls the Goodix touch controller of the LCD sub-board and reports taps on the talk button.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::Timer;
use embedded_graphics::prelude::*;
use esp_hal::{
    Async,
    i2c::master::{Error, I2c},
};

use crate::display::BUTTON;

/// Goodix controllers answer on one of these addresses, depending on the INT level at reset.
const ADDRESSES: [u8; 2] = [0x5D, 0x14];
/// Bit 7: new data is ready. Bits 3:0: the number of touch points.
const STATUS: u16 = 0x814E;
/// X and Y of the first touch point, little-endian.
const POINT_1: u16 = 0x8150;

static TAPS: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Waits for the next tap on the talk button.
pub async fn wait_for_tap() {
    TAPS.reset();
    TAPS.wait().await;
}

enum Report {
    Unchanged,
    Released,
    Touched(Point),
}

#[embassy_executor::task]
pub async fn run(mut i2c: I2c<'static, Async>) {
    let mut address = None;
    for candidate in ADDRESSES {
        if i2c
            .write_read_async(candidate, &STATUS.to_be_bytes(), &mut [0])
            .await
            .is_ok()
        {
            address = Some(candidate);
        }
    }
    let address = address.expect("No touch controller found");
    log::info!("Touch controller at {address:#04x}");

    let mut touching = false;
    loop {
        Timer::after_millis(30).await;
        match read(&mut i2c, address).await {
            Ok(Report::Unchanged) => {}
            Ok(Report::Released) => touching = false,
            Ok(Report::Touched(point)) => {
                if !touching && BUTTON.contains(point) {
                    TAPS.signal(());
                }
                touching = true;
            }
            Err(error) => log::warn!("Touch read failed: {error:?}"),
        }
    }
}

async fn read(i2c: &mut I2c<'static, Async>, address: u8) -> Result<Report, Error> {
    let mut status = [0];
    i2c.write_read_async(address, &STATUS.to_be_bytes(), &mut status)
        .await?;
    if status[0] & 0x80 == 0 {
        return Ok(Report::Unchanged);
    }

    let report = if status[0] & 0x0F == 0 {
        Report::Released
    } else {
        let mut point = [0; 4];
        i2c.write_read_async(address, &POINT_1.to_be_bytes(), &mut point)
            .await?;
        let x = u16::from_le_bytes([point[0], point[1]]);
        let y = u16::from_le_bytes([point[2], point[3]]);
        Report::Touched(Point::new(i32::from(x), i32::from(y)))
    };

    // Hand the buffer back to the controller.
    let [high, low] = STATUS.to_be_bytes();
    i2c.write_async(address, &[high, low, 0]).await?;
    Ok(report)
}
