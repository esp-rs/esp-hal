//! The Goodix touch controller (GT911 or GT1151) on the I2C bus.

use esp_hal::{
    Async,
    i2c::master::{Config, I2c},
    time::Rate,
};

use crate::{
    TouchResources,
    display::{HEIGHT, WIDTH},
};

/// The status register. Bit 7 tells that new data is ready, bits 0 to 3 count the
/// touch points. Writing 0 acknowledges the data.
const STATUS: [u8; 2] = [0x81, 0x4e];
/// The coordinates of the first touch point.
const POINT: [u8; 2] = [0x81, 0x50];

pub enum TouchEvent {
    Up,
    Down(i32, i32),
}

pub struct Touch {
    i2c: I2c<'static, Async>,
    address: u8,
}

impl Touch {
    /// Finds the controller at either of its I2C addresses.
    pub async fn new(r: TouchResources<'static>) -> Self {
        let mut i2c = I2c::new(r.i2c, Config::default().with_frequency(Rate::from_khz(400)))
            .unwrap()
            .with_sda(r.sda)
            .with_scl(r.scl)
            .into_async();

        for address in [0x14, 0x5d] {
            let mut status = [0u8; 1];
            if i2c
                .write_read_async(address, &STATUS, &mut status)
                .await
                .is_ok()
            {
                return Self { i2c, address };
            }
        }
        panic!("Goodix touch controller not found");
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    /// Reads the controller. Returns `None` when it has no new data.
    pub async fn read(&mut self) -> Option<TouchEvent> {
        let mut status = [0u8; 1];
        self.i2c
            .write_read_async(self.address, &STATUS, &mut status)
            .await
            .ok()?;
        if status[0] & 0x80 == 0 {
            return None;
        }

        let event = if status[0] & 0x0f == 0 {
            Some(TouchEvent::Up)
        } else {
            let mut point = [0u8; 4];
            self.i2c
                .write_read_async(self.address, &POINT, &mut point)
                .await
                .ok()
                .map(|_| {
                    let x = u16::from_le_bytes([point[0], point[1]]).min(WIDTH as u16 - 1);
                    let y = u16::from_le_bytes([point[2], point[3]]).min(HEIGHT as u16 - 1);
                    TouchEvent::Down(x as i32, y as i32)
                })
        };

        let _ = self
            .i2c
            .write_async(self.address, &[STATUS[0], STATUS[1], 0])
            .await;
        event
    }
}
