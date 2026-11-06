//! The Goodix touch controller (GT911 or GT1151) on the I2C bus.

use esp_hal::{Async, i2c::master::I2c};
use gt911::{Error, GET_TOUCH_BUF_SIZE, Gt911};

use crate::display::{HEIGHT, WIDTH};

pub struct Touch {
    i2c: I2c<'static, Async>,
    controller: Gt911<I2c<'static, Async>>,
    address: u8,
    /// Whether a finger was on the panel at the last read.
    down: bool,
}

impl Touch {
    /// Finds the controller at either of its I2C addresses.
    pub async fn new(mut i2c: I2c<'static, Async>) -> Self {
        for address in [0x14, 0x5d] {
            let controller = Gt911::new(address);
            let mut buffer = [0; GET_TOUCH_BUF_SIZE];
            // `init` would reject the GT1151, so read a touch to find the controller.
            if !matches!(
                controller.get_touch(&mut i2c, &mut buffer).await,
                Err(Error::I2C(_))
            ) {
                return Self {
                    i2c,
                    controller,
                    address,
                    down: false,
                };
            }
        }
        panic!("Goodix touch controller not found");
    }

    pub fn address(&self) -> u8 {
        self.address
    }

    /// Returns where a finger touched the panel since the last call, if one did.
    pub async fn tap(&mut self) -> Option<(i32, i32)> {
        let mut buffer = [0; GET_TOUCH_BUF_SIZE];
        let point = self
            .controller
            .get_touch(&mut self.i2c, &mut buffer)
            .await
            .ok()?;
        let was_down = core::mem::replace(&mut self.down, point.is_some());
        let point = point.filter(|_| !was_down)?;
        let x = point.x.min(WIDTH as u16 - 1);
        let y = point.y.min(HEIGHT as u16 - 1);
        Some((i32::from(x), i32::from(y)))
    }
}
