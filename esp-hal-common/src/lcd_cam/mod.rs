//! LCD_CAM

pub mod cam;
pub mod lcd;

use crate::{
    lcd_cam::{cam::Cam, lcd::Lcd},
    peripheral::Peripheral,
    peripherals::LCD_CAM,
    system,
    system::PeripheralClockControl,
};

pub struct LcdCam<'d> {
    pub lcd: Lcd<'d>,
    pub cam: Cam<'d>,
}

impl<'d> LcdCam<'d> {
    pub fn new(lcd_cam: impl Peripheral<P = LCD_CAM> + 'd) -> Self {
        crate::into_ref!(lcd_cam);

        PeripheralClockControl::enable(system::Peripheral::LcdCam);

        Self {
            lcd: Lcd {
                lcd_cam: unsafe { lcd_cam.clone_unchecked() },
            },
            cam: Cam {
                _lcd_cam: unsafe { lcd_cam.clone_unchecked() },
            },
        }
    }
}
