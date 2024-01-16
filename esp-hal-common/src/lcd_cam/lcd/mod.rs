use crate::{peripheral::PeripheralRef, peripherals::LCD_CAM};

pub mod i8080;

pub struct Lcd<'d> {
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,
}
