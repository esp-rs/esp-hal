use crate::{peripheral::PeripheralRef, peripherals::LCD_CAM};

pub struct Cam<'d> {
    pub(crate) _lcd_cam: PeripheralRef<'d, LCD_CAM>,
}
