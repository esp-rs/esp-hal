//! USB Serial JTAG tests

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use esp_hal::{prelude::*, timer::timg::TimerGroup, usb_serial_jtag::UsbSerialJtag};
    use hil_test as _;

    #[test]
    fn creating_peripheral_does_not_break_debug_connection() {
        let (peripherals, clocks) = esp_hal::init(Config::default());

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        esp_hal_embassy::init(&clocks, timg0.timer0);

        _ = UsbSerialJtag::new_async(peripherals.USB_DEVICE).split();
    }
}
