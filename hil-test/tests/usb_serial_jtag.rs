//! USB Serial JTAG tests

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use esp_hal::{
        clock::ClockControl,
        peripherals::Peripherals,
        system::SystemControl,
        timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
        usb_serial_jtag::UsbSerialJtag,
    };
    use hil_test as _;

    // When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
    macro_rules! mk_static {
        ($t:ty,$val:expr) => {{
            static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
            #[deny(unused_attributes)]
            let x = STATIC_CELL.uninit().write(($val));
            x
        }};
    }

    #[test]
    fn creating_peripheral_does_not_break_debug_connection() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
        let timer0: ErasedTimer = timg0.timer0.into();
        let timers = [OneShotTimer::new(timer0)];
        let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
        esp_hal_embassy::init(&clocks, timers);

        _ = UsbSerialJtag::new_async(peripherals.USB_DEVICE).split();
    }
}
