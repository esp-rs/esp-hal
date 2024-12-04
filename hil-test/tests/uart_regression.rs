//! Misc UART TX/RX regression tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        gpio::OutputPin,
        prelude::*,
        uart::{self, UartRx, UartTx},
    };
    use hil_test as _;
    use nb::block;

    #[test]
    fn test_that_creating_tx_does_not_cause_a_pulse() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, mut tx) = hil_test::common_test_pins!(peripherals);

        let mut rx = UartRx::new(peripherals.UART1, uart::Config::default(), rx).unwrap();

        // start reception
        _ = rx.read_byte(); // this will just return WouldBlock

        unsafe { tx.set_output_high(false, esp_hal::Internal::conjure()) };

        // set up TX and send a byte
        let mut tx = UartTx::new(peripherals.UART0, uart::Config::default(), tx).unwrap();

        tx.flush_tx().unwrap();
        tx.write_bytes(&[0x42]).unwrap();
        let read = block!(rx.read_byte());

        assert_eq!(read, Ok(0x42));
    }
}
