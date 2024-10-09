//! Misc UART TX/RX regression tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use esp_hal::{
        gpio::Io,
        prelude::*,
        uart::{UartRx, UartTx},
    };
    use hil_test as _;
    use nb::block;

    #[test]
    #[timeout(3)]
    fn test_that_creating_tx_does_not_cause_a_pulse() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let (rx, tx) = hil_test::common_test_pins!(io);

        let mut rx = UartRx::new(peripherals.UART1, rx).unwrap();

        // start reception
        _ = rx.read_byte(); // this will just return WouldBlock

        // set up TX and send a byte
        let mut tx = UartTx::new(peripherals.UART0, tx).unwrap();

        tx.flush_tx().unwrap();
        tx.write_bytes(&[0x42]).unwrap();
        let read = block!(rx.read_byte());

        assert_eq!(read, Ok(0x42));
    }
}
