//! Misc UART TX/RX regression tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        gpio::Flex,
        uart::{self, UartRx, UartTx},
    };
    use hil_test as _;

    #[test]
    fn test_that_creating_tx_does_not_cause_a_pulse() {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (rx, tx) = hil_test::common_test_pins!(peripherals);

        let mut rx = UartRx::new(peripherals.UART1, uart::Config::default())
            .unwrap()
            .with_rx(rx);

        // start reception
        let mut buf = [0u8; 1];
        _ = rx.read_bytes(&mut buf);

        // set up TX and send a byte
        let mut tx = UartTx::new(peripherals.UART0, uart::Config::default())
            .unwrap()
            .with_tx(tx);

        tx.flush();
        tx.write_bytes(&[0x42]).unwrap();
        rx.read_bytes(&mut buf).unwrap();

        assert_eq!(buf[0], 0x42);
    }
}
