//! I2S Loopback Test
//!
//! It's assumed GPIO2 is connected to GPIO3
//!
//! This test uses I2S TX to transmit known data to I2S RX (forced to slave mode
//! with loopback mode enabled). It's using circular DMA mode

//% CHIPS: esp32c3 esp32c6 esp32s3 esp32h2

#![no_std]
#![no_main]

use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    i2s::{DataFormat, I2s, I2sReadDma, I2sWriteDma, Standard},
    peripheral::Peripheral,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};
use hil_test as _;

// choose values which DON'T restart on every descriptor buffer's start
const ADD: u8 = 5;
const CUT_OFF: u8 = 113;

#[cfg(test)]
#[embedded_test::tests]
mod tests {

    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn test_i2s_loopback() {
        let peripherals = Peripherals::take();
        let system = SystemControl::new(peripherals.SYSTEM);
        let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

        let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

        let delay = Delay::new(&clocks);

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (tx_buffer, tx_descriptors, mut rx_buffer, rx_descriptors) = dma_buffers!(16000, 16000);

        let i2s = I2s::new(
            peripherals.I2S0,
            Standard::Philips,
            DataFormat::Data16Channel16,
            16000.Hz(),
            dma_channel.configure(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
            &clocks,
        );

        let mut i2s_tx = i2s
            .i2s_tx
            .with_bclk(unsafe { io.pins.gpio0.clone_unchecked() })
            .with_ws(unsafe { io.pins.gpio1.clone_unchecked() })
            .with_dout(unsafe { io.pins.gpio2.clone_unchecked() })
            .build();

        let mut i2s_rx = i2s
            .i2s_rx
            .with_bclk(io.pins.gpio0)
            .with_ws(io.pins.gpio1)
            .with_din(io.pins.gpio3)
            .build();

        // enable loopback testing
        unsafe {
            let i2s = esp_hal::peripherals::I2S0::steal();
            i2s.tx_conf().modify(|_, w| w.sig_loopback().set_bit());

            i2s.rx_conf().modify(|_, w| w.rx_slave_mod().set_bit());

            i2s.tx_conf().modify(|_, w| w.tx_update().clear_bit());
            i2s.tx_conf().modify(|_, w| w.tx_update().set_bit());

            i2s.rx_conf().modify(|_, w| w.rx_update().clear_bit());
            i2s.rx_conf().modify(|_, w| w.rx_update().set_bit());
        }

        let mut iteration = 0;
        let mut failed = false;
        let mut check_i: u8 = 0;
        let mut i = 0;
        for b in tx_buffer.iter_mut() {
            *b = i;
            i = (i + ADD) % CUT_OFF;
        }

        let mut rcv = [0u8; 11000];
        let mut filler = [0x1u8; 12000];

        let mut rx_transfer = i2s_rx.read_dma_circular(&mut rx_buffer).unwrap();
        // trying to pop data before calling `available` should just do nothing
        assert_eq!(0, rx_transfer.pop(&mut rcv[..100]).unwrap());

        // no data available yet
        assert_eq!(0, rx_transfer.available());

        let mut tx_transfer = i2s_tx.write_dma_circular(&tx_buffer).unwrap();

        'outer: loop {
            let tx_avail = tx_transfer.available();

            // make sure there are more than one descriptor buffers ready to push
            if tx_avail > 5000 {
                for b in &mut filler[0..tx_avail].iter_mut() {
                    *b = i;
                    i = (i + ADD) % CUT_OFF;
                }
                tx_transfer.push(&filler[0..tx_avail]).unwrap();
            }

            // test calling available multiple times doesn't break anything
            rx_transfer.available();
            rx_transfer.available();
            rx_transfer.available();
            rx_transfer.available();
            rx_transfer.available();
            rx_transfer.available();
            let rx_avail = rx_transfer.available();

            // make sure there are more than one descriptor buffers ready to pop
            if rx_avail > 0 {
                // trying to pop less data than available is an error
                assert_eq!(
                    Err(esp_hal::dma::DmaError::BufferTooSmall),
                    rx_transfer.pop(&mut rcv[..rx_avail / 2])
                );

                rcv.fill(0xff);
                let len = rx_transfer.pop(&mut rcv).unwrap();
                assert!(len > 0);

                for &b in &rcv[..len] {
                    if b != check_i {
                        failed = true;
                        break 'outer;
                    }
                    check_i = (check_i + ADD) % CUT_OFF;
                }

                iteration += 1;

                if iteration == 1 {
                    // delay to make it likely `available` will need to handle more than one
                    // descriptor next time
                    delay.delay_millis(160);
                }
            }

            if iteration > 30 {
                break;
            }
        }

        assert!(!failed);
    }
}
