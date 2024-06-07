//! I2S Loopback Test
//!
//! It's assumed GPIO2 is connected to GPIO4
//!
//! This test uses I2S TX to transmit known data to I2S RX (forced to slave mode
//! with loopback mode enabled). It's using circular DMA mode

//! H2 is disabled because of https://github.com/esp-rs/esp-hal/issues/1637

//% CHIPS: esp32c3 esp32c6 esp32s3

#![no_std]
#![no_main]

use defmt_rtt as _;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::Io,
    i2s::{DataFormat, I2s, I2sReadDma, I2sWriteDma, Standard},
    peripheral::Peripheral,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
};

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

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        let (tx_buffer, mut tx_descriptors, mut rx_buffer, mut rx_descriptors) =
            dma_buffers!(32000, 32000);

        let i2s = I2s::new(
            peripherals.I2S0,
            Standard::Philips,
            DataFormat::Data16Channel16,
            441000.Hz(),
            dma_channel.configure(
                false,
                &mut tx_descriptors,
                &mut rx_descriptors,
                DmaPriority::Priority0,
            ),
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
            .with_din(io.pins.gpio4)
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
            i = i.wrapping_add(1);
        }

        let mut rcv = [0u8; 15000];
        let mut filler = [0x1u8; 10000];

        let mut rx_transfer = i2s_rx.read_dma_circular(&mut rx_buffer).unwrap();
        // trying to pop data before calling `available` should just do nothing
        assert_eq!(0, rx_transfer.pop(&mut rcv[..100]).unwrap());

        let mut tx_transfer = i2s_tx.write_dma_circular(&tx_buffer).unwrap();

        'outer: loop {
            let tx_avail = tx_transfer.available();

            // make sure there are more than one descriptor buffers ready to push
            if tx_avail > 5000 {
                for b in &mut filler[0..tx_avail].iter_mut() {
                    *b = i;
                    i = i.wrapping_add(1);
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
            if rx_avail > 5000 {
                // trying to pop less data than available is an error
                assert_eq!(
                    Err(esp_hal::dma::DmaError::BufferTooSmall),
                    rx_transfer.pop(&mut rcv[..rx_avail / 2])
                );

                rcv.fill(0);
                rx_transfer.pop(&mut rcv[..rx_avail]).unwrap();
                for &b in &rcv[..rx_avail] {
                    if b != check_i {
                        failed = true;
                        break 'outer;
                    }
                    check_i = check_i.wrapping_add(1);
                }
                iteration += 1;
            }

            if iteration > 100 {
                break;
            }
        }

        assert!(!failed);
    }
}
