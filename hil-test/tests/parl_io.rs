//! PARL_IO test

//% CHIPS: esp32c6 esp32h2
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    dma::{DmaChannel0, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{AnyPin, Pin},
    parl_io::{
        BitPackOrder,
        ClkOutPin,
        EnableMode,
        ParlIoFullDuplex,
        RxClkInPin,
        RxFourBits,
        RxPinConfigWithValidPin,
        SampleEdge,
        TxFourBits,
        TxPinConfigWithValidPin,
    },
    peripherals::PARL_IO,
    time::RateExtU32,
};
use hil_test as _;

struct Context {
    parl_io: PARL_IO,
    dma_channel: DmaChannel0,
    clock_pin: AnyPin,
    valid_pin: AnyPin,
    data_pins: [AnyPin; 4],
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let dma_channel = peripherals.DMA_CH0;

        let parl_io = peripherals.PARL_IO;

        Context {
            parl_io,
            dma_channel,
            clock_pin: peripherals.GPIO11.degrade(),
            valid_pin: peripherals.GPIO10.degrade(),
            data_pins: [
                peripherals.GPIO1.degrade(),
                peripherals.GPIO0.degrade(),
                peripherals.GPIO14.degrade(),
                peripherals.GPIO23.degrade(),
            ],
        }
    }

    #[test]
    fn test_parl_io_rx_can_read_tx(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = ctx.clock_pin.split();
        let (valid_rx, valid_tx) = ctx.valid_pin.split();
        let [(d0_rx, d0_tx), (d1_rx, d1_tx), (d2_rx, d2_tx), (d3_rx, d3_tx)] =
            ctx.data_pins.map(|pin| pin.split());

        let tx_pins = TxFourBits::new(d0_tx, d1_tx, d2_tx, d3_tx);
        let rx_pins = RxFourBits::new(d0_rx, d1_rx, d2_rx, d3_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let mut rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let mut clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIoFullDuplex::new(ctx.parl_io, ctx.dma_channel, 40.MHz()).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                0,
                SampleEdge::Normal,
                BitPackOrder::Lsb,
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(&mut rx_pins, &mut clock_in_pin, BitPackOrder::Lsb, None)
            .unwrap();

        for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *b = i as u8;
        }

        let rx_transfer = pio_rx.read(Some(dma_rx_buf.len()), dma_rx_buf).unwrap();
        let tx_transfer = pio_tx.write(dma_tx_buf.len(), dma_tx_buf).unwrap();
        (_, _, dma_tx_buf) = tx_transfer.wait();
        (_, _, dma_rx_buf) = rx_transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), dma_tx_buf.as_slice());
    }
}
