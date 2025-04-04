//! PARL_IO test

//% CHIPS: esp32c6 esp32h2
//% FEATURES: unstable

#![no_std]
#![no_main]

#[cfg(not(esp32h2))]
use esp_hal::parl_io::{RxSixteenBits, TxSixteenBits};
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
        RxEightBits,
        RxFourBits,
        RxOneBit,
        RxPinConfigWithValidPin,
        RxTwoBits,
        SampleEdge,
        TxEightBits,
        TxFourBits,
        TxOneBit,
        TxPinConfigWithValidPin,
        TxTwoBits,
    },
    peripherals::PARL_IO,
    time::Rate,
};
use hil_test as _;

struct Context {
    parl_io: PARL_IO<'static>,
    dma_channel: DmaChannel0<'static>,
    clock_pin: AnyPin<'static>,
    valid_pin: AnyPin<'static>,
    data_pins: [AnyPin<'static>; 8],
    #[cfg(esp32c6)]
    extra_data_pins: [AnyPin<'static>; 7],
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
            #[cfg(esp32h2)]
            clock_pin: peripherals.GPIO13.degrade(),
            #[cfg(esp32h2)]
            valid_pin: peripherals.GPIO24.degrade(),
            #[cfg(esp32c6)]
            clock_pin: peripherals.GPIO16.degrade(),
            #[cfg(esp32c6)]
            valid_pin: peripherals.GPIO17.degrade(),
            data_pins: [
                peripherals.GPIO2.degrade(),
                peripherals.GPIO4.degrade(),
                peripherals.GPIO10.degrade(),
                peripherals.GPIO11.degrade(),
                peripherals.GPIO14.degrade(),
                peripherals.GPIO23.degrade(),
                peripherals.GPIO0.degrade(),
                peripherals.GPIO1.degrade(),
            ],
            #[cfg(esp32c6)]
            extra_data_pins: [
                peripherals.GPIO18.degrade(),
                peripherals.GPIO19.degrade(),
                peripherals.GPIO20.degrade(),
                peripherals.GPIO22.degrade(),
                peripherals.GPIO8.degrade(),
                peripherals.GPIO9.degrade(),
                peripherals.GPIO15.degrade(),
            ],
        }
    }

    #[test]
    fn test_parl_io_rx_can_read_tx_in_1_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = ctx.clock_pin.split();
        let (valid_rx, valid_tx) = ctx.valid_pin.split();
        let [(d0_rx, d0_tx), ..] = ctx.data_pins.map(|pin| pin.split());

        let tx_pins = TxOneBit::new(d0_tx);
        let rx_pins = RxOneBit::new(d0_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let mut rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let mut clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIoFullDuplex::new(ctx.parl_io, ctx.dma_channel, Rate::from_mhz(40)).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                0,
                SampleEdge::Invert,
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

        let rx_transfer = pio_rx
            .read(Some(dma_rx_buf.len()), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let tx_transfer = pio_tx
            .write(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, _, dma_tx_buf) = tx_transfer.wait();
        (_, _, dma_rx_buf) = rx_transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), dma_tx_buf.as_slice());
    }

    #[test]
    fn test_parl_io_rx_can_read_tx_in_2_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = ctx.clock_pin.split();
        let (valid_rx, valid_tx) = ctx.valid_pin.split();
        let [(d0_rx, d0_tx), (d1_rx, d1_tx), ..] = ctx.data_pins.map(|pin| pin.split());

        let tx_pins = TxTwoBits::new(d0_tx, d1_tx);
        let rx_pins = RxTwoBits::new(d0_rx, d1_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let mut rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let mut clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIoFullDuplex::new(ctx.parl_io, ctx.dma_channel, Rate::from_mhz(40)).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                0,
                SampleEdge::Invert,
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

        let rx_transfer = pio_rx
            .read(Some(dma_rx_buf.len()), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let tx_transfer = pio_tx
            .write(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, _, dma_tx_buf) = tx_transfer.wait();
        (_, _, dma_rx_buf) = rx_transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), dma_tx_buf.as_slice());
    }

    #[test]
    fn test_parl_io_rx_can_read_tx_in_4_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = ctx.clock_pin.split();
        let (valid_rx, valid_tx) = ctx.valid_pin.split();
        let [(d0_rx, d0_tx), (d1_rx, d1_tx), (d2_rx, d2_tx), (d3_rx, d3_tx), ..] =
            ctx.data_pins.map(|pin| pin.split());

        let tx_pins = TxFourBits::new(d0_tx, d1_tx, d2_tx, d3_tx);
        let rx_pins = RxFourBits::new(d0_rx, d1_rx, d2_rx, d3_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let mut rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let mut clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIoFullDuplex::new(ctx.parl_io, ctx.dma_channel, Rate::from_mhz(40)).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                0,
                SampleEdge::Invert,
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

        let rx_transfer = pio_rx
            .read(Some(dma_rx_buf.len()), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let tx_transfer = pio_tx
            .write(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, _, dma_tx_buf) = tx_transfer.wait();
        (_, _, dma_rx_buf) = rx_transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), dma_tx_buf.as_slice());
    }

    #[test]
    fn test_parl_io_rx_can_read_tx_in_8_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 250;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = ctx.clock_pin.split();
        let (valid_rx, valid_tx) = ctx.valid_pin.split();
        let [(d0_rx, d0_tx), (d1_rx, d1_tx), (d2_rx, d2_tx), (d3_rx, d3_tx), (d4_rx, d4_tx), (d5_rx, d5_tx), (d6_rx, d6_tx), (d7_rx, d7_tx)] =
            ctx.data_pins.map(|pin| pin.split());

        let tx_pins = TxEightBits::new(d0_tx, d1_tx, d2_tx, d3_tx, d4_tx, d5_tx, d6_tx, d7_tx);
        #[cfg_attr(not(esp32h2), allow(unused_mut))]
        let mut rx_pins = RxEightBits::new(d0_rx, d1_rx, d2_rx, d3_rx, d4_rx, d5_rx, d6_rx, d7_rx);

        #[cfg(not(esp32h2))]
        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        #[cfg(not(esp32h2))]
        let mut rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let mut clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIoFullDuplex::new(ctx.parl_io, ctx.dma_channel, Rate::from_khz(50)).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                0xFFFF,
                SampleEdge::Invert,
                BitPackOrder::Msb,
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(&mut rx_pins, &mut clock_in_pin, BitPackOrder::Msb, None)
            .unwrap();

        for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *b = i as u8;
        }

        let rx_transfer = pio_rx
            .read(Some(dma_rx_buf.len()), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let tx_transfer = pio_tx
            .write(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, _, dma_tx_buf) = tx_transfer.wait();
        (_, _, dma_rx_buf) = rx_transfer.wait();

        if cfg!(esp32h2) {
            // skip the first few idle bytes received by the RX unit. This is needed due to
            // the lack of an enable pin.
            let number_of_invalid_bytes = dma_rx_buf.as_slice().partition_point(|&b| b == 0xFF);

            let received_data = &dma_rx_buf.as_slice()[number_of_invalid_bytes..];
            let successfully_transmitted_data = &dma_tx_buf.as_slice()[..received_data.len()];

            // This is unfortunately a flakey test. This can be improved by dropping the
            // frequency or (as a last resort) reducing the number here.
            assert!(received_data.len() > 200);

            assert_eq!(successfully_transmitted_data, received_data);
        } else {
            assert_eq!(dma_tx_buf.as_slice(), dma_rx_buf.as_slice());
        }
    }

    #[cfg(not(esp32h2))]
    #[test]
    fn test_parl_io_rx_can_read_tx_in_16_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 250;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = ctx.clock_pin.split();
        let [(d0_rx, d0_tx), (d1_rx, d1_tx), (d2_rx, d2_tx), (d3_rx, d3_tx), (d4_rx, d4_tx), (d5_rx, d5_tx), (d6_rx, d6_tx), (d7_rx, d7_tx)] =
            ctx.data_pins.map(|pin| pin.split());
        let [(d8_rx, d8_tx), (d9_rx, d9_tx), (d10_rx, d10_tx), (d11_rx, d11_tx), (d12_rx, d12_tx), (d13_rx, d13_tx), (d14_rx, d14_tx)] =
            ctx.extra_data_pins.map(|pin| pin.split());
        let (d15_rx, d15_tx) = ctx.valid_pin.split();

        let tx_pins = TxSixteenBits::new(
            d0_tx, d1_tx, d2_tx, d3_tx, d4_tx, d5_tx, d6_tx, d7_tx, d8_tx, d9_tx, d10_tx, d11_tx,
            d12_tx, d13_tx, d14_tx, d15_tx,
        );
        let mut rx_pins = RxSixteenBits::new(
            d0_rx, d1_rx, d2_rx, d3_rx, d4_rx, d5_rx, d6_rx, d7_rx, d8_rx, d9_rx, d10_rx, d11_rx,
            d12_rx, d13_rx, d14_rx, d15_rx,
        );

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let mut clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIoFullDuplex::new(ctx.parl_io, ctx.dma_channel, Rate::from_khz(50)).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                0xFFFF,
                SampleEdge::Invert,
                BitPackOrder::Msb,
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(&mut rx_pins, &mut clock_in_pin, BitPackOrder::Msb, None)
            .unwrap();

        for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *b = (i | 0) as u8;
        }

        let rx_transfer = pio_rx
            .read(Some(dma_rx_buf.len()), dma_rx_buf)
            .map_err(|e| e.0)
            .unwrap();
        let tx_transfer = pio_tx
            .write(dma_tx_buf.len(), dma_tx_buf)
            .map_err(|e| e.0)
            .unwrap();
        (_, _, dma_tx_buf) = tx_transfer.wait();
        (_, _, dma_rx_buf) = rx_transfer.wait();

        // skip the first few idle bytes received by the RX unit. This is needed due to
        // the lack of an enable pin.
        let number_of_invalid_bytes = dma_rx_buf.as_slice().partition_point(|&b| b == 0xFF);

        let received_data = &dma_rx_buf.as_slice()[number_of_invalid_bytes..];
        let successfully_transmitted_data = &dma_tx_buf.as_slice()[..received_data.len()];

        // This is unfortunately a flakey test. This can be improved by dropping the
        // frequency or (as a last resort) reducing the number here.
        assert!(received_data.len() > 200);

        assert_eq!(successfully_transmitted_data, received_data);
    }
}
