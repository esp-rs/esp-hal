//! PARL_IO test

//% CHIP_FILTER: parl_io_driver_supported
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

type DmaChannel = cfg_select! {
    parl_io_dma_engine = "AXI_GDMA" => { esp_hal::peripherals::DMA_AXI_CH0<'static> },
    _ => { esp_hal::peripherals::DMA_CH0<'static> },
};

macro_rules! dma_channel {
    ($peripherals:expr) => {
        cfg_select! {
            parl_io_dma_engine = "AXI_GDMA" => $peripherals.DMA_AXI_CH0,
            _ => $peripherals.DMA_CH0,
        }
    };
}

// 16 TX lines without data, and the valid signal.
#[cfg(parl_io_max_data_width = "16")]
macro_rules! sixteen_bit_pins_with_valid {
    ($valid:expr) => {
        cfg_select! {
            parl_io_has_tx_cs => TxPinConfigWithValidPin::new(
                TxSixteenBits::new(
                    NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin,
                    NoPin, NoPin, NoPin, NoPin, NoPin,
                ),
                $valid,
            ),
            _ => TxPinConfigIncludingValidPin::new(TxSixteenBits::new(
                NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin,
                NoPin, NoPin, NoPin, $valid,
            )),
        }
    };
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    #[cfg(any(parl_io_has_tx_cs, parl_io_max_data_width = "16"))]
    use esp_hal::gpio::NoPin;
    #[cfg(not(parl_io_has_tx_cs))]
    use esp_hal::parl_io::TxPinConfigIncludingValidPin;
    #[cfg(parl_io_max_data_width = "16")]
    use esp_hal::parl_io::{RxSixteenBits, TxSixteenBits};
    use esp_hal::{
        dma_rx_buffer,
        dma_tx_buffer,
        gpio::{AnyPin, Pin},
        parl_io::{
            BitPackOrder,
            ClkOutPin,
            EnableMode,
            ParlIo,
            RxClkInPin,
            RxConfig,
            RxEightBits,
            RxFourBits,
            RxOneBit,
            RxPinConfigIncludingValidPin,
            RxPinConfigWithValidPin,
            RxTwoBits,
            SampleEdge,
            TxConfig,
            TxEightBits,
            TxFourBits,
            TxOneBit,
            TxPinConfigWithValidPin,
            TxTwoBits,
        },
        peripherals::PARL_IO,
        time::Rate,
    };

    use super::DmaChannel;

    struct Context {
        parl_io: PARL_IO<'static>,
        dma_channel: DmaChannel,
        clock_pin: AnyPin<'static>,
        valid_pin: AnyPin<'static>,
        data_pins: [AnyPin<'static>; 8],
    }

    const MAX_CLK_RATE: Rate = Rate::from_mhz(40);

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let dma_channel = dma_channel!(peripherals);

        let parl_io = peripherals.PARL_IO;

        let (valid_pin, clock_pin, data_pins) = cfg_select! {
            esp32c5 => {
                // 27 is RGB, 13 and 14 is USB, 4 is tied to 5, 9 is tied to 10 (only 9 is driven)
                (
                    peripherals.GPIO0.degrade(),
                    peripherals.GPIO1.degrade(),
                    [
                        peripherals.GPIO2.degrade(),
                        peripherals.GPIO3.degrade(),
                        peripherals.GPIO23.degrade(),
                        peripherals.GPIO24.degrade(),
                        peripherals.GPIO6.degrade(),
                        peripherals.GPIO7.degrade(),
                        peripherals.GPIO8.degrade(),
                        peripherals.GPIO9.degrade(),
                    ],
                )
            }
            esp32c6 => {
                // 8 is RGB, 12 and 13 is USB, 2 is tied to 3 (only 2 is driven)
                (
                    peripherals.GPIO0.degrade(),
                    peripherals.GPIO1.degrade(),
                    [
                        peripherals.GPIO2.degrade(),
                        peripherals.GPIO4.degrade(),
                        peripherals.GPIO5.degrade(),
                        peripherals.GPIO6.degrade(),
                        peripherals.GPIO7.degrade(),
                        peripherals.GPIO8.degrade(),
                        peripherals.GPIO9.degrade(),
                        peripherals.GPIO10.degrade(),
                    ],
                )
            }
            esp32h2 => {
                // 8 is RGB, 26 and 27 is USB, 12 and 22 are I2C, 2 is tied to 3, 10 is tied to 11.
                // 23 is UART0 RX. The 8-bit test does not drive the last pin.
                (
                    peripherals.GPIO0.degrade(),
                    peripherals.GPIO1.degrade(),
                    [
                        peripherals.GPIO2.degrade(),
                        peripherals.GPIO4.degrade(),
                        peripherals.GPIO5.degrade(),
                        peripherals.GPIO8.degrade(),
                        peripherals.GPIO9.degrade(),
                        peripherals.GPIO13.degrade(),
                        peripherals.GPIO25.degrade(),
                        peripherals.GPIO23.degrade(),
                    ],
                )
            }
            esp32h4 => {
                // 2 and 3, 19 and 20 are tied, 6 to 12 are flash, 13 and 14 are USB, 23 and 24 are
                // UART0, 26 and 27 are I2C.
                (
                    peripherals.GPIO4.degrade(),
                    peripherals.GPIO5.degrade(),
                    [
                        peripherals.GPIO0.degrade(),
                        peripherals.GPIO1.degrade(),
                        peripherals.GPIO15.degrade(),
                        peripherals.GPIO16.degrade(),
                        peripherals.GPIO17.degrade(),
                        peripherals.GPIO18.degrade(),
                        peripherals.GPIO25.degrade(),
                        peripherals.GPIO28.degrade(),
                    ],
                )
            }
            esp32p4 => {
                // 2 and 3 are I2C, 5 and 6, 46 and 53 are tied, 24 to 27 are USB, 37 and 38 are
                // UART0.
                (
                    peripherals.GPIO20.degrade(),
                    peripherals.GPIO21.degrade(),
                    [
                        peripherals.GPIO22.degrade(),
                        peripherals.GPIO23.degrade(),
                        peripherals.GPIO32.degrade(),
                        peripherals.GPIO33.degrade(),
                        peripherals.GPIO36.degrade(),
                        peripherals.GPIO45.degrade(),
                        peripherals.GPIO47.degrade(),
                        peripherals.GPIO48.degrade(),
                    ],
                )
            }
            esp32s31 => {
                // The pins of the LCD_CAM test.
                (
                    peripherals.GPIO20.degrade(),
                    peripherals.GPIO21.degrade(),
                    [
                        peripherals.GPIO22.degrade(),
                        peripherals.GPIO23.degrade(),
                        peripherals.GPIO24.degrade(),
                        peripherals.GPIO25.degrade(),
                        peripherals.GPIO35.degrade(),
                        peripherals.GPIO42.degrade(),
                        peripherals.GPIO43.degrade(),
                        peripherals.GPIO46.degrade(),
                    ],
                )
            }
        };

        Context {
            parl_io,
            dma_channel,
            clock_pin,
            valid_pin,
            data_pins,
        }
    }

    #[test]
    fn test_parl_io_rx_can_read_tx_in_1_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let mut dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [(d0_rx, d0_tx), ..] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = TxOneBit::new(d0_tx);
        let rx_pins = RxOneBit::new(d0_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        // Sample on the opposite edge of TX. Loopback through the GPIO matrix delays
        // clock and data and same-edge sampling can violate hold time.
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Invert);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Lsb),
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(
                rx_pins,
                clock_in_pin,
                RxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_bit_order(BitPackOrder::Lsb),
            )
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

        let mut dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [(d0_rx, d0_tx), (d1_rx, d1_tx), ..] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = TxTwoBits::new(d0_tx, d1_tx);
        let rx_pins = RxTwoBits::new(d0_rx, d1_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Invert);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Lsb),
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(
                rx_pins,
                clock_in_pin,
                RxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_bit_order(BitPackOrder::Lsb),
            )
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

        let mut dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [
            (d0_rx, d0_tx),
            (d1_rx, d1_tx),
            (d2_rx, d2_tx),
            (d3_rx, d3_tx),
            ..,
        ] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = TxFourBits::new(d0_tx, d1_tx, d2_tx, d3_tx);
        let rx_pins = RxFourBits::new(d0_rx, d1_rx, d2_rx, d3_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Invert);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Lsb),
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(
                rx_pins,
                clock_in_pin,
                RxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_bit_order(BitPackOrder::Lsb),
            )
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

    // On 8-line chips, `RxEightBits` uses data line 7 for the valid signal.
    #[cfg(parl_io_max_data_width = "16")]
    #[test]
    fn test_parl_io_rx_can_read_tx_in_8_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 250;

        let mut dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [
            (d0_rx, d0_tx),
            (d1_rx, d1_tx),
            (d2_rx, d2_tx),
            (d3_rx, d3_tx),
            (d4_rx, d4_tx),
            (d5_rx, d5_tx),
            (d6_rx, d6_tx),
            (d7_rx, d7_tx),
        ] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = TxEightBits::new(d0_tx, d1_tx, d2_tx, d3_tx, d4_tx, d5_tx, d6_tx, d7_tx);
        let rx_pins = RxEightBits::new(d0_rx, d1_rx, d2_rx, d3_rx, d4_rx, d5_rx, d6_rx, d7_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Invert);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_khz(100))
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Msb),
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(
                rx_pins,
                clock_in_pin,
                RxConfig::default()
                    .with_frequency(Rate::from_khz(100))
                    .with_bit_order(BitPackOrder::Msb),
            )
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

    #[cfg(parl_io_max_data_width = "8")]
    #[test]
    fn test_parl_io_rx_can_read_tx_in_8_bit_mode_with_included_valid_pin(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let mut dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [
            (d0_rx, d0_tx),
            (d1_rx, d1_tx),
            (d2_rx, d2_tx),
            (d3_rx, d3_tx),
            (d4_rx, d4_tx),
            (d5_rx, d5_tx),
            (d6_rx, d6_tx),
            _,
        ] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = cfg_select! {
            parl_io_has_tx_cs => TxPinConfigWithValidPin::new(
                TxEightBits::new(d0_tx, d1_tx, d2_tx, d3_tx, d4_tx, d5_tx, d6_tx, NoPin),
                valid_tx,
            ),
            _ => TxPinConfigIncludingValidPin::new(TxEightBits::new(
                d0_tx, d1_tx, d2_tx, d3_tx, d4_tx, d5_tx, d6_tx, valid_tx,
            )),
        };
        let rx_pins = RxEightBits::new(d0_rx, d1_rx, d2_rx, d3_rx, d4_rx, d5_rx, d6_rx, valid_rx);
        let rx_pins = RxPinConfigIncludingValidPin::new(rx_pins, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Invert);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Lsb),
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(
                rx_pins,
                clock_in_pin,
                RxConfig::default()
                    .with_frequency(MAX_CLK_RATE)
                    .with_bit_order(BitPackOrder::Lsb),
            )
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

        // Data line 7 carries the valid signal. RX reads the valid level as data bit 7.
        for (rx, tx) in dma_rx_buf.as_slice().iter().zip(dma_tx_buf.as_slice()) {
            assert_eq!(*rx, tx & 0x7F | 0x80);
        }
    }

    #[cfg(parl_io_max_data_width = "16")]
    #[test]
    fn test_parl_io_rx_can_read_tx_in_16_bit_mode_with_included_valid_pin(ctx: Context) {
        const BUFFER_SIZE: usize = 128;

        let mut dma_rx_buf = dma_rx_buffer!(BUFFER_SIZE).unwrap();
        let mut dma_tx_buf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [
            (d0_rx, d0_tx),
            (d1_rx, d1_tx),
            (d2_rx, d2_tx),
            (d3_rx, d3_tx),
            (d4_rx, d4_tx),
            (d5_rx, d5_tx),
            (d6_rx, d6_tx),
            (d7_rx, d7_tx),
        ] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = cfg_select! {
            parl_io_has_tx_cs => TxPinConfigWithValidPin::new(
                TxSixteenBits::new(
                    d0_tx, d1_tx, d2_tx, d3_tx, d4_tx, d5_tx, d6_tx, d7_tx, NoPin, NoPin, NoPin,
                    NoPin, NoPin, NoPin, NoPin, NoPin,
                ),
                valid_tx,
            ),
            _ => TxPinConfigIncludingValidPin::new(TxSixteenBits::new(
                d0_tx, d1_tx, d2_tx, d3_tx, d4_tx, d5_tx, d6_tx, d7_tx, NoPin, NoPin, NoPin, NoPin,
                NoPin, NoPin, NoPin, valid_tx,
            )),
        };
        let rx_pins = RxSixteenBits::new(
            d0_rx, d1_rx, d2_rx, d3_rx, d4_rx, d5_rx, d6_rx, d7_rx, NoPin, NoPin, NoPin, NoPin,
            NoPin, NoPin, NoPin, valid_rx,
        );

        let rx_pins = RxPinConfigIncludingValidPin::new(rx_pins, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Invert);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        // RX times out with 16 lines at `MAX_CLK_RATE`.
        // With `BitPackOrder::Lsb`, RX swaps each pair of 16-bit samples.
        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(MAX_CLK_RATE / 2)
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Msb),
            )
            .unwrap();
        let pio_rx = pio
            .rx
            .with_config(
                rx_pins,
                clock_in_pin,
                RxConfig::default()
                    .with_frequency(MAX_CLK_RATE / 2)
                    .with_bit_order(BitPackOrder::Msb),
            )
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

        // Data lines 8 to 14 read low. Data line 15 carries the valid signal.
        // RX reads the valid level as data bit 15.
        let rx = dma_rx_buf.as_slice().chunks_exact(2);
        let tx = dma_tx_buf.as_slice().chunks_exact(2);
        for (rx, tx) in rx.zip(tx) {
            assert_eq!(rx[0], tx[0]);
            assert_eq!(rx[1], 0x80);
        }
    }
}

// The TX tests count clock edges with PCNT.
#[cfg(pcnt_driver_supported)]
#[embedded_test::tests(default_timeout = 3)]
mod tx {
    use defmt::info;
    #[cfg(not(parl_io_has_tx_cs))]
    use esp_hal::parl_io::TxPinConfigIncludingValidPin;
    #[cfg(any(parl_io_has_tx_cs, parl_io_max_data_width = "16"))]
    use esp_hal::parl_io::TxPinConfigWithValidPin;
    #[cfg(parl_io_max_data_width = "16")]
    use esp_hal::parl_io::TxSixteenBits;
    use esp_hal::{
        dma::DmaTxBuf,
        dma_tx_buffer,
        gpio::{
            NoPin,
            interconnect::{InputSignal, OutputSignal},
        },
        parl_io::{BitPackOrder, ClkOutPin, ParlIo, SampleEdge, TxConfig, TxEightBits},
        pcnt::{
            channel::{CtrlMode, EdgeMode},
            unit::Unit,
        },
        peripherals::PARL_IO,
        time::Rate,
    };

    use super::DmaChannel;

    struct Context {
        parl_io: PARL_IO<'static>,
        dma_channel: DmaChannel,
        clock: OutputSignal<'static>,
        valid: OutputSignal<'static>,
        clock_loopback: InputSignal<'static>,
        valid_loopback: InputSignal<'static>,
        pcnt_unit: Unit<'static>,
    }
    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (clock, _) = hil_test::common_test_pins!(peripherals);
        let valid = hil_test::unconnected_pin!(peripherals);
        let (clock_loopback, clock) = unsafe { clock.split() };
        let (valid_loopback, valid) = unsafe { valid.split() };
        let pcnt_unit = Unit::new(peripherals.PCNT0_UNIT0);
        let dma_channel = dma_channel!(peripherals);

        let parl_io = peripherals.PARL_IO;

        Context {
            parl_io,
            dma_channel,
            clock,
            valid,
            clock_loopback,
            valid_loopback,
            pcnt_unit,
        }
    }

    #[cfg(parl_io_max_data_width = "16")]
    #[test]
    fn test_parl_io_tx_16bit_valid_clock_count(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let mut dma_tx_buf: DmaTxBuf = dma_tx_buffer!(2 * BUFFER_SIZE).unwrap();

        let pins = sixteen_bit_pins_with_valid!(ctx.valid);
        let clock_pin = ClkOutPin::new(ctx.clock);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let mut pio = pio
            .tx
            .with_config(
                pins,
                clock_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_mhz(10))
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Msb),
            )
            .unwrap(); // TODO: handle error

        // use a PCNT unit to count the negative clock edges only when valid is high
        let clock_unit = ctx.pcnt_unit;
        clock_unit.channel0.set_edge_signal(ctx.clock_loopback);
        clock_unit.channel0.set_ctrl_signal(ctx.valid_loopback);
        clock_unit
            .channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);
        clock_unit
            .channel0
            .set_ctrl_mode(CtrlMode::Disable, CtrlMode::Keep);

        for _ in 0..100 {
            clock_unit.clear();
            let xfer = pio
                .write(dma_tx_buf.len(), dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (_, pio, dma_tx_buf) = xfer.wait();
            info!("clock count: {}", clock_unit.value());
            assert_eq!(clock_unit.value(), BUFFER_SIZE as _);
        }
    }

    #[test]
    fn test_parl_io_tx_8bit_valid_clock_count(ctx: Context) {
        const BUFFER_SIZE: usize = 64;
        let mut dma_tx_buf: DmaTxBuf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let pins = cfg_select! {
            all(parl_io_max_data_width = "8", not(parl_io_has_tx_cs)) => {
                TxPinConfigIncludingValidPin::new(TxEightBits::new(
                    NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, ctx.valid,
                ))
            }
            _ => TxPinConfigWithValidPin::new(
                TxEightBits::new(NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin),
                ctx.valid,
            ),
        };

        let clock_pin = ClkOutPin::new(ctx.clock);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let mut pio = pio
            .tx
            .with_config(
                pins,
                clock_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_mhz(10))
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Msb),
            )
            .unwrap(); // TODO: handle error

        // use a PCNT unit to count the negative clock edges only when valid is high
        let clock_unit = ctx.pcnt_unit;
        clock_unit.channel0.set_edge_signal(ctx.clock_loopback);
        clock_unit.channel0.set_ctrl_signal(ctx.valid_loopback);
        clock_unit
            .channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);
        clock_unit
            .channel0
            .set_ctrl_mode(CtrlMode::Disable, CtrlMode::Keep);

        for _ in 0..100 {
            clock_unit.clear();
            let xfer = pio
                .write(dma_tx_buf.len(), dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            (_, pio, dma_tx_buf) = xfer.wait();
            info!("clock count: {}", clock_unit.value());
            assert_eq!(clock_unit.value(), BUFFER_SIZE as _);
        }
    }
}
// The TX tests count clock edges with PCNT.
#[cfg(pcnt_driver_supported)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod async_tx {
    use defmt::info;
    #[cfg(not(parl_io_has_tx_cs))]
    use esp_hal::parl_io::TxPinConfigIncludingValidPin;
    #[cfg(any(parl_io_has_tx_cs, parl_io_max_data_width = "16"))]
    use esp_hal::parl_io::TxPinConfigWithValidPin;
    #[cfg(parl_io_max_data_width = "16")]
    use esp_hal::parl_io::TxSixteenBits;
    use esp_hal::{
        dma::DmaTxBuf,
        dma_tx_buffer,
        gpio::{
            NoPin,
            interconnect::{InputSignal, OutputSignal},
        },
        parl_io::{BitPackOrder, ClkOutPin, ParlIo, SampleEdge, TxConfig, TxEightBits},
        pcnt::{
            channel::{CtrlMode, EdgeMode},
            unit::Unit,
        },
        peripherals::PARL_IO,
        time::Rate,
    };

    use super::DmaChannel;

    struct Context {
        parl_io: PARL_IO<'static>,
        dma_channel: DmaChannel,
        clock: OutputSignal<'static>,
        valid: OutputSignal<'static>,
        clock_loopback: InputSignal<'static>,
        valid_loopback: InputSignal<'static>,
        pcnt_unit: Unit<'static>,
    }
    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (clock, _) = hil_test::common_test_pins!(peripherals);
        let valid = hil_test::unconnected_pin!(peripherals);
        let (clock_loopback, clock) = unsafe { clock.split() };
        let (valid_loopback, valid) = unsafe { valid.split() };
        let pcnt_unit = Unit::new(peripherals.PCNT0_UNIT0);
        let dma_channel = dma_channel!(peripherals);

        let parl_io = peripherals.PARL_IO;

        Context {
            parl_io,
            dma_channel,
            clock,
            valid,
            clock_loopback,
            valid_loopback,
            pcnt_unit,
        }
    }

    #[cfg(parl_io_max_data_width = "16")]
    #[test]
    async fn test_parl_io_tx_async_16bit_valid_clock_count(ctx: Context) {
        const BUFFER_SIZE: usize = 64;
        let mut dma_tx_buf: DmaTxBuf = dma_tx_buffer!(2 * BUFFER_SIZE).unwrap();

        let pins = sixteen_bit_pins_with_valid!(ctx.valid);
        let clock_pin = ClkOutPin::new(ctx.clock);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel)
            .unwrap()
            .into_async();

        let mut pio = pio
            .tx
            .with_config(
                pins,
                clock_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_mhz(10))
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Msb),
            )
            .unwrap();

        // use a PCNT unit to count the negitive clock edges only when valid is high
        let clock_unit = ctx.pcnt_unit;
        clock_unit.channel0.set_edge_signal(ctx.clock_loopback);
        clock_unit.channel0.set_ctrl_signal(ctx.valid_loopback);
        clock_unit
            .channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);
        clock_unit
            .channel0
            .set_ctrl_mode(CtrlMode::Disable, CtrlMode::Keep);

        for _ in 0..100 {
            clock_unit.clear();
            let mut xfer = pio
                .write(dma_tx_buf.len(), dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            xfer.wait_for_done().await;
            (_, pio, dma_tx_buf) = xfer.wait();
            info!("clock count: {}", clock_unit.value());
            assert_eq!(clock_unit.value(), BUFFER_SIZE as _);
        }
    }

    #[test]
    async fn test_parl_io_tx_async_8bit_valid_clock_count(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let mut dma_tx_buf: DmaTxBuf = dma_tx_buffer!(BUFFER_SIZE).unwrap();

        let pins = cfg_select! {
            all(parl_io_max_data_width = "8", not(parl_io_has_tx_cs)) => {
                TxPinConfigIncludingValidPin::new(TxEightBits::new(
                    NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, ctx.valid,
                ))
            }
            _ => TxPinConfigWithValidPin::new(
                TxEightBits::new(NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin),
                ctx.valid,
            ),
        };

        let clock_pin = ClkOutPin::new(ctx.clock);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel)
            .unwrap()
            .into_async();

        let mut pio = pio
            .tx
            .with_config(
                pins,
                clock_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_mhz(10))
                    .with_sample_edge(SampleEdge::Normal)
                    .with_bit_order(BitPackOrder::Msb),
            )
            .unwrap();

        // use a PCNT unit to count the negitive clock edges only when
        // valid is high
        let clock_unit = ctx.pcnt_unit;
        clock_unit.channel0.set_edge_signal(ctx.clock_loopback);
        clock_unit.channel0.set_ctrl_signal(ctx.valid_loopback);
        clock_unit
            .channel0
            .set_input_mode(EdgeMode::Increment, EdgeMode::Hold);
        clock_unit
            .channel0
            .set_ctrl_mode(CtrlMode::Disable, CtrlMode::Keep);

        for _ in 0..100 {
            clock_unit.clear();
            let mut xfer = pio
                .write(dma_tx_buf.len(), dma_tx_buf)
                .map_err(|e| e.0)
                .unwrap();
            xfer.wait_for_done().await;
            (_, pio, dma_tx_buf) = xfer.wait();
            info!("clock count: {}", clock_unit.value());
            assert_eq!(clock_unit.value(), BUFFER_SIZE as _);
        }
    }
}
