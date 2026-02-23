//! PARL_IO test

//% CHIPS: esp32c5 esp32c6 esp32h2
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        dma::{DmaRxBuf, DmaTxBuf},
        dma_buffers,
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
        peripherals::{DMA_CH0, PARL_IO},
        time::Rate,
    };

    struct Context {
        parl_io: PARL_IO<'static>,
        dma_channel: DMA_CH0<'static>,
        clock_pin: AnyPin<'static>,
        valid_pin: AnyPin<'static>,
        data_pins: [AnyPin<'static>; 8],
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let dma_channel = peripherals.DMA_CH0;

        let parl_io = peripherals.PARL_IO;

        cfg_if::cfg_if! {
            if #[cfg(esp32c5)] {
                // 27 is RGB, 9 and 10 are connected, 13 and 14 is USB
                let valid_pin = peripherals.GPIO0.degrade();
                let clock_pin = peripherals.GPIO1.degrade();
                let data_pins = [
                    peripherals.GPIO2.degrade(),
                    peripherals.GPIO3.degrade(),
                    peripherals.GPIO4.degrade(),
                    peripherals.GPIO5.degrade(),
                    peripherals.GPIO6.degrade(),
                    peripherals.GPIO7.degrade(),
                    peripherals.GPIO8.degrade(),
                    peripherals.GPIO9.degrade(),
                ];
            } else if #[cfg(esp32c6)] {
                // 8 is RGB, 2 and 3 are connected, 12 and 13 is USB
                let valid_pin = peripherals.GPIO0.degrade();
                let clock_pin = peripherals.GPIO1.degrade();
                let data_pins = [
                    peripherals.GPIO2.degrade(),
                    peripherals.GPIO4.degrade(),
                    peripherals.GPIO5.degrade(),
                    peripherals.GPIO6.degrade(),
                    peripherals.GPIO7.degrade(),
                    peripherals.GPIO8.degrade(),
                    peripherals.GPIO9.degrade(),
                    peripherals.GPIO10.degrade(),
                ];
            } else if #[cfg(esp32h2)] {
                // 8 is RGB, 2 and 3 are connected, 26 and 27 is USB
                let valid_pin = peripherals.GPIO0.degrade();
                let clock_pin = peripherals.GPIO1.degrade();
                let data_pins = [
                    peripherals.GPIO2.degrade(),
                    peripherals.GPIO4.degrade(),
                    peripherals.GPIO5.degrade(),
                    peripherals.GPIO8.degrade(),
                    peripherals.GPIO9.degrade(),
                    peripherals.GPIO10.degrade(),
                    peripherals.GPIO11.degrade(),
                    peripherals.GPIO23.degrade(),
                ];
            }
        }

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

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [(d0_rx, d0_tx), ..] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = TxOneBit::new(d0_tx);
        let rx_pins = RxOneBit::new(d0_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_mhz(40))
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
                    .with_frequency(Rate::from_mhz(40))
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

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let (clock_rx, clock_tx) = unsafe { ctx.clock_pin.split() };
        let (valid_rx, valid_tx) = unsafe { ctx.valid_pin.split() };
        let [(d0_rx, d0_tx), (d1_rx, d1_tx), ..] = ctx.data_pins.map(|pin| unsafe { pin.split() });

        let tx_pins = TxTwoBits::new(d0_tx, d1_tx);
        let rx_pins = RxTwoBits::new(d0_rx, d1_rx);

        let tx_pins = TxPinConfigWithValidPin::new(tx_pins, valid_tx);
        let rx_pins = RxPinConfigWithValidPin::new(rx_pins, valid_rx, EnableMode::HighLevel);

        let clock_out_pin = ClkOutPin::new(clock_tx);
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_mhz(40))
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
                    .with_frequency(Rate::from_mhz(40))
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

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

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
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

        let pio = ParlIo::new(ctx.parl_io, ctx.dma_channel).unwrap();

        let pio_tx = pio
            .tx
            .with_config(
                tx_pins,
                clock_out_pin,
                TxConfig::default()
                    .with_frequency(Rate::from_mhz(40))
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
                    .with_frequency(Rate::from_mhz(40))
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

    // This test is disabled on the H2 because in 8-bit mode, there's no external
    // enable pin to indicate when data is valid. The C6 has the same issue in
    // 16-bit mode. The test would have to be written differently for it to work
    // reliably. See https://github.com/esp-rs/esp-hal/pull/3339.
    #[cfg(esp32c6)]
    #[test]
    fn test_parl_io_rx_can_read_tx_in_8_bit_mode(ctx: Context) {
        const BUFFER_SIZE: usize = 250;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

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
        let clock_in_pin = RxClkInPin::new(clock_rx, SampleEdge::Normal);

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
}

#[embedded_test::tests(default_timeout = 3)]
mod tx {
    use defmt::info;
    #[cfg(any(esp32c6, esp32h2))]
    use esp_hal::parl_io::TxPinConfigIncludingValidPin;
    #[cfg(any(esp32c5, esp32c6))]
    use esp_hal::parl_io::TxPinConfigWithValidPin;
    #[cfg(esp32c6)]
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
            Pcnt,
            channel::{CtrlMode, EdgeMode},
            unit::Unit,
        },
        peripherals::{DMA_CH0, PARL_IO},
        time::Rate,
    };

    struct Context {
        parl_io: PARL_IO<'static>,
        dma_channel: DMA_CH0<'static>,
        clock: OutputSignal<'static>,
        valid: OutputSignal<'static>,
        clock_loopback: InputSignal<'static>,
        valid_loopback: InputSignal<'static>,
        pcnt_unit: Unit<'static, 0>,
    }
    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (clock, _) = hil_test::common_test_pins!(peripherals);
        let valid = hil_test::unconnected_pin!(peripherals);
        let (clock_loopback, clock) = unsafe { clock.split() };
        let (valid_loopback, valid) = unsafe { valid.split() };
        let pcnt = Pcnt::new(peripherals.PCNT);
        let pcnt_unit = pcnt.unit0;
        let dma_channel = peripherals.DMA_CH0;

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

    #[cfg(esp32c6)]
    #[test]
    fn test_parl_io_tx_16bit_valid_clock_count(ctx: Context) {
        const BUFFER_SIZE: usize = 64;

        let mut dma_tx_buf: DmaTxBuf = dma_tx_buffer!(2 * BUFFER_SIZE).unwrap();

        let pins = TxSixteenBits::new(
            NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin,
            NoPin, NoPin, NoPin, ctx.valid,
        );
        let pins = TxPinConfigIncludingValidPin::new(pins);
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

        let pins = TxEightBits::new(
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            #[cfg(esp32h2)]
            ctx.valid,
            #[cfg(any(esp32c5, esp32c6))]
            NoPin,
        );

        #[cfg(esp32h2)]
        let pins = TxPinConfigIncludingValidPin::new(pins);
        #[cfg(any(esp32c5, esp32c6))]
        let pins = TxPinConfigWithValidPin::new(pins, ctx.valid);

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
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod async_tx {
    use defmt::info;
    #[cfg(any(esp32c6, esp32h2))]
    use esp_hal::parl_io::TxPinConfigIncludingValidPin;
    #[cfg(any(esp32c5, esp32c6))]
    use esp_hal::parl_io::TxPinConfigWithValidPin;
    #[cfg(esp32c6)]
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
            Pcnt,
            channel::{CtrlMode, EdgeMode},
            unit::Unit,
        },
        peripherals::{DMA_CH0, PARL_IO},
        time::Rate,
    };

    struct Context {
        parl_io: PARL_IO<'static>,
        dma_channel: DMA_CH0<'static>,
        clock: OutputSignal<'static>,
        valid: OutputSignal<'static>,
        clock_loopback: InputSignal<'static>,
        valid_loopback: InputSignal<'static>,
        pcnt_unit: Unit<'static, 0>,
    }
    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (clock, _) = hil_test::common_test_pins!(peripherals);
        let valid = hil_test::unconnected_pin!(peripherals);
        let (clock_loopback, clock) = unsafe { clock.split() };
        let (valid_loopback, valid) = unsafe { valid.split() };
        let pcnt = Pcnt::new(peripherals.PCNT);
        let pcnt_unit = pcnt.unit0;
        let dma_channel = peripherals.DMA_CH0;

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

    #[cfg(esp32c6)]
    #[test]
    async fn test_parl_io_tx_async_16bit_valid_clock_count(ctx: Context) {
        const BUFFER_SIZE: usize = 64;
        let mut dma_tx_buf: DmaTxBuf = dma_tx_buffer!(2 * BUFFER_SIZE).unwrap();

        let pins = TxSixteenBits::new(
            NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin,
            NoPin, NoPin, NoPin, ctx.valid,
        );
        let pins = TxPinConfigIncludingValidPin::new(pins);
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

        let pins = TxEightBits::new(
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            NoPin,
            #[cfg(esp32h2)]
            ctx.valid,
            #[cfg(any(esp32c5, esp32c6))]
            NoPin,
        );

        #[cfg(esp32h2)]
        let pins = TxPinConfigIncludingValidPin::new(pins);
        #[cfg(any(esp32c5, esp32c6))]
        let pins = TxPinConfigWithValidPin::new(pins, ctx.valid);

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
