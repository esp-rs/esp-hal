//! PARL_IO TX async test

//% CHIPS: esp32c6 esp32h2
//% FEATURES: unstable

#![no_std]
#![no_main]

#[cfg(esp32c6)]
use esp_hal::parl_io::{TxPinConfigWithValidPin, TxSixteenBits};
use esp_hal::{
    dma::{DmaChannel0, DmaTxBuf},
    dma_tx_buffer,
    gpio::{
        interconnect::{InputSignal, OutputSignal},
        NoPin,
    },
    parl_io::{
        BitPackOrder,
        ClkOutPin,
        ParlIoTxOnly,
        SampleEdge,
        TxEightBits,
        TxPinConfigIncludingValidPin,
    },
    pcnt::{
        channel::{CtrlMode, EdgeMode},
        unit::Unit,
        Pcnt,
    },
    peripherals::PARL_IO,
    time::RateExtU32,
};
use hil_test as _;

struct Context {
    parl_io: PARL_IO,
    dma_channel: DmaChannel0,
    clock: OutputSignal,
    valid: OutputSignal,
    clock_loopback: InputSignal,
    valid_loopback: InputSignal,
    pcnt_unit: Unit<'static, 0>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3, executor = hil_test::Executor::new())]
mod tests {
    use defmt::info;

    use super::*;

    #[init]
    async fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (clock, _) = hil_test::common_test_pins!(peripherals);
        let valid = hil_test::unconnected_pin!(peripherals);
        let (clock_loopback, clock) = clock.split();
        let (valid_loopback, valid) = valid.split();
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
        let mut pins = TxPinConfigIncludingValidPin::new(pins);
        let mut clock_pin = ClkOutPin::new(ctx.clock);

        let pio = ParlIoTxOnly::new(ctx.parl_io, ctx.dma_channel, 10.MHz())
            .unwrap()
            .into_async();

        let mut pio = pio
            .tx
            .with_config(
                &mut pins,
                &mut clock_pin,
                0,
                SampleEdge::Invert,
                BitPackOrder::Msb,
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
            let mut xfer = pio.write(dma_tx_buf.len(), dma_tx_buf).unwrap();
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
            #[cfg(esp32c6)]
            NoPin,
        );

        #[cfg(esp32h2)]
        let mut pins = TxPinConfigIncludingValidPin::new(pins);
        #[cfg(esp32c6)]
        let mut pins = TxPinConfigWithValidPin::new(pins, ctx.valid);

        let mut clock_pin = ClkOutPin::new(ctx.clock);

        let pio = ParlIoTxOnly::new(ctx.parl_io, ctx.dma_channel, 10.MHz())
            .unwrap()
            .into_async();

        let mut pio = pio
            .tx
            .with_config(
                &mut pins,
                &mut clock_pin,
                0,
                SampleEdge::Invert,
                BitPackOrder::Msb,
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
            let mut xfer = pio.write(dma_tx_buf.len(), dma_tx_buf).unwrap();
            xfer.wait_for_done().await;
            (_, pio, dma_tx_buf) = xfer.wait();
            info!("clock count: {}", clock_unit.value());
            assert_eq!(clock_unit.value(), BUFFER_SIZE as _);
        }
    }
}
