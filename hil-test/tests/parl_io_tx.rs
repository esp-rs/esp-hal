//! PARL_IO TX test

//% CHIPS: esp32c6 esp32h2
//% FEATURES: unstable

#![no_std]
#![no_main]

#[cfg(esp32c6)]
use esp_hal::parl_io::{TxPinConfigWithValidPin, TxSixteenBits};
use esp_hal::{
    dma::DmaTxBuf,
    dma_tx_buffer,
    gpio::{
        NoPin,
        interconnect::{InputSignal, OutputSignal},
    },
    parl_io::{
        BitPackOrder,
        ClkOutPin,
        ParlIo,
        SampleEdge,
        TxConfig,
        TxEightBits,
        TxPinConfigIncludingValidPin,
    },
    pcnt::{
        Pcnt,
        channel::{CtrlMode, EdgeMode},
        unit::Unit,
    },
    peripherals::{DMA_CH0, PARL_IO},
    time::Rate,
};
use hil_test as _;

struct Context {
    parl_io: PARL_IO<'static>,
    dma_channel: DMA_CH0<'static>,
    clock: OutputSignal<'static>,
    valid: OutputSignal<'static>,
    clock_loopback: InputSignal<'static>,
    valid_loopback: InputSignal<'static>,
    pcnt_unit: Unit<'static, 0>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use defmt::info;

    use super::*;

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
                    .with_sample_edge(SampleEdge::Invert)
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
            #[cfg(esp32c6)]
            NoPin,
        );

        #[cfg(esp32h2)]
        let pins = TxPinConfigIncludingValidPin::new(pins);
        #[cfg(esp32c6)]
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
                    .with_sample_edge(SampleEdge::Invert)
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
