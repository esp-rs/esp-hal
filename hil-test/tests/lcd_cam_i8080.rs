//! lcd_cam i8080 tests

//% CHIPS: esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    dma::{DmaChannel0, DmaTxBuf},
    dma_buffers,
    gpio::{GpioPin, NoPin},
    lcd_cam::{
        lcd::i8080::{Command, Config, TxEightBits, TxSixteenBits, I8080},
        BitOrder,
        LcdCam,
    },
    pcnt::{
        channel::{CtrlMode, EdgeMode},
        Pcnt,
    },
    time::RateExtU32,
    Blocking,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

#[allow(non_snake_case)]
struct Pins {
    pub GPIO8: GpioPin<8>,
    pub GPIO11: GpioPin<11>,
    pub GPIO12: GpioPin<12>,
    pub GPIO16: GpioPin<16>,
    pub GPIO17: GpioPin<17>,
}

struct Context<'d> {
    lcd_cam: LcdCam<'d, Blocking>,
    pcnt: Pcnt<'d>,
    pins: Pins,
    dma: DmaChannel0,
    dma_buf: DmaTxBuf,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
        let pcnt = Pcnt::new(peripherals.PCNT);

        let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, DATA_SIZE);
        let dma_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        Context {
            lcd_cam,
            dma: peripherals.DMA_CH0,
            pcnt,
            pins: Pins {
                GPIO8: peripherals.GPIO8,
                GPIO11: peripherals.GPIO11,
                GPIO12: peripherals.GPIO12,
                GPIO16: peripherals.GPIO16,
                GPIO17: peripherals.GPIO17,
            },
            dma_buf,
        }
    }

    #[test]
    fn test_i8080_8bit(ctx: Context<'static>) {
        let pins = TxEightBits::new(NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin);

        let i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            ctx.dma,
            pins,
            Config::default().with_frequency(20.MHz()),
        )
        .unwrap();

        let xfer = i8080.send(Command::<u8>::None, 0, ctx.dma_buf).unwrap();
        xfer.wait().0.unwrap();
    }

    #[test]
    fn test_i8080_8bit_is_seen_by_pcnt(ctx: Context<'static>) {
        // FIXME: Update this test to exercise all the I8080 output signals once the
        // issue with configuring pins as outputs after inputs have been sorted
        // out. See https://github.com/esp-rs/esp-hal/pull/2173#issue-2529323702

        let (unit_ctrl, cs_signal) = ctx.pins.GPIO8.split();
        let (unit0_input, unit0_signal) = ctx.pins.GPIO11.split();
        let (unit1_input, unit1_signal) = ctx.pins.GPIO12.split();
        let (unit2_input, unit2_signal) = ctx.pins.GPIO16.split();
        let (unit3_input, unit3_signal) = ctx.pins.GPIO17.split();

        let pcnt = ctx.pcnt;

        pcnt.unit0
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
        pcnt.unit1
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
        pcnt.unit2
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
        pcnt.unit3
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);

        pcnt.unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        pcnt.unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        pcnt.unit2
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        pcnt.unit3
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let pins = TxEightBits::new(
            unit0_signal,
            unit1_signal,
            unit2_signal,
            unit3_signal,
            NoPin,
            NoPin,
            NoPin,
            NoPin,
        );

        let mut i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            ctx.dma,
            pins,
            Config::default().with_frequency(20.MHz()),
        )
        .unwrap()
        .with_cs(cs_signal)
        .with_ctrl_pins(NoPin, NoPin);

        // explicitly drop the camera half to see if it disables clocks (unexpectedly,
        // I8080 should keep it alive)
        core::mem::drop(ctx.lcd_cam.cam);

        // This is to make the test values look more intuitive.
        i8080.set_bit_order(BitOrder::Inverted);

        pcnt.unit0.channel0.set_edge_signal(unit0_input);
        pcnt.unit1.channel0.set_edge_signal(unit1_input);
        pcnt.unit2.channel0.set_edge_signal(unit2_input);
        pcnt.unit3.channel0.set_edge_signal(unit3_input);

        pcnt.unit0.channel0.set_ctrl_signal(unit_ctrl.clone());
        pcnt.unit1.channel0.set_ctrl_signal(unit_ctrl.clone());
        pcnt.unit2.channel0.set_ctrl_signal(unit_ctrl.clone());
        pcnt.unit3.channel0.set_ctrl_signal(unit_ctrl.clone());

        pcnt.unit0.resume();
        pcnt.unit1.resume();
        pcnt.unit2.resume();
        pcnt.unit3.resume();

        let data_to_send = [
            0b0000_0000,
            0b1010_0000,
            0b0110_0000,
            0b1110_0000,
            0b0000_0000,
            0b1000_0000,
            0b0100_0000,
            0b1010_0000,
            0b0101_0000,
            0b1000_0000,
        ];

        let mut dma_buf = ctx.dma_buf;
        dma_buf.as_mut_slice().fill(0);
        dma_buf.as_mut_slice()[..data_to_send.len()].copy_from_slice(&data_to_send);

        let xfer = i8080.send(Command::<u8>::None, 0, dma_buf).unwrap();
        xfer.wait().0.unwrap();

        let actual = [
            pcnt.unit0.value(),
            pcnt.unit1.value(),
            pcnt.unit2.value(),
            pcnt.unit3.value(),
        ];
        let expected = [5, 3, 2, 1];

        assert_eq!(expected, actual);
    }

    #[test]
    fn test_i8080_16bit_is_seen_by_pcnt(ctx: Context<'static>) {
        // FIXME: Update this test to exercise all the I8080 output signals once the
        // issue with configuring pins as outputs after inputs have been sorted
        // out. See https://github.com/esp-rs/esp-hal/pull/2173#issue-2529323702

        let (unit_ctrl, cs_signal) = ctx.pins.GPIO8.split();
        let (unit0_input, unit0_signal) = ctx.pins.GPIO11.split();
        let (unit1_input, unit1_signal) = ctx.pins.GPIO12.split();
        let (unit2_input, unit2_signal) = ctx.pins.GPIO16.split();
        let (unit3_input, unit3_signal) = ctx.pins.GPIO17.split();

        let pcnt = ctx.pcnt;

        pcnt.unit0
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
        pcnt.unit1
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
        pcnt.unit2
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);
        pcnt.unit3
            .channel0
            .set_ctrl_mode(CtrlMode::Keep, CtrlMode::Disable);

        pcnt.unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        pcnt.unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        pcnt.unit2
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);
        pcnt.unit3
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let pins = TxSixteenBits::new(
            NoPin,
            NoPin,
            NoPin,
            unit0_signal,
            NoPin,
            NoPin,
            NoPin,
            unit1_signal,
            NoPin,
            NoPin,
            NoPin,
            unit2_signal,
            NoPin,
            NoPin,
            NoPin,
            unit3_signal,
        );

        let mut i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            ctx.dma,
            pins,
            Config::default().with_frequency(20.MHz()),
        )
        .unwrap()
        .with_cs(cs_signal)
        .with_ctrl_pins(NoPin, NoPin);

        // This is to make the test values look more intuitive.
        i8080.set_bit_order(BitOrder::Inverted);

        pcnt.unit0.channel0.set_edge_signal(unit0_input);
        pcnt.unit1.channel0.set_edge_signal(unit1_input);
        pcnt.unit2.channel0.set_edge_signal(unit2_input);
        pcnt.unit3.channel0.set_edge_signal(unit3_input);

        pcnt.unit0.channel0.set_ctrl_signal(unit_ctrl.clone());
        pcnt.unit1.channel0.set_ctrl_signal(unit_ctrl.clone());
        pcnt.unit2.channel0.set_ctrl_signal(unit_ctrl.clone());
        pcnt.unit3.channel0.set_ctrl_signal(unit_ctrl.clone());

        pcnt.unit0.resume();
        pcnt.unit1.resume();
        pcnt.unit2.resume();
        pcnt.unit3.resume();

        let data_to_send = [
            0b0000_0000_0000_0000u16,
            0b0001_0000_0001_0000,
            0b0000_0001_0001_0000,
            0b0001_0001_0001_0000,
            0b0000_0000_0000_0000,
            0b0001_0000_0000_0000,
            0b0000_0001_0000_0000,
            0b0001_0000_0001_0000,
            0b0000_0001_0000_0001,
            0b0001_0000_0000_0000,
        ];

        let mut dma_buf = ctx.dma_buf;

        // FIXME: Replace this 16 -> 8 bit copy once DmaTxBuf takes a generic parameter.
        // i.e. DmaTxBuf<u16>

        // Copy 16 bit array into 8 bit buffer.
        dma_buf
            .as_mut_slice()
            .iter_mut()
            .zip(data_to_send.iter().flat_map(|&d| d.to_ne_bytes()))
            .for_each(|(d, s)| *d = s);

        let xfer = i8080.send(Command::<u16>::None, 0, dma_buf).unwrap();
        xfer.wait().0.unwrap();

        let actual = [
            pcnt.unit0.value(),
            pcnt.unit1.value(),
            pcnt.unit2.value(),
            pcnt.unit3.value(),
        ];
        let expected = [5, 3, 2, 1];

        assert_eq!(expected, actual);
    }
}
