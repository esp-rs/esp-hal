//! lcd_cam i8080 tests

//% CHIPS: esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaDescriptor, DmaPriority},
    dma_buffers,
    gpio::{Io, NoPin},
    lcd_cam::{
        lcd::i8080::{Command, Config, TxEightBits, TxSixteenBits, I8080},
        BitOrder,
        LcdCam,
    },
    pcnt::{
        channel::{CtrlMode, EdgeMode},
        Pcnt,
    },
    prelude::*,
};
use hil_test as _;
use static_cell::ConstStaticCell;

const DATA_SIZE: usize = 1024 * 10;

struct Context<'d> {
    lcd_cam: LcdCam<'d, esp_hal::Blocking>,
    pcnt: Pcnt<'d>,
    io: Io,
    dma: Dma<'d>,
    tx_buffer: &'static mut [u8],
    tx_descriptors: &'static mut [DmaDescriptor],
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        let dma = Dma::new(peripherals.DMA);
        let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
        let pcnt = Pcnt::new(peripherals.PCNT);
        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, DATA_SIZE);

        Context {
            lcd_cam,
            dma,
            pcnt,
            io,
            tx_buffer,
            tx_descriptors,
        }
    }

    #[test]
    fn test_i8080_8bit(ctx: Context<'static>) {
        let channel = ctx.dma.channel0.configure(false, DmaPriority::Priority0);

        let pins = TxEightBits::new(NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin);

        let mut i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            channel.tx,
            ctx.tx_descriptors,
            pins,
            20.MHz(),
            Config::default(),
        );

        let xfer = i8080
            .send_dma(Command::<u8>::None, 0, &ctx.tx_buffer)
            .unwrap();
        xfer.wait().unwrap();
    }

    #[test]
    fn test_i8080_8bit_async_channel(ctx: Context<'static>) {
        let channel = ctx
            .dma
            .channel0
            .configure_for_async(false, DmaPriority::Priority0);
        let pins = TxEightBits::new(NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin, NoPin);

        let mut i8080 = I8080::new(
            ctx.lcd_cam.lcd,
            channel.tx,
            ctx.tx_descriptors,
            pins,
            20.MHz(),
            Config::default(),
        );

        let xfer = i8080
            .send_dma(Command::<u8>::None, 0, &ctx.tx_buffer)
            .unwrap();
        xfer.wait().unwrap();
    }

    #[test]
    fn test_i8080_8bit_is_seen_by_pcnt(ctx: Context<'static>) {
        // FIXME: Update this test to exercise all the I8080 output signals once the issue with
        // configuring pins as outputs after inputs have been sorted out.
        // See https://github.com/esp-rs/esp-hal/pull/2173#issue-2529323702

        let cs_signal = ctx.io.pins.gpio8;
        let unit0_signal = ctx.io.pins.gpio11;
        let unit1_signal = ctx.io.pins.gpio12;
        let unit2_signal = ctx.io.pins.gpio16;
        let unit3_signal = ctx.io.pins.gpio17;

        let pcnt = ctx.pcnt;

        let unit_ctrl = cs_signal.peripheral_input();
        let unit0_input = unit0_signal.peripheral_input();
        let unit1_input = unit1_signal.peripheral_input();
        let unit2_input = unit2_signal.peripheral_input();
        let unit3_input = unit3_signal.peripheral_input();

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

        let channel = ctx.dma.channel0.configure(false, DmaPriority::Priority0);
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
            channel.tx,
            ctx.tx_descriptors,
            pins,
            20.MHz(),
            Config::default(),
        )
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

        let tx_buffer = ctx.tx_buffer;
        tx_buffer.fill(0);
        tx_buffer[..data_to_send.len()].copy_from_slice(&data_to_send);

        let xfer = i8080.send_dma(Command::<u8>::None, 0, &tx_buffer).unwrap();
        xfer.wait().unwrap();

        let actual = [
            pcnt.unit0.get_value(),
            pcnt.unit1.get_value(),
            pcnt.unit2.get_value(),
            pcnt.unit3.get_value(),
        ];
        let expected = [5, 3, 2, 1];

        assert_eq!(expected, actual);
    }

    #[test]
    fn test_i8080_16bit_is_seen_by_pcnt(ctx: Context<'static>) {
        // FIXME: Update this test to exercise all the I8080 output signals once the issue with
        // configuring pins as outputs after inputs have been sorted out.
        // See https://github.com/esp-rs/esp-hal/pull/2173#issue-2529323702

        let cs_signal = ctx.io.pins.gpio8;
        let unit0_signal = ctx.io.pins.gpio11;
        let unit1_signal = ctx.io.pins.gpio12;
        let unit2_signal = ctx.io.pins.gpio16;
        let unit3_signal = ctx.io.pins.gpio17;

        let pcnt = ctx.pcnt;

        let unit_ctrl = cs_signal.peripheral_input();
        let unit0_input = unit0_signal.peripheral_input();
        let unit1_input = unit1_signal.peripheral_input();
        let unit2_input = unit2_signal.peripheral_input();
        let unit3_input = unit3_signal.peripheral_input();

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

        let channel = ctx.dma.channel0.configure(false, DmaPriority::Priority0);
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
            channel.tx,
            ctx.tx_descriptors,
            pins,
            20.MHz(),
            Config::default(),
        )
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
            0b0000_0000_0000_0000,
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

        static TX_BUF: ConstStaticCell<[u16; 10]> = ConstStaticCell::new([0; 10]);
        let tx_buffer = TX_BUF.take();
        tx_buffer.copy_from_slice(&data_to_send);

        let xfer = i8080.send_dma(Command::<u16>::None, 0, &tx_buffer).unwrap();
        xfer.wait().unwrap();

        let actual = [
            pcnt.unit0.get_value(),
            pcnt.unit1.get_value(),
            pcnt.unit2.get_value(),
            pcnt.unit3.get_value(),
        ];
        let expected = [5, 3, 2, 1];

        assert_eq!(expected, actual);
    }
}
