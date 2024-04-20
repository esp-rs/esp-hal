use embedded_dma::WriteBuffer;
use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{
        ChannelRx,
        ChannelTypes,
        DmaError,
        DmaPeripheral,
        DmaTransfer,
        LcdCamPeripheral,
        RegisterAccess,
        Rx,
        RxChannel,
        RxPrivate,
    },
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    i2s::Error,
    lcd_cam::{cam::private::RxPins, private::calculate_clkm, BitOrder},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
};

/// Generation of GDMA SUC EOF
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EofMode {
    /// Generate GDMA SUC EOF by data byte length
    ByteLen,
    /// Generate GDMA SUC EOF by the vsync signal
    VsyncSignal,
}

pub struct Cam<'d> {
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,
}

pub struct Camera<'d, RX, P> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    rx_channel: RX,
    _pins: P,
}

impl<'d, T, R, P: RxPins> Camera<'d, ChannelRx<'d, T, R>, P>
where
    T: RxChannel<R>,
    R: ChannelTypes + RegisterAccess,
    R::P: LcdCamPeripheral,
{
    pub fn new(
        cam: Cam<'d>,
        mut channel: ChannelRx<'d, T, R>,
        mut pins: P,
        frequency: HertzU32,
        clocks: &Clocks,
    ) -> Self {
        let lcd_cam = cam.lcd_cam;

        let (i, divider) = calculate_clkm(
            frequency.to_Hz() as _,
            &[
                clocks.xtal_clock.to_Hz() as _,
                clocks.cpu_clock.to_Hz() as _,
                clocks.crypto_pwm_clock.to_Hz() as _,
            ],
        );

        lcd_cam.cam_ctrl().write(|w| {
            // Force enable the clock for all configuration registers.
            unsafe {
                w.cam_clk_sel()
                    .bits((i + 1) as _)
                    .cam_clkm_div_num()
                    .bits(divider.div_num as _)
                    .cam_clkm_div_b()
                    .bits(divider.div_b as _)
                    .cam_clkm_div_a()
                    .bits(divider.div_a as _)
                    .cam_vsync_filter_thres()
                    .bits(0)
                    .cam_vs_eof_en()
                    .set_bit()
            }
        });
        lcd_cam
            .cam_ctrl1()
            .write(|w| w.cam_vh_de_mode_en().clear_bit());

        lcd_cam
            .cam_rgb_yuv()
            .write(|w| w.cam_conv_bypass().clear_bit());

        lcd_cam.cam_ctrl().modify(|_, w| w.cam_update().set_bit());

        channel.init_channel();
        pins.configure();

        Self {
            lcd_cam,
            rx_channel: channel,
            _pins: pins,
        }
    }
}

impl<'d, RX: Rx, P: RxPins> Camera<'d, RX, P> {
    pub fn set_bit_order(&mut self, bit_order: BitOrder) -> &mut Self {
        self.lcd_cam
            .cam_ctrl()
            .modify(|_, w| w.cam_bit_order().bit(bit_order != BitOrder::default()));
        self
    }

    pub fn with_master_clock<MCLK: OutputPin>(self, mclk: impl Peripheral<P = MCLK> + 'd) -> Self {
        crate::into_ref!(mclk);
        mclk.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::CAM_CLK);
        self
    }

    pub fn with_ctrl_pins<VSYNC: InputPin, HENABLE: InputPin, PCLK: InputPin>(
        self,
        vsync: impl Peripheral<P = VSYNC> + 'd,
        h_enable: impl Peripheral<P = HENABLE> + 'd,
        pclk: impl Peripheral<P = PCLK> + 'd,
    ) -> Self {
        crate::into_ref!(vsync);
        crate::into_ref!(h_enable);
        crate::into_ref!(pclk);

        vsync
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_V_SYNC);
        h_enable
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_H_ENABLE);
        pclk.set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_PCLK);

        self
    }

    // Reset Camera control unit and Async Rx FIFO
    fn reset_unit_and_fifo(&self) {
        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_reset().set_bit());
        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_reset().clear_bit());
        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_afifo_reset().set_bit());
        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_afifo_reset().clear_bit());
    }

    // Start the Camera unit to listen for incoming DVP stream.
    fn start_unit(&self) {
        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().set_bit());
    }

    pub fn read_dma<'t, RXBUF>(
        &'t mut self,
        buf: &'t mut RXBUF,
    ) -> Result<Transfer<'t, 'd, RX, P>, DmaError>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { buf.write_buffer() };

        self.reset_unit_and_fifo();

        // Start DMA to receive incoming transfer.
        self.rx_channel
            .prepare_transfer_without_start(false, DmaPeripheral::LcdCam, ptr, len)?;
        self.rx_channel.start_transfer()?;

        self.start_unit();

        Ok(Transfer { instance: self })
    }

    pub fn read_dma_circular<'t, RXBUF>(
        &'t mut self,
        buf: &'t mut RXBUF,
    ) -> Result<Transfer<'t, 'd, RX, P>, DmaError>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { buf.write_buffer() };

        self.reset_unit_and_fifo();

        // Start DMA to receive incoming transfer.
        self.rx_channel
            .prepare_transfer_without_start(true, DmaPeripheral::LcdCam, ptr, len)?;
        self.rx_channel.start_transfer()?;

        self.start_unit();

        Ok(Transfer { instance: self })
    }
}

/// An in-progress transfer
#[must_use]
pub struct Transfer<'t, 'd, RX: Rx, P> {
    instance: &'t mut Camera<'d, RX, P>,
}

impl<'t, 'd, RX: Rx, P> Transfer<'t, 'd, RX, P> {
    /// Amount of bytes which can be popped
    pub fn available(&mut self) -> usize {
        self.instance.rx_channel.available()
    }

    pub fn pop(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        Ok(self.instance.rx_channel.pop(data)?)
    }

    /// Wait for the DMA transfer to complete.
    /// Length of the received data is returned
    #[allow(clippy::type_complexity)]
    pub fn wait_receive(self, dst: &mut [u8]) -> Result<usize, (DmaError, usize)> {
        // Wait for DMA transfer to finish.
        while !self.is_done() {}

        let len = self
            .instance
            .rx_channel
            .drain_buffer(dst)
            .map_err(|e| (e, 0))?;

        if self.instance.rx_channel.has_error() {
            Err((DmaError::DescriptorError, len))
        } else {
            Ok(len)
        }
    }
}

impl<'t, 'd, RX: Rx, P> DmaTransfer for Transfer<'t, 'd, RX, P> {
    fn wait(self) -> Result<(), DmaError> {
        // Wait for DMA transfer to finish.
        while !self.is_done() {}

        let ch = &self.instance.rx_channel;
        if ch.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    fn is_done(&self) -> bool {
        let ch = &self.instance.rx_channel;
        // Wait for IN_SUC_EOF (i.e. VSYNC)
        ch.is_done() ||
        // Or for IN_DSCR_EMPTY (i.e. No more buffer space)
        ch.has_dscr_empty_error() ||
        // Or for IN_DSCR_ERR (i.e. bad descriptor)
        ch.has_error()
    }
}

impl<'t, 'd, RX: Rx, P> Drop for Transfer<'t, 'd, RX, P> {
    fn drop(&mut self) {
        self.instance
            .lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().clear_bit());
        // TODO: Stop DMA?? self.instance.rx_channel.stop_transfer();
    }
}

pub struct RxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7> {
    pin_0: PeripheralRef<'d, P0>,
    pin_1: PeripheralRef<'d, P1>,
    pin_2: PeripheralRef<'d, P2>,
    pin_3: PeripheralRef<'d, P3>,
    pin_4: PeripheralRef<'d, P4>,
    pin_5: PeripheralRef<'d, P5>,
    pin_6: PeripheralRef<'d, P6>,
    pin_7: PeripheralRef<'d, P7>,
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> RxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: InputPin,
    P1: InputPin,
    P2: InputPin,
    P3: InputPin,
    P4: InputPin,
    P5: InputPin,
    P6: InputPin,
    P7: InputPin,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pin_0: impl Peripheral<P = P0> + 'd,
        pin_1: impl Peripheral<P = P1> + 'd,
        pin_2: impl Peripheral<P = P2> + 'd,
        pin_3: impl Peripheral<P = P3> + 'd,
        pin_4: impl Peripheral<P = P4> + 'd,
        pin_5: impl Peripheral<P = P5> + 'd,
        pin_6: impl Peripheral<P = P6> + 'd,
        pin_7: impl Peripheral<P = P7> + 'd,
    ) -> Self {
        crate::into_ref!(pin_0);
        crate::into_ref!(pin_1);
        crate::into_ref!(pin_2);
        crate::into_ref!(pin_3);
        crate::into_ref!(pin_4);
        crate::into_ref!(pin_5);
        crate::into_ref!(pin_6);
        crate::into_ref!(pin_7);

        Self {
            pin_0,
            pin_1,
            pin_2,
            pin_3,
            pin_4,
            pin_5,
            pin_6,
            pin_7,
        }
    }
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> RxPins for RxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: InputPin,
    P1: InputPin,
    P2: InputPin,
    P3: InputPin,
    P4: InputPin,
    P5: InputPin,
    P6: InputPin,
    P7: InputPin,
{
    type Word = u8;

    fn configure(&mut self) {
        self.pin_0
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_0);
        self.pin_1
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_1);
        self.pin_2
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_2);
        self.pin_3
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_3);
        self.pin_4
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_4);
        self.pin_5
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_5);
        self.pin_6
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_6);
        self.pin_7
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_7);
    }
}

pub struct RxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> {
    pin_0: PeripheralRef<'d, P0>,
    pin_1: PeripheralRef<'d, P1>,
    pin_2: PeripheralRef<'d, P2>,
    pin_3: PeripheralRef<'d, P3>,
    pin_4: PeripheralRef<'d, P4>,
    pin_5: PeripheralRef<'d, P5>,
    pin_6: PeripheralRef<'d, P6>,
    pin_7: PeripheralRef<'d, P7>,
    pin_8: PeripheralRef<'d, P8>,
    pin_9: PeripheralRef<'d, P9>,
    pin_10: PeripheralRef<'d, P10>,
    pin_11: PeripheralRef<'d, P11>,
    pin_12: PeripheralRef<'d, P12>,
    pin_13: PeripheralRef<'d, P13>,
    pin_14: PeripheralRef<'d, P14>,
    pin_15: PeripheralRef<'d, P15>,
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
    RxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: InputPin,
    P1: InputPin,
    P2: InputPin,
    P3: InputPin,
    P4: InputPin,
    P5: InputPin,
    P6: InputPin,
    P7: InputPin,
    P8: InputPin,
    P9: InputPin,
    P10: InputPin,
    P11: InputPin,
    P12: InputPin,
    P13: InputPin,
    P14: InputPin,
    P15: InputPin,
{
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pin_0: impl Peripheral<P = P0> + 'd,
        pin_1: impl Peripheral<P = P1> + 'd,
        pin_2: impl Peripheral<P = P2> + 'd,
        pin_3: impl Peripheral<P = P3> + 'd,
        pin_4: impl Peripheral<P = P4> + 'd,
        pin_5: impl Peripheral<P = P5> + 'd,
        pin_6: impl Peripheral<P = P6> + 'd,
        pin_7: impl Peripheral<P = P7> + 'd,
        pin_8: impl Peripheral<P = P8> + 'd,
        pin_9: impl Peripheral<P = P9> + 'd,
        pin_10: impl Peripheral<P = P10> + 'd,
        pin_11: impl Peripheral<P = P11> + 'd,
        pin_12: impl Peripheral<P = P12> + 'd,
        pin_13: impl Peripheral<P = P13> + 'd,
        pin_14: impl Peripheral<P = P14> + 'd,
        pin_15: impl Peripheral<P = P15> + 'd,
    ) -> Self {
        crate::into_ref!(pin_0);
        crate::into_ref!(pin_1);
        crate::into_ref!(pin_2);
        crate::into_ref!(pin_3);
        crate::into_ref!(pin_4);
        crate::into_ref!(pin_5);
        crate::into_ref!(pin_6);
        crate::into_ref!(pin_7);
        crate::into_ref!(pin_8);
        crate::into_ref!(pin_9);
        crate::into_ref!(pin_10);
        crate::into_ref!(pin_11);
        crate::into_ref!(pin_12);
        crate::into_ref!(pin_13);
        crate::into_ref!(pin_14);
        crate::into_ref!(pin_15);

        Self {
            pin_0,
            pin_1,
            pin_2,
            pin_3,
            pin_4,
            pin_5,
            pin_6,
            pin_7,
            pin_8,
            pin_9,
            pin_10,
            pin_11,
            pin_12,
            pin_13,
            pin_14,
            pin_15,
        }
    }
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> RxPins
    for RxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: InputPin,
    P1: InputPin,
    P2: InputPin,
    P3: InputPin,
    P4: InputPin,
    P5: InputPin,
    P6: InputPin,
    P7: InputPin,
    P8: InputPin,
    P9: InputPin,
    P10: InputPin,
    P11: InputPin,
    P12: InputPin,
    P13: InputPin,
    P14: InputPin,
    P15: InputPin,
{
    type Word = u16;
    fn configure(&mut self) {
        self.pin_0
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_0);
        self.pin_1
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_1);
        self.pin_2
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_2);
        self.pin_3
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_3);
        self.pin_4
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_4);
        self.pin_5
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_5);
        self.pin_6
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_6);
        self.pin_7
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_7);
        self.pin_8
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_8);
        self.pin_9
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_9);
        self.pin_10
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_10);
        self.pin_11
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_11);
        self.pin_12
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_12);
        self.pin_13
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_13);
        self.pin_14
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_14);
        self.pin_15
            .set_to_input()
            .connect_input_to_peripheral(InputSignal::CAM_DATA_15);
    }
}

mod private {
    pub trait RxPins {
        type Word: Copy;
        fn configure(&mut self);
    }
}
