//! # Camera - Master or Slave Mode
//!
//! ## Overview
//! The LCD_CAM peripheral supports receiving 8/16 bit DVP signals in either
//! master or slave mode. In master mode, the peripheral provides the master
//! clock to drive the camera, in slave mode it does not. This is configured
//! with the `with_master_clock` method on the camera driver. The driver (due to
//! the peripheral) mandates DMA (Direct Memory Access) for efficient data
//! transfer.
//!
//! ## Examples
//! Following code shows how to receive some bytes from an 8 bit DVP stream in
//! master mode.
//!
//! ```no_run
//! let mclk_pin = io.pins.gpio15;
//! let vsync_pin = io.pins.gpio6;
//! let href_pin = io.pins.gpio7;
//! let pclk_pin = io.pins.gpio13;
//! let data_pins = RxEightBits::new(
//!     io.pins.gpio11,
//!     io.pins.gpio9,
//!     io.pins.gpio8,
//!     io.pins.gpio10,
//!     io.pins.gpio12,
//!     io.pins.gpio18,
//!     io.pins.gpio17,
//!     io.pins.gpio16,
//! );
//!
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//! let mut camera = Camera::new(lcd_cam.cam, channel.rx, data_pins, 20u32.MHz(), &clocks)
//!     .with_master_clock(mclk_pin) // Remove this for slave mode.
//!     .with_ctrl_pins(vsync_pin, href_pin, pclk_pin);
//! ```

use core::mem::size_of;

use embedded_dma::WriteBuffer;
use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{
        dma_private::{DmaSupport, DmaSupportRx},
        ChannelRx,
        ChannelTypes,
        DmaError,
        DmaPeripheral,
        DmaTransferRx,
        DmaTransferRxCircular,
        LcdCamPeripheral,
        RegisterAccess,
        Rx,
        RxChannel,
        RxPrivate,
    },
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    lcd_cam::{cam::private::RxPins, private::calculate_clkm, BitOrder, ByteOrder},
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

/// Vsync Filter Threshold
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VsyncFilterThreshold {
    One,
    Two,
    Three,
    Four,
    Five,
    Six,
    Seven,
    Eight,
}

pub struct Cam<'d> {
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,
}

pub struct Camera<'d, RX> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    rx_channel: RX,
    // 1 or 2
    bus_width: usize,
}

impl<'d, T, R> Camera<'d, ChannelRx<'d, T, R>>
where
    T: RxChannel<R>,
    R: ChannelTypes + RegisterAccess,
    R::P: LcdCamPeripheral,
{
    pub fn new<P: RxPins>(
        cam: Cam<'d>,
        mut channel: ChannelRx<'d, T, R>,
        _pins: P,
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
                    .cam_line_int_en()
                    .clear_bit()
                    .cam_stop_en()
                    .clear_bit()
            }
        });
        lcd_cam.cam_ctrl1().write(|w| unsafe {
            w.cam_vh_de_mode_en()
                .set_bit()
                .cam_rec_data_bytelen()
                .bits(0)
                .cam_line_int_num()
                .bits(0)
                .cam_vsync_filter_en()
                .clear_bit()
                .cam_2byte_en()
                .clear_bit()
                .cam_clk_inv()
                .clear_bit()
                .cam_de_inv()
                .clear_bit()
                .cam_hsync_inv()
                .clear_bit()
                .cam_vsync_inv()
                .clear_bit()
        });

        lcd_cam
            .cam_rgb_yuv()
            .write(|w| w.cam_conv_bypass().clear_bit());

        lcd_cam.cam_ctrl().modify(|_, w| w.cam_update().set_bit());

        channel.init_channel();

        Self {
            lcd_cam,
            rx_channel: channel,
            bus_width: P::BUS_WIDTH,
        }
    }
}

impl<'d, RX: Rx> DmaSupport for Camera<'d, RX> {
    fn peripheral_wait_dma(&mut self, _is_tx: bool, _is_rx: bool) {
        while !
        // Wait for IN_SUC_EOF (i.e. VSYNC)
        self.rx_channel.is_done() ||
        // Or for IN_DSCR_EMPTY (i.e. No more buffer space)
        self.rx_channel.has_dscr_empty_error() ||
        // Or for IN_DSCR_ERR (i.e. bad descriptor)
        self.rx_channel.has_error()
        {}
    }

    fn peripheral_dma_stop(&mut self) {
        // TODO: Stop DMA?? self.instance.rx_channel.stop_transfer();
    }
}

impl<'d, RX: Rx> DmaSupportRx for Camera<'d, RX> {
    type RX = RX;

    fn rx(&mut self) -> &mut Self::RX {
        &mut self.rx_channel
    }
}

impl<'d, RX: Rx> Camera<'d, RX> {
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        self.lcd_cam
            .cam_ctrl()
            .modify(|_, w| w.cam_byte_order().bit(byte_order != ByteOrder::default()));
        self
    }

    pub fn set_bit_order(&mut self, bit_order: BitOrder) -> &mut Self {
        self.lcd_cam
            .cam_ctrl()
            .modify(|_, w| w.cam_bit_order().bit(bit_order != BitOrder::default()));
        self
    }

    pub fn set_vsync_filter(&mut self, threshold: Option<VsyncFilterThreshold>) -> &mut Self {
        if let Some(threshold) = threshold {
            let value = match threshold {
                VsyncFilterThreshold::One => 0,
                VsyncFilterThreshold::Two => 1,
                VsyncFilterThreshold::Three => 2,
                VsyncFilterThreshold::Four => 3,
                VsyncFilterThreshold::Five => 4,
                VsyncFilterThreshold::Six => 5,
                VsyncFilterThreshold::Seven => 6,
                VsyncFilterThreshold::Eight => 7,
            };

            self.lcd_cam
                .cam_ctrl()
                .modify(|_, w| unsafe { w.cam_vsync_filter_thres().bits(value) });
            self.lcd_cam
                .cam_ctrl1()
                .modify(|_, w| w.cam_vsync_filter_en().set_bit());
        } else {
            self.lcd_cam
                .cam_ctrl1()
                .modify(|_, w| w.cam_vsync_filter_en().clear_bit());
        }
        self
    }

    pub fn with_master_clock<MCLK: OutputPin>(self, mclk: impl Peripheral<P = MCLK> + 'd) -> Self {
        crate::into_ref!(mclk);
        mclk.set_to_push_pull_output(crate::private::Internal);
        mclk.connect_peripheral_to_output(OutputSignal::CAM_CLK, crate::private::Internal);
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

        vsync.set_to_input(crate::private::Internal);
        vsync.connect_input_to_peripheral(InputSignal::CAM_V_SYNC, crate::private::Internal);
        h_enable.set_to_input(crate::private::Internal);
        h_enable.connect_input_to_peripheral(InputSignal::CAM_H_ENABLE, crate::private::Internal);
        pclk.set_to_input(crate::private::Internal);
        pclk.connect_input_to_peripheral(InputSignal::CAM_PCLK, crate::private::Internal);

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
            .cam_ctrl()
            .modify(|_, w| w.cam_update().set_bit());
        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().set_bit());
    }

    fn start_dma<RXBUF: WriteBuffer>(
        &mut self,
        circular: bool,
        buf: &mut RXBUF,
    ) -> Result<(), DmaError> {
        let (ptr, len) = unsafe { buf.write_buffer() };

        assert_eq!(self.bus_width, size_of::<RXBUF::Word>());

        unsafe {
            self.rx_channel.prepare_transfer_without_start(
                circular,
                DmaPeripheral::LcdCam,
                ptr as _,
                len * size_of::<RXBUF::Word>(),
            )?;
        }
        self.rx_channel.start_transfer()
    }

    pub fn read_dma<'t, RXBUF: WriteBuffer>(
        &'t mut self,
        buf: &'t mut RXBUF,
    ) -> Result<DmaTransferRx<Self>, DmaError> {
        self.reset_unit_and_fifo();
        // Start DMA to receive incoming transfer.
        self.start_dma(false, buf)?;
        self.start_unit();

        Ok(DmaTransferRx::new(self))
    }

    pub fn read_dma_circular<'t, RXBUF: WriteBuffer>(
        &'t mut self,
        buf: &'t mut RXBUF,
    ) -> Result<DmaTransferRxCircular<Self>, DmaError> {
        self.reset_unit_and_fifo();
        // Start DMA to receive incoming transfer.
        self.start_dma(true, buf)?;
        self.start_unit();

        Ok(DmaTransferRxCircular::new(self))
    }
}

pub struct RxEightBits {
    _pins: (),
}

impl RxEightBits {
    #[allow(clippy::too_many_arguments)]
    pub fn new<'d, P0, P1, P2, P3, P4, P5, P6, P7>(
        pin_0: impl Peripheral<P = P0> + 'd,
        pin_1: impl Peripheral<P = P1> + 'd,
        pin_2: impl Peripheral<P = P2> + 'd,
        pin_3: impl Peripheral<P = P3> + 'd,
        pin_4: impl Peripheral<P = P4> + 'd,
        pin_5: impl Peripheral<P = P5> + 'd,
        pin_6: impl Peripheral<P = P6> + 'd,
        pin_7: impl Peripheral<P = P7> + 'd,
    ) -> Self
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
        crate::into_ref!(pin_0);
        crate::into_ref!(pin_1);
        crate::into_ref!(pin_2);
        crate::into_ref!(pin_3);
        crate::into_ref!(pin_4);
        crate::into_ref!(pin_5);
        crate::into_ref!(pin_6);
        crate::into_ref!(pin_7);

        pin_0.set_to_input(crate::private::Internal);
        pin_0.connect_input_to_peripheral(InputSignal::CAM_DATA_0, crate::private::Internal);
        pin_1.set_to_input(crate::private::Internal);
        pin_1.connect_input_to_peripheral(InputSignal::CAM_DATA_1, crate::private::Internal);
        pin_2.set_to_input(crate::private::Internal);
        pin_2.connect_input_to_peripheral(InputSignal::CAM_DATA_2, crate::private::Internal);
        pin_3.set_to_input(crate::private::Internal);
        pin_3.connect_input_to_peripheral(InputSignal::CAM_DATA_3, crate::private::Internal);
        pin_4.set_to_input(crate::private::Internal);
        pin_4.connect_input_to_peripheral(InputSignal::CAM_DATA_4, crate::private::Internal);
        pin_5.set_to_input(crate::private::Internal);
        pin_5.connect_input_to_peripheral(InputSignal::CAM_DATA_5, crate::private::Internal);
        pin_6.set_to_input(crate::private::Internal);
        pin_6.connect_input_to_peripheral(InputSignal::CAM_DATA_6, crate::private::Internal);
        pin_7.set_to_input(crate::private::Internal);
        pin_7.connect_input_to_peripheral(InputSignal::CAM_DATA_7, crate::private::Internal);

        Self { _pins: () }
    }
}

impl RxPins for RxEightBits {
    const BUS_WIDTH: usize = 1;
}

pub struct RxSixteenBits {
    _pins: (),
}

impl RxSixteenBits {
    #[allow(clippy::too_many_arguments)]
    pub fn new<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>(
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
    ) -> Self
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

        pin_0.set_to_input(crate::private::Internal);
        pin_0.connect_input_to_peripheral(InputSignal::CAM_DATA_0, crate::private::Internal);
        pin_1.set_to_input(crate::private::Internal);
        pin_1.connect_input_to_peripheral(InputSignal::CAM_DATA_1, crate::private::Internal);
        pin_2.set_to_input(crate::private::Internal);
        pin_2.connect_input_to_peripheral(InputSignal::CAM_DATA_2, crate::private::Internal);
        pin_3.set_to_input(crate::private::Internal);
        pin_3.connect_input_to_peripheral(InputSignal::CAM_DATA_3, crate::private::Internal);
        pin_4.set_to_input(crate::private::Internal);
        pin_4.connect_input_to_peripheral(InputSignal::CAM_DATA_4, crate::private::Internal);
        pin_5.set_to_input(crate::private::Internal);
        pin_5.connect_input_to_peripheral(InputSignal::CAM_DATA_5, crate::private::Internal);
        pin_6.set_to_input(crate::private::Internal);
        pin_6.connect_input_to_peripheral(InputSignal::CAM_DATA_6, crate::private::Internal);
        pin_7.set_to_input(crate::private::Internal);
        pin_7.connect_input_to_peripheral(InputSignal::CAM_DATA_7, crate::private::Internal);
        pin_8.set_to_input(crate::private::Internal);
        pin_8.connect_input_to_peripheral(InputSignal::CAM_DATA_8, crate::private::Internal);
        pin_9.set_to_input(crate::private::Internal);
        pin_9.connect_input_to_peripheral(InputSignal::CAM_DATA_9, crate::private::Internal);
        pin_10.set_to_input(crate::private::Internal);
        pin_10.connect_input_to_peripheral(InputSignal::CAM_DATA_10, crate::private::Internal);
        pin_11.set_to_input(crate::private::Internal);
        pin_11.connect_input_to_peripheral(InputSignal::CAM_DATA_11, crate::private::Internal);
        pin_12.set_to_input(crate::private::Internal);
        pin_12.connect_input_to_peripheral(InputSignal::CAM_DATA_12, crate::private::Internal);
        pin_13.set_to_input(crate::private::Internal);
        pin_13.connect_input_to_peripheral(InputSignal::CAM_DATA_13, crate::private::Internal);
        pin_14.set_to_input(crate::private::Internal);
        pin_14.connect_input_to_peripheral(InputSignal::CAM_DATA_14, crate::private::Internal);
        pin_15.set_to_input(crate::private::Internal);
        pin_15.connect_input_to_peripheral(InputSignal::CAM_DATA_15, crate::private::Internal);

        Self { _pins: () }
    }
}

impl RxPins for RxSixteenBits {
    const BUS_WIDTH: usize = 2;
}

mod private {
    pub trait RxPins {
        const BUS_WIDTH: usize;
    }
}
