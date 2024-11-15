//! # Camera - Master or Slave Mode
//!
//! ## Overview
//! The camera module is designed to receive parallel video data signals, and
//! its bus supports DVP 8-/16-bit modes in master or slave mode.
//!
//! ## Configuration
//! In master mode, the peripheral provides the master clock to drive the
//! camera, in slave mode it does not. This is configured with the
//! `with_master_clock` method on the camera driver. The driver (due to the
//! peripheral) mandates DMA (Direct Memory Access) for efficient data transfer.
//!
//! ## Examples
//! ## Master Mode
//! Following code shows how to receive some bytes from an 8 bit DVP stream in
//! master mode.
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::lcd_cam::{cam::{Camera, RxEightBits}, LcdCam};
//! # use fugit::RateExtU32;
//! # use esp_hal::dma_rx_stream_buffer;
//! # use esp_hal::dma::{Dma, DmaPriority};
//!
//! # let dma = Dma::new(peripherals.DMA);
//! # let channel = dma.channel0;
//!
//! # let dma_buf = dma_rx_stream_buffer!(20 * 1000, 1000);
//!
//! # let channel = channel.configure(
//! #     false,
//! #     DmaPriority::Priority0,
//! # );
//!
//! let mclk_pin = peripherals.GPIO15;
//! let vsync_pin = peripherals.GPIO6;
//! let href_pin = peripherals.GPIO7;
//! let pclk_pin = peripherals.GPIO13;
//! let data_pins = RxEightBits::new(
//!     peripherals.GPIO11,
//!     peripherals.GPIO9,
//!     peripherals.GPIO8,
//!     peripherals.GPIO10,
//!     peripherals.GPIO12,
//!     peripherals.GPIO18,
//!     peripherals.GPIO17,
//!     peripherals.GPIO16,
//! );
//!
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//! let mut camera = Camera::new(
//!     lcd_cam.cam,
//!     channel.rx,
//!     data_pins,
//!     20u32.MHz(),
//! )
//! // Remove this for slave mode.
//! .with_master_clock(mclk_pin)
//! .with_pixel_clock(pclk_pin)
//! .with_ctrl_pins(vsync_pin, href_pin);
//!
//! let transfer = camera.receive(dma_buf).map_err(|e| e.0).unwrap();
//!
//! # }
//! ```

use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{ChannelRx, DmaChannelConvert, DmaEligible, DmaError, DmaPeripheral, DmaRxBuffer, Rx},
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        InputSignal,
        OutputSignal,
        Pull,
    },
    lcd_cam::{calculate_clkm, BitOrder, ByteOrder},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
    system::PeripheralGuard,
    Blocking,
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
    /// Requires 1 valid VSYNC pulse to trigger synchronization.
    One,
    /// Requires 2 valid VSYNC pulse to trigger synchronization.
    Two,
    /// Requires 3 valid VSYNC pulse to trigger synchronization.
    Three,
    /// Requires 4 valid VSYNC pulse to trigger synchronization.
    Four,
    /// Requires 5 valid VSYNC pulse to trigger synchronization.
    Five,
    /// Requires 6 valid VSYNC pulse to trigger synchronization.
    Six,
    /// Requires 7 valid VSYNC pulse to trigger synchronization.
    Seven,
    /// Requires 8 valid VSYNC pulse to trigger synchronization.
    Eight,
}

/// Represents the camera interface.
pub struct Cam<'d> {
    /// The LCD_CAM peripheral reference for managing the camera functionality.
    pub(crate) lcd_cam: PeripheralRef<'d, LCD_CAM>,
    pub(super) _guard: PeripheralGuard,
}

/// Represents the camera interface with DMA support.
pub struct Camera<'d> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    rx_channel: ChannelRx<'d, Blocking, <LCD_CAM as DmaEligible>::Dma>,
    _guard: PeripheralGuard,
}

impl<'d> Camera<'d> {
    /// Creates a new `Camera` instance with DMA support.
    pub fn new<P, CH>(
        cam: Cam<'d>,
        channel: ChannelRx<'d, Blocking, CH>,
        _pins: P,
        frequency: HertzU32,
    ) -> Self
    where
        CH: DmaChannelConvert<<LCD_CAM as DmaEligible>::Dma>,
        P: RxPins,
    {
        let guard = PeripheralGuard::new(crate::system::Peripheral::LcdCam);

        let lcd_cam = cam.lcd_cam;

        let clocks = Clocks::get();
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
                w.cam_clk_sel().bits((i + 1) as _);
                w.cam_clkm_div_num().bits(divider.div_num as _);
                w.cam_clkm_div_b().bits(divider.div_b as _);
                w.cam_clkm_div_a().bits(divider.div_a as _);
                w.cam_vsync_filter_thres().bits(0);
                w.cam_vs_eof_en().set_bit();
                w.cam_line_int_en().clear_bit();
                w.cam_stop_en().clear_bit()
            }
        });
        lcd_cam.cam_ctrl1().write(|w| unsafe {
            w.cam_vh_de_mode_en().set_bit();
            w.cam_rec_data_bytelen().bits(0);
            w.cam_line_int_num().bits(0);
            w.cam_vsync_filter_en().clear_bit();
            w.cam_2byte_en().bit(P::BUS_WIDTH == 2);
            w.cam_clk_inv().clear_bit();
            w.cam_de_inv().clear_bit();
            w.cam_hsync_inv().clear_bit();
            w.cam_vsync_inv().clear_bit()
        });

        lcd_cam
            .cam_rgb_yuv()
            .write(|w| w.cam_conv_bypass().clear_bit());

        lcd_cam.cam_ctrl().modify(|_, w| w.cam_update().set_bit());

        Self {
            lcd_cam,
            rx_channel: channel.degrade(),
            _guard: guard,
        }
    }
}

impl<'d> Camera<'d> {
    /// Configures the byte order for the camera data.
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        self.lcd_cam
            .cam_ctrl()
            .modify(|_, w| w.cam_byte_order().bit(byte_order != ByteOrder::default()));
        self
    }

    /// Configures the bit order for the camera data.
    pub fn set_bit_order(&mut self, bit_order: BitOrder) -> &mut Self {
        self.lcd_cam
            .cam_ctrl()
            .modify(|_, w| w.cam_bit_order().bit(bit_order != BitOrder::default()));
        self
    }

    /// Configures the VSYNC filter threshold.
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

    /// Configures the master clock (MCLK) pin for the camera interface.
    pub fn with_master_clock<MCLK: PeripheralOutput>(
        self,
        mclk: impl Peripheral<P = MCLK> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(mclk);

        mclk.set_to_push_pull_output(crate::private::Internal);
        OutputSignal::CAM_CLK.connect_to(mclk);

        self
    }

    /// Configures the pixel clock (PCLK) pin for the camera interface.
    pub fn with_pixel_clock<PCLK: PeripheralInput>(
        self,
        pclk: impl Peripheral<P = PCLK> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(pclk);

        pclk.init_input(Pull::None, crate::private::Internal);
        InputSignal::CAM_PCLK.connect_to(pclk);

        self
    }

    /// Configures the control pins for the camera interface (VSYNC and
    /// HENABLE).
    pub fn with_ctrl_pins<VSYNC: PeripheralInput, HENABLE: PeripheralInput>(
        self,
        vsync: impl Peripheral<P = VSYNC> + 'd,
        h_enable: impl Peripheral<P = HENABLE> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(vsync, h_enable);

        vsync.init_input(Pull::None, crate::private::Internal);
        InputSignal::CAM_V_SYNC.connect_to(vsync);
        h_enable.init_input(Pull::None, crate::private::Internal);
        InputSignal::CAM_H_ENABLE.connect_to(h_enable);

        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_vh_de_mode_en().clear_bit());

        self
    }

    /// Configures the control pins for the camera interface (VSYNC, HSYNC, and
    /// HENABLE) with DE (data enable).
    pub fn with_ctrl_pins_and_de<
        VSYNC: PeripheralInput,
        HSYNC: PeripheralInput,
        HENABLE: PeripheralInput,
    >(
        self,
        vsync: impl Peripheral<P = VSYNC> + 'd,
        hsync: impl Peripheral<P = HSYNC> + 'd,
        h_enable: impl Peripheral<P = HENABLE> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(vsync, hsync, h_enable);

        vsync.init_input(Pull::None, crate::private::Internal);
        InputSignal::CAM_V_SYNC.connect_to(vsync);
        hsync.init_input(Pull::None, crate::private::Internal);
        InputSignal::CAM_H_SYNC.connect_to(hsync);
        h_enable.init_input(Pull::None, crate::private::Internal);
        InputSignal::CAM_H_ENABLE.connect_to(h_enable);

        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_vh_de_mode_en().set_bit());

        self
    }

    /// Starts a DMA transfer to receive data from the camera peripheral.
    pub fn receive<BUF: DmaRxBuffer>(
        mut self,
        mut buf: BUF,
    ) -> Result<CameraTransfer<'d, BUF>, (DmaError, Self, BUF)> {
        // Reset Camera control unit and Async Rx FIFO
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

        // Start DMA to receive incoming transfer.
        let result = unsafe {
            self.rx_channel
                .prepare_transfer(DmaPeripheral::LcdCam, &mut buf)
                .and_then(|_| self.rx_channel.start_transfer())
        };

        if let Err(e) = result {
            return Err((e, self, buf));
        }

        // Start the Camera unit to listen for incoming DVP stream.
        self.lcd_cam.cam_ctrl().modify(|_, w| {
            // Automatically stops the camera unit once the GDMA Rx FIFO is full.
            w.cam_stop_en().set_bit();

            w.cam_update().set_bit()
        });
        self.lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().set_bit());

        Ok(CameraTransfer {
            camera: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
        })
    }
}

/// Represents an ongoing (or potentially stopped) transfer from the Camera to a
/// DMA buffer.
pub struct CameraTransfer<'d, BUF: DmaRxBuffer> {
    camera: ManuallyDrop<Camera<'d>>,
    buffer_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF: DmaRxBuffer> CameraTransfer<'d, BUF> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        // This peripheral doesn't really "complete". As long the camera (or anything
        // pretending to be :D) sends data, it will receive it and pass it to the DMA.
        // This implementation of is_done is an opinionated one. When the transfer is
        // started, the CAM_STOP_EN bit is set, which tells the LCD_CAM to stop
        // itself when the DMA stops emptying its async RX FIFO. This will
        // typically be because the DMA ran out descriptors but there could be other
        // reasons as well.

        // In the future, a user of esp_hal may not want this behaviour, which would be
        // a reasonable ask. At which point is_done and wait would go away, and
        // the driver will stop pretending that this peripheral has some kind of
        // finish line.

        // For now, most people probably want this behaviour, so it shall be kept for
        // the sake of familiarity and similarity with other drivers.

        self.camera
            .lcd_cam
            .cam_ctrl1()
            .read()
            .cam_start()
            .bit_is_clear()
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(mut self) -> (Camera<'d>, BUF) {
        self.stop_peripherals();
        let (camera, view) = self.release();
        (camera, BUF::from_view(view))
    }

    /// Waits for the transfer to stop and returns the peripheral and buffer.
    ///
    /// Note: The camera doesn't really "finish" its transfer, so what you're
    /// really waiting for here is a DMA Error. You typically just want to
    /// call [Self::stop] once you have the data you need.
    pub fn wait(mut self) -> (Result<(), DmaError>, Camera<'d>, BUF) {
        while !self.is_done() {}

        // Stop the DMA as it doesn't know that the camera has stopped.
        self.camera.rx_channel.stop_transfer();

        // Note: There is no "done" interrupt to clear.

        let (camera, view) = self.release();

        let result = if camera.rx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, camera, BUF::from_view(view))
    }

    fn release(mut self) -> (Camera<'d>, BUF::View) {
        // SAFETY: Since forget is called on self, we know that self.camera and
        // self.buffer_view won't be touched again.
        let result = unsafe {
            let camera = ManuallyDrop::take(&mut self.camera);
            let view = ManuallyDrop::take(&mut self.buffer_view);
            (camera, view)
        };
        core::mem::forget(self);
        result
    }

    fn stop_peripherals(&mut self) {
        // Stop the LCD_CAM peripheral.
        self.camera
            .lcd_cam
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().clear_bit());

        // Stop the DMA
        self.camera.rx_channel.stop_transfer();
    }
}

impl<'d, BUF: DmaRxBuffer> Deref for CameraTransfer<'d, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<'d, BUF: DmaRxBuffer> DerefMut for CameraTransfer<'d, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<'d, BUF: DmaRxBuffer> Drop for CameraTransfer<'d, BUF> {
    fn drop(&mut self) {
        self.stop_peripherals();

        // SAFETY: This is Drop, we know that self.camera and self.buffer_view
        // won't be touched again.
        unsafe {
            ManuallyDrop::drop(&mut self.camera);
            ManuallyDrop::drop(&mut self.buffer_view);
        }
    }
}

/// Represents an 8-bit wide camera data bus.
/// Is used to configure the camera interface to receive 8-bit data.
pub struct RxEightBits {
    _pins: (),
}

impl RxEightBits {
    #[allow(clippy::too_many_arguments)]
    /// Creates a new instance of `RxEightBits`, configuring the specified pins
    /// as the 8-bit data bus.
    pub fn new<'d>(
        pin_0: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_1: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_2: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_3: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_4: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_5: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_6: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_7: impl Peripheral<P = impl PeripheralInput> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(pin_0);
        crate::into_mapped_ref!(pin_1);
        crate::into_mapped_ref!(pin_2);
        crate::into_mapped_ref!(pin_3);
        crate::into_mapped_ref!(pin_4);
        crate::into_mapped_ref!(pin_5);
        crate::into_mapped_ref!(pin_6);
        crate::into_mapped_ref!(pin_7);

        let pairs = [
            (pin_0, InputSignal::CAM_DATA_0),
            (pin_1, InputSignal::CAM_DATA_1),
            (pin_2, InputSignal::CAM_DATA_2),
            (pin_3, InputSignal::CAM_DATA_3),
            (pin_4, InputSignal::CAM_DATA_4),
            (pin_5, InputSignal::CAM_DATA_5),
            (pin_6, InputSignal::CAM_DATA_6),
            (pin_7, InputSignal::CAM_DATA_7),
        ];

        for (pin, signal) in pairs.into_iter() {
            pin.init_input(Pull::None, crate::private::Internal);
            signal.connect_to(pin);
        }

        Self { _pins: () }
    }
}

impl RxPins for RxEightBits {
    const BUS_WIDTH: usize = 1;
}

/// Represents a 16-bit wide camera data bus.
/// Is used to configure the camera interface to receive 16-bit data.
pub struct RxSixteenBits {
    _pins: (),
}

impl RxSixteenBits {
    #[allow(clippy::too_many_arguments)]
    /// Creates a new instance of `RxSixteenBits`, configuring the specified
    /// pins as the 16-bit data bus.
    pub fn new<'d>(
        pin_0: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_1: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_2: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_3: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_4: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_5: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_6: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_7: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_8: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_9: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_10: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_11: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_12: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_13: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_14: impl Peripheral<P = impl PeripheralInput> + 'd,
        pin_15: impl Peripheral<P = impl PeripheralInput> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(pin_0);
        crate::into_mapped_ref!(pin_1);
        crate::into_mapped_ref!(pin_2);
        crate::into_mapped_ref!(pin_3);
        crate::into_mapped_ref!(pin_4);
        crate::into_mapped_ref!(pin_5);
        crate::into_mapped_ref!(pin_6);
        crate::into_mapped_ref!(pin_7);
        crate::into_mapped_ref!(pin_8);
        crate::into_mapped_ref!(pin_9);
        crate::into_mapped_ref!(pin_10);
        crate::into_mapped_ref!(pin_11);
        crate::into_mapped_ref!(pin_12);
        crate::into_mapped_ref!(pin_13);
        crate::into_mapped_ref!(pin_14);
        crate::into_mapped_ref!(pin_15);

        let pairs = [
            (pin_0, InputSignal::CAM_DATA_0),
            (pin_1, InputSignal::CAM_DATA_1),
            (pin_2, InputSignal::CAM_DATA_2),
            (pin_3, InputSignal::CAM_DATA_3),
            (pin_4, InputSignal::CAM_DATA_4),
            (pin_5, InputSignal::CAM_DATA_5),
            (pin_6, InputSignal::CAM_DATA_6),
            (pin_7, InputSignal::CAM_DATA_7),
            (pin_8, InputSignal::CAM_DATA_8),
            (pin_9, InputSignal::CAM_DATA_9),
            (pin_10, InputSignal::CAM_DATA_10),
            (pin_11, InputSignal::CAM_DATA_11),
            (pin_12, InputSignal::CAM_DATA_12),
            (pin_13, InputSignal::CAM_DATA_13),
            (pin_14, InputSignal::CAM_DATA_14),
            (pin_15, InputSignal::CAM_DATA_15),
        ];

        for (pin, signal) in pairs.into_iter() {
            pin.init_input(Pull::None, crate::private::Internal);
            signal.connect_to(pin);
        }

        Self { _pins: () }
    }
}

impl RxPins for RxSixteenBits {
    const BUS_WIDTH: usize = 2;
}

#[doc(hidden)]
pub trait RxPins {
    const BUS_WIDTH: usize;
}
