#![cfg_attr(docsrs, procmacros::doc_replace(
    "dma_channel" => {
        cfg(lcd_cam_dma_engine = "AHB_GDMA") => "DMA_CH0",
        cfg(lcd_cam_dma_engine = "AXI_GDMA") => "DMA_AXI_CH0",
    },
    "mclk_pin" => gpio_for_signal!(CAM_CLK, "GPIO15"),
    "vsync_pin" => gpio_for_signal!(CAM_V_SYNC, "GPIO6"),
    "href_pin" => gpio_for_signal!(CAM_H_ENABLE, "GPIO7"),
    "pclk_pin" => gpio_for_signal!(CAM_PCLK, "GPIO13"),
    "data0_pin" => gpio_for_signal!(CAM_DATA_0, "GPIO11"),
    "data1_pin" => gpio_for_signal!(CAM_DATA_1, "GPIO9"),
    "data2_pin" => gpio_for_signal!(CAM_DATA_2, "GPIO8"),
    "data3_pin" => gpio_for_signal!(CAM_DATA_3, "GPIO10"),
    "data4_pin" => gpio_for_signal!(CAM_DATA_4, "GPIO12"),
    "data5_pin" => gpio_for_signal!(CAM_DATA_5, "GPIO18"),
    "data6_pin" => gpio_for_signal!(CAM_DATA_6, "GPIO17"),
    "data7_pin" => gpio_for_signal!(CAM_DATA_7, "GPIO16"),
))]
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
//! # {before_snippet}
//! # use esp_hal::lcd_cam::{cam::{Camera, Config}, LcdCam};
//! # use esp_hal::dma_rx_stream_buffer;
//!
//! # let dma_buf = dma_rx_stream_buffer!(20 * 1000, 1000);
//!
//! let mclk_pin = peripherals.__mclk_pin__;
//! let vsync_pin = peripherals.__vsync_pin__;
//! let href_pin = peripherals.__href_pin__;
//! let pclk_pin = peripherals.__pclk_pin__;
//!
//! let config = Config::default().with_frequency(Rate::from_mhz(20));
//!
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//! let mut camera = Camera::new(lcd_cam.cam, peripherals.__dma_channel__, config)?
//!     .with_master_clock(mclk_pin) // Remove this for slave mode
//!     .with_pixel_clock(pclk_pin)
//!     .with_vsync(vsync_pin)
//!     .with_h_enable(href_pin)
//!     .with_data0(peripherals.__data0_pin__)
//!     .with_data1(peripherals.__data1_pin__)
//!     .with_data2(peripherals.__data2_pin__)
//!     .with_data3(peripherals.__data3_pin__)
//!     .with_data4(peripherals.__data4_pin__)
//!     .with_data5(peripherals.__data5_pin__)
//!     .with_data6(peripherals.__data6_pin__)
//!     .with_data7(peripherals.__data7_pin__);
//!
//! let transfer = camera.receive(dma_buf).map_err(|e| e.0)?;
//!
//! # {after_snippet}
//! ```
//!
//! ## Asynchronous Reception
//!
//! [`Camera::into_async`] enables DMA interrupt-driven EOF waits. The EOF
//! boundary is selected by [`Config::with_eof_mode`]. Call
//! [`CameraTransfer::stop`] after waiting to return the camera and buffer.
//!
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::{dma_rx_buffer, lcd_cam::{LcdCam, cam::{Camera, Config, EofMode}}};
//! # let camera = Camera::new(LcdCam::new(peripherals.LCD_CAM).cam,
//! #     peripherals.__dma_channel__, Config::default().with_eof_mode(EofMode::ByteLen(2499)))?;
//! # let buffer = dma_rx_buffer!(2500).unwrap();
//! let camera = camera.into_async();
//! let mut transfer = camera.receive(buffer).map_err(|(error, _, _)| error)?;
//! transfer.wait_for_dma_eof().await?;
//! let (camera, buffer) = transfer.stop();
//! # {after_snippet}
//! ```

use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use crate::{
    Async,
    Blocking,
    DriverMode,
    dma::{ChannelRx, DmaError, DmaPeripheral, DmaRxBuffer, asynch::DmaRxFuture},
    gpio::{
        InputConfig,
        InputSignal,
        OutputConfig,
        OutputSignal,
        interconnect::{PeripheralInput, PeripheralOutput},
    },
    lcd_cam::{
        BitOrder,
        ByteOrder,
        CamDmaRxChannel,
        ClockError,
        ErasedRxChannel,
        calculate_clkm,
        ll,
    },
    pac,
    peripherals::LCD_CAM,
    soc::clocks::{ClockTree, LcdCamCamClockConfig, LcdCamInstance},
    system::{self, GenericPeripheralGuard},
    time::Rate,
};

/// Generation of GDMA SUC EOF
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EofMode {
    /// Generates GDMA SUC EOF by data byte length.
    ///
    /// When the length of received data reaches this value + 1, GDMA in_suc_eof is triggered.
    ByteLen(u16),
    /// Generates GDMA SUC EOF by the vsync signal.
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

/// Vsync/Hsync or Data Enable Mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum VhdeMode {
    /// VSYNC + HSYNC mode is selected, in this mode,
    /// the signals of VSYNC, HSYNC and DE are used to control the data.
    /// For this case, the VSYNC, HSYNC, and DE signal lines must be wired.
    VsyncHsync,

    /// DE mode is selected, the signals of VSYNC and
    /// DE are used to control the data. For this case, wiring HSYNC signal
    /// line is not a must. But in this case, the YUV-RGB conversion
    /// function of camera module is not available.
    De,
}

/// Vsync Filter Threshold
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// The frequency is out of range.
    Clock(ClockError),

    /// The line interrupt value is too large. Max value is 127.
    LineInterrupt,
}

/// Represents the camera interface.
pub struct Cam<'d> {
    /// The LCD_CAM peripheral reference for managing the camera functionality.
    pub(crate) lcd_cam: LCD_CAM<'d>,
    pub(super) _guard: GenericPeripheralGuard<{ system::Peripheral::LcdCam as u8 }>,
    pub(super) clock_requested: bool,
}

impl Drop for Cam<'_> {
    fn drop(&mut self) {
        if self.clock_requested {
            ClockTree::with(|clocks| LcdCamInstance::LcdCam.release_cam_clock(clocks));
        }
    }
}

/// Represents the camera interface with DMA support.
pub struct Camera<'d, Dm: DriverMode = Blocking> {
    cam: Cam<'d>,
    rx_channel: ChannelRx<Dm, ErasedRxChannel<'d>>,
}

impl<'d> Camera<'d, Blocking> {
    /// Creates a new `Camera` instance with DMA support.
    pub fn new(
        cam: Cam<'d>,
        channel: impl CamDmaRxChannel<'d>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let rx_channel = ChannelRx::new(channel.into());
        rx_channel.runtime_ensure_compatible(DmaPeripheral::LCD_CAM);

        let mut this = Self { cam, rx_channel };

        this.apply_config(&config)?;

        Ok(this)
    }

    /// Reconfigures the DMA channel for asynchronous operation.
    pub fn into_async(self) -> Camera<'d, Async> {
        Camera {
            cam: self.cam,
            rx_channel: self.rx_channel.into_async(),
        }
    }
}

impl<'d> Camera<'d, Async> {
    /// Reconfigures the DMA channel for blocking operation.
    pub fn into_blocking(self) -> Camera<'d, Blocking> {
        Camera {
            cam: self.cam,
            rx_channel: self.rx_channel.into_blocking(),
        }
    }
}

impl<Dm: DriverMode> Camera<'_, Dm> {
    fn regs(&self) -> &pac::lcd_cam::RegisterBlock {
        self.cam.lcd_cam.register_block()
    }

    /// Applies the configuration to the camera interface.
    ///
    /// # Errors
    ///
    /// [`ConfigError::Clock`] when the frequency passed in `Config` is too low.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let sources = property!("clock_tree.lcd_cam.cam_clock.sclk");
        let (i, divider) = calculate_clkm(
            config.frequency.as_hz(),
            &sources.map(LcdCamInstance::cam_clock_source_frequency),
        )
        .map_err(ConfigError::Clock)?;
        let clock_config =
            LcdCamCamClockConfig::new(sources[i], divider.div_num, divider.div_a, divider.div_b);

        if let Some(value) = config.line_interrupt
            && value > 0b0111_1111
        {
            return Err(ConfigError::LineInterrupt);
        }

        ClockTree::with(|clocks| {
            LcdCamInstance::LcdCam.configure_cam_clock(clocks, clock_config);
            if !self.cam.clock_requested {
                LcdCamInstance::LcdCam.request_cam_clock(clocks);
                self.cam.clock_requested = true;
            }
        });

        self.regs().cam_ctrl().modify(|_, w| unsafe {
            w.cam_vsync_filter_thres().bits(
                config
                    .vsync_filter_threshold
                    .map_or(0, |threshold| threshold as _),
            );
            w.cam_byte_order()
                .bit(config.byte_order != ByteOrder::default());
            w.cam_bit_order()
                .bit(config.bit_order != BitOrder::default());
            w.cam_vs_eof_en()
                .bit(matches!(config.eof_mode, EofMode::VsyncSignal));
            w.cam_line_int_en().bit(config.line_interrupt.is_some());
            w.cam_stop_en().clear_bit()
        });
        self.regs().cam_ctrl1().write(|w| unsafe {
            w.cam_2byte_en().bit(config.enable_2byte_mode);
            w.cam_vh_de_mode_en()
                .bit(matches!(config.vh_de_mode, VhdeMode::VsyncHsync));
            if let EofMode::ByteLen(value) = config.eof_mode {
                w.cam_rec_data_bytelen().bits(value);
            }
            if let Some(value) = config.line_interrupt {
                w.cam_line_int_num().bits(value);
            }
            w.cam_vsync_filter_en()
                .bit(config.vsync_filter_threshold.is_some());
            w.cam_clk_inv().bit(config.invert_pixel_clock);
            w.cam_de_inv().bit(config.invert_h_enable);
            w.cam_hsync_inv().bit(config.invert_hsync);
            w.cam_vsync_inv().bit(config.invert_vsync)
        });

        ll::set_cam_conv_bypass(self.regs());

        self.regs()
            .cam_ctrl()
            .modify(|_, w| w.cam_update().set_bit());

        Ok(())
    }
}

impl<'d, Dm: DriverMode> Camera<'d, Dm> {
    /// Configures the master clock (MCLK) pin for the camera interface.
    pub fn with_master_clock(self, mclk: impl PeripheralOutput<'d>) -> Self {
        let mclk = mclk.into();

        mclk.apply_output_config(&OutputConfig::default());
        mclk.set_output_enable(true);

        OutputSignal::CAM_CLK.connect_to(&mclk);

        self
    }

    /// Configures the pixel clock (PCLK) pin for the camera interface.
    pub fn with_pixel_clock(self, pclk: impl PeripheralInput<'d>) -> Self {
        let pclk = pclk.into();

        pclk.apply_input_config(&InputConfig::default());
        pclk.set_input_enable(true);
        InputSignal::CAM_PCLK.connect_to(&pclk);

        self
    }

    /// Configures the Vertical Sync (VSYNC) pin for the camera interface.
    pub fn with_vsync(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.apply_input_config(&InputConfig::default());
        pin.set_input_enable(true);
        InputSignal::CAM_V_SYNC.connect_to(&pin);

        self
    }

    /// Configures the Horizontal Sync (HSYNC) pin for the camera interface.
    pub fn with_hsync(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.apply_input_config(&InputConfig::default());
        pin.set_input_enable(true);
        InputSignal::CAM_H_SYNC.connect_to(&pin);

        self
    }

    /// Configures the Horizontal Enable (HENABLE) pin for the camera interface.
    ///
    /// Also known as "Data Enable".
    pub fn with_h_enable(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.apply_input_config(&InputConfig::default());
        pin.set_input_enable(true);
        InputSignal::CAM_H_ENABLE.connect_to(&pin);

        self
    }

    fn with_data_pin(self, signal: InputSignal, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.apply_input_config(&InputConfig::default());
        pin.set_input_enable(true);
        signal.connect_to(&pin);

        self
    }

    /// Configures the DATA 0 pin for the camera interface.
    pub fn with_data0(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_0, pin)
    }

    /// Configures the DATA 1 pin for the camera interface.
    pub fn with_data1(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_1, pin)
    }

    /// Configures the DATA 2 pin for the camera interface.
    pub fn with_data2(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_2, pin)
    }

    /// Configures the DATA 3 pin for the camera interface.
    pub fn with_data3(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_3, pin)
    }

    /// Configures the DATA 4 pin for the camera interface.
    pub fn with_data4(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_4, pin)
    }

    /// Configures the DATA 5 pin for the camera interface.
    pub fn with_data5(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_5, pin)
    }

    /// Configures the DATA 6 pin for the camera interface.
    pub fn with_data6(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_6, pin)
    }

    /// Configures the DATA 7 pin for the camera interface.
    pub fn with_data7(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_7, pin)
    }

    /// Configures the DATA 8 pin for the camera interface.
    pub fn with_data8(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_8, pin)
    }

    /// Configures the DATA 9 pin for the camera interface.
    pub fn with_data9(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_9, pin)
    }

    /// Configures the DATA 10 pin for the camera interface.
    pub fn with_data10(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_10, pin)
    }

    /// Configures the DATA 11 pin for the camera interface.
    pub fn with_data11(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_11, pin)
    }

    /// Configures the DATA 12 pin for the camera interface.
    pub fn with_data12(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_12, pin)
    }

    /// Configures the DATA 13 pin for the camera interface.
    pub fn with_data13(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_13, pin)
    }

    /// Configures the DATA 14 pin for the camera interface.
    pub fn with_data14(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_14, pin)
    }

    /// Configures the DATA 15 pin for the camera interface.
    pub fn with_data15(self, pin: impl PeripheralInput<'d>) -> Self {
        self.with_data_pin(InputSignal::CAM_DATA_15, pin)
    }

    /// Starts a DMA transfer to receive data from the camera peripheral.
    pub fn receive<BUF: DmaRxBuffer>(
        mut self,
        mut buf: BUF,
    ) -> Result<CameraTransfer<'d, BUF, Dm>, (DmaError, Self, BUF)> {
        // Reset Camera control unit and Async Rx FIFO
        self.regs()
            .cam_ctrl1()
            .modify(|_, w| w.cam_reset().set_bit());
        self.regs()
            .cam_ctrl1()
            .modify(|_, w| w.cam_reset().clear_bit());
        self.regs()
            .cam_ctrl1()
            .modify(|_, w| w.cam_afifo_reset().set_bit());
        self.regs()
            .cam_ctrl1()
            .modify(|_, w| w.cam_afifo_reset().clear_bit());

        // Start DMA to receive incoming transfer.
        let result = unsafe {
            self.rx_channel
                .prepare_transfer(DmaPeripheral::LCD_CAM, &mut buf)
                .and_then(|_| self.rx_channel.start_transfer())
        };

        if let Err(e) = result {
            return Err((e, self, buf));
        }

        // Start the Camera unit to listen for incoming DVP stream.
        self.regs().cam_ctrl().modify(|_, w| {
            // Automatically stops the camera unit once the GDMA Rx FIFO is full.
            w.cam_stop_en().set_bit();

            w.cam_update().set_bit()
        });
        self.regs()
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().set_bit());

        Ok(CameraTransfer {
            camera: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
            eof_error: None,
        })
    }
}

/// Represents an ongoing (or potentially stopped) transfer from the Camera to a
/// DMA buffer.
pub struct CameraTransfer<'d, BUF: DmaRxBuffer, Dm: DriverMode = Blocking> {
    camera: ManuallyDrop<Camera<'d, Dm>>,
    buffer_view: ManuallyDrop<BUF::View>,
    eof_error: Option<DmaError>,
}

impl<'d, BUF: DmaRxBuffer, Dm: DriverMode> CameraTransfer<'d, BUF, Dm> {
    /// Returns whether [`Self::wait`] will not block.
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
            .regs()
            .cam_ctrl1()
            .read()
            .cam_start()
            .bit_is_clear()
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn stop(mut self) -> (Camera<'d, Dm>, BUF::Final) {
        self.stop_peripherals();
        let (camera, view) = self.release();
        (camera, BUF::from_view(view))
    }

    /// Waits for the transfer to stop and returns the peripheral and buffer.
    ///
    /// The camera does not really "finish" its transfer, so this typically
    /// waits for a DMA error. Call [`Self::stop`] once the needed data is
    /// available.
    pub fn wait(mut self) -> (Result<(), DmaError>, Camera<'d, Dm>, BUF::Final) {
        while !self.is_done() {}

        // Stop the DMA as it doesn't know that the camera has stopped.
        self.camera.rx_channel.stop_transfer();

        // Note: There is no "done" interrupt to clear.

        let result = match self.eof_error {
            Some(error) => Err(error),
            None if self.camera.rx_channel.has_error() => Err(DmaError::DescriptorError),
            None => Ok(()),
        };
        let (camera, view) = self.release();

        (result, camera, BUF::from_view(view))
    }

    fn release(mut self) -> (Camera<'d, Dm>, BUF::View) {
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
            .regs()
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().clear_bit());

        // Stop the DMA
        self.camera.rx_channel.stop_transfer();
    }
}

impl<BUF: DmaRxBuffer> CameraTransfer<'_, BUF, Async> {
    /// Waits for a DMA EOF or receive error in this transfer.
    ///
    /// The EOF boundary is configured by [`Config::with_eof_mode`].
    /// Reception can start partway through a frame, so EOF alone does not
    /// guarantee that a complete frame has been received.
    ///
    /// This does not stop the camera or DMA. Call [`Self::stop`] to return
    /// the camera and buffer. After a successful wait, another call can wait
    /// for a later EOF. Receive errors are retained for this transfer.
    ///
    /// # Errors
    ///
    /// Returns [`DmaError::DescriptorError`] if DMA reports an invalid
    /// descriptor, descriptor exhaustion, or an error EOF.
    /// Receive errors take precedence over a pending EOF.
    ///
    /// # Cancellation Safety
    ///
    /// Dropping the future does not stop the camera or DMA. The wait can be retried.
    pub async fn wait_for_dma_eof(&mut self) -> Result<(), DmaError> {
        if let Some(error) = self.eof_error {
            return Err(error);
        }
        let result = DmaRxFuture::new(&mut self.camera.rx_channel).await;
        if let Err(error) = result {
            self.eof_error = Some(error);
        }
        result
    }
}

impl<BUF: DmaRxBuffer, Dm: DriverMode> Deref for CameraTransfer<'_, BUF, Dm> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<BUF: DmaRxBuffer, Dm: DriverMode> DerefMut for CameraTransfer<'_, BUF, Dm> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<BUF: DmaRxBuffer, Dm: DriverMode> Drop for CameraTransfer<'_, BUF, Dm> {
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

#[derive(Debug, Clone, Copy, PartialEq, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Configuration settings for the Camera interface.
pub struct Config {
    /// The pixel clock frequency for the camera interface.
    frequency: Rate,

    /// Enables 16 bit mode (instead of 8 bit).
    enable_2byte_mode: bool,

    /// The byte order for the camera data.
    byte_order: ByteOrder,

    /// The bit order for the camera data.
    bit_order: BitOrder,

    /// Vsync/Hsync or Data Enable Mode
    vh_de_mode: VhdeMode,

    /// The Vsync filter threshold.
    vsync_filter_threshold: Option<VsyncFilterThreshold>,

    /// Conditions under which Camera should emit a SUC_EOF to the DMA.
    eof_mode: EofMode,

    /// If set, the line interrupt is enabled and will be triggered when
    /// the number of received lines reaches this value + 1.
    ///
    /// This is a 7 bit value which means a max of 128 lines.
    line_interrupt: Option<u8>,

    /// Inverts VSYNC signal, valid in high level.
    invert_vsync: bool,

    /// Inverts HSYNC signal, valid in high level.
    invert_hsync: bool,

    /// Inverts H_ENABLE signal (Also known as "Data Enable"), valid in high level.
    invert_h_enable: bool,

    /// Inverts PCLK signal.
    invert_pixel_clock: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Rate::from_mhz(20),
            enable_2byte_mode: false,
            byte_order: Default::default(),
            bit_order: Default::default(),
            vh_de_mode: VhdeMode::De,
            vsync_filter_threshold: None,
            eof_mode: EofMode::VsyncSignal,
            line_interrupt: None,
            invert_vsync: false,
            invert_hsync: false,
            invert_h_enable: false,
            invert_pixel_clock: false,
        }
    }
}
