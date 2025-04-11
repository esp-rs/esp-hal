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
//! # use esp_hal::lcd_cam::{cam::{Camera, Config}, LcdCam};
//! # use esp_hal::dma_rx_stream_buffer;
//!
//! # let dma_buf = dma_rx_stream_buffer!(20 * 1000, 1000);
//!
//! let mclk_pin = peripherals.GPIO15;
//! let vsync_pin = peripherals.GPIO6;
//! let href_pin = peripherals.GPIO7;
//! let pclk_pin = peripherals.GPIO13;
//!
//! let config = Config::default().with_frequency(Rate::from_mhz(20));
//!
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//! let mut camera = Camera::new(
//!     lcd_cam.cam,
//!     peripherals.DMA_CH0,
//!     config,
//! )?
//! .with_master_clock(mclk_pin) // Remove this for slave mode
//! .with_pixel_clock(pclk_pin)
//! .with_vsync(vsync_pin)
//! .with_h_enable(href_pin)
//! .with_data0(peripherals.GPIO11)
//! .with_data1(peripherals.GPIO9)
//! .with_data2(peripherals.GPIO8)
//! .with_data3(peripherals.GPIO10)
//! .with_data4(peripherals.GPIO12)
//! .with_data5(peripherals.GPIO18)
//! .with_data6(peripherals.GPIO17)
//! .with_data7(peripherals.GPIO16);
//!
//! let transfer = camera.receive(dma_buf).map_err(|e| e.0)?;
//!
//! # Ok(())
//! # }
//! ```

use core::{
    mem::ManuallyDrop,
    ops::{Deref, DerefMut},
};

use crate::{
    clock::Clocks,
    dma::{ChannelRx, DmaError, DmaPeripheral, DmaRxBuffer, PeripheralRxChannel, RxChannelFor},
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        InputSignal,
        OutputSignal,
        Pull,
    },
    lcd_cam::{calculate_clkm, BitOrder, ByteOrder, ClockError},
    pac,
    peripherals::LCD_CAM,
    system::{self, GenericPeripheralGuard},
    time::Rate,
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

/// Vsync Filter Threshold
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// The frequency is out of range.
    Clock(ClockError),
}

/// Represents the camera interface.
pub struct Cam<'d> {
    /// The LCD_CAM peripheral reference for managing the camera functionality.
    pub(crate) lcd_cam: LCD_CAM<'d>,
    pub(super) _guard: GenericPeripheralGuard<{ system::Peripheral::LcdCam as u8 }>,
}

/// Represents the camera interface with DMA support.
pub struct Camera<'d> {
    lcd_cam: LCD_CAM<'d>,
    rx_channel: ChannelRx<Blocking, PeripheralRxChannel<LCD_CAM<'d>>>,
    _guard: GenericPeripheralGuard<{ system::Peripheral::LcdCam as u8 }>,
}

impl<'d> Camera<'d> {
    /// Creates a new `Camera` instance with DMA support.
    pub fn new(
        cam: Cam<'d>,
        channel: impl RxChannelFor<LCD_CAM<'d>>,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let rx_channel = ChannelRx::new(channel.degrade());

        let mut this = Self {
            lcd_cam: cam.lcd_cam,
            rx_channel,
            _guard: cam._guard,
        };

        this.apply_config(&config)?;

        Ok(this)
    }

    fn regs(&self) -> &pac::lcd_cam::RegisterBlock {
        self.lcd_cam.register_block()
    }

    /// Applies the configuration to the camera interface.
    ///
    /// # Errors
    ///
    /// [`ConfigError::Clock`] will be returned if the frequency passed in
    /// `Config` is too low.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let clocks = Clocks::get();
        let (i, divider) = calculate_clkm(
            config.frequency.as_hz() as _,
            &[
                clocks.xtal_clock.as_hz() as _,
                clocks.cpu_clock.as_hz() as _,
                clocks.crypto_pwm_clock.as_hz() as _,
            ],
        )
        .map_err(ConfigError::Clock)?;

        self.regs().cam_ctrl().write(|w| {
            // Force enable the clock for all configuration registers.
            unsafe {
                w.cam_clk_sel().bits((i + 1) as _);
                w.cam_clkm_div_num().bits(divider.div_num as _);
                w.cam_clkm_div_b().bits(divider.div_b as _);
                w.cam_clkm_div_a().bits(divider.div_a as _);
                if let Some(threshold) = config.vsync_filter_threshold {
                    w.cam_vsync_filter_thres().bits(threshold as _);
                }
                w.cam_byte_order()
                    .bit(config.byte_order != ByteOrder::default());
                w.cam_bit_order()
                    .bit(config.bit_order != BitOrder::default());
                w.cam_vs_eof_en().set_bit();
                w.cam_line_int_en().clear_bit();
                w.cam_stop_en().clear_bit()
            }
        });
        self.regs().cam_ctrl1().write(|w| unsafe {
            w.cam_2byte_en().bit(config.enable_2byte_mode);
            w.cam_vh_de_mode_en().bit(config.vh_de_mode_en);
            w.cam_rec_data_bytelen().bits(0);
            w.cam_line_int_num().bits(0);
            w.cam_vsync_filter_en()
                .bit(config.vsync_filter_threshold.is_some());
            w.cam_clk_inv().clear_bit();
            w.cam_de_inv().clear_bit();
            w.cam_hsync_inv().clear_bit();
            w.cam_vsync_inv().clear_bit()
        });

        self.regs()
            .cam_rgb_yuv()
            .write(|w| w.cam_conv_bypass().clear_bit());

        self.regs()
            .cam_ctrl()
            .modify(|_, w| w.cam_update().set_bit());

        Ok(())
    }
}

impl<'d> Camera<'d> {
    /// Configures the master clock (MCLK) pin for the camera interface.
    pub fn with_master_clock(self, mclk: impl PeripheralOutput<'d>) -> Self {
        let mclk = mclk.into();
        mclk.set_to_push_pull_output();
        OutputSignal::CAM_CLK.connect_to(&mclk);

        self
    }

    /// Configures the pixel clock (PCLK) pin for the camera interface.
    pub fn with_pixel_clock(self, pclk: impl PeripheralInput<'d>) -> Self {
        let pclk = pclk.into();

        pclk.init_input(Pull::None);
        InputSignal::CAM_PCLK.connect_to(&pclk);

        self
    }

    /// Configures the Vertical Sync (VSYNC) pin for the camera interface.
    pub fn with_vsync(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_V_SYNC.connect_to(&pin);

        self
    }

    /// Configures the Horizontal Sync (HSYNC) pin for the camera interface.
    pub fn with_hsync(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_H_SYNC.connect_to(&pin);

        self
    }

    /// Configures the Horizontal Enable (HENABLE) pin for the camera interface.
    ///
    /// Also known as "Data Enable".
    pub fn with_h_enable(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_H_ENABLE.connect_to(&pin);

        self
    }

    /// Configures the DATA 0 pin for the camera interface.
    pub fn with_data0(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_0.connect_to(&pin);

        self
    }

    /// Configures the DATA 1 pin for the camera interface.
    pub fn with_data1(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_1.connect_to(&pin);

        self
    }

    /// Configures the DATA 2 pin for the camera interface.
    pub fn with_data2(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_2.connect_to(&pin);

        self
    }

    /// Configures the DATA 3 pin for the camera interface.
    pub fn with_data3(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_3.connect_to(&pin);

        self
    }

    /// Configures the DATA 4 pin for the camera interface.
    pub fn with_data4(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_4.connect_to(&pin);

        self
    }

    /// Configures the DATA 5 pin for the camera interface.
    pub fn with_data5(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_5.connect_to(&pin);

        self
    }

    /// Configures the DATA 6 pin for the camera interface.
    pub fn with_data6(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_6.connect_to(&pin);

        self
    }

    /// Configures the DATA 7 pin for the camera interface.
    pub fn with_data7(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_7.connect_to(&pin);

        self
    }

    /// Configures the DATA 8 pin for the camera interface.
    pub fn with_data8(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_8.connect_to(&pin);

        self
    }

    /// Configures the DATA 9 pin for the camera interface.
    pub fn with_data9(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_9.connect_to(&pin);

        self
    }

    /// Configures the DATA 10 pin for the camera interface.
    pub fn with_data10(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_10.connect_to(&pin);

        self
    }

    /// Configures the DATA 11 pin for the camera interface.
    pub fn with_data11(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_11.connect_to(&pin);

        self
    }

    /// Configures the DATA 12 pin for the camera interface.
    pub fn with_data12(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_12.connect_to(&pin);

        self
    }

    /// Configures the DATA 13 pin for the camera interface.
    pub fn with_data13(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_13.connect_to(&pin);

        self
    }

    /// Configures the DATA 14 pin for the camera interface.
    pub fn with_data14(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_14.connect_to(&pin);

        self
    }

    /// Configures the DATA 15 pin for the camera interface.
    pub fn with_data15(self, pin: impl PeripheralInput<'d>) -> Self {
        let pin = pin.into();

        pin.init_input(Pull::None);
        InputSignal::CAM_DATA_15.connect_to(&pin);

        self
    }

    /// Starts a DMA transfer to receive data from the camera peripheral.
    pub fn receive<BUF: DmaRxBuffer>(
        mut self,
        mut buf: BUF,
    ) -> Result<CameraTransfer<'d, BUF>, (DmaError, Self, BUF)> {
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
                .prepare_transfer(DmaPeripheral::LcdCam, &mut buf)
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
            .regs()
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
            .regs()
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().clear_bit());

        // Stop the DMA
        self.camera.rx_channel.stop_transfer();
    }
}

impl<BUF: DmaRxBuffer> Deref for CameraTransfer<'_, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<BUF: DmaRxBuffer> DerefMut for CameraTransfer<'_, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<BUF: DmaRxBuffer> Drop for CameraTransfer<'_, BUF> {
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

    /// Enable 16 bit mode (instead of 8 bit).
    enable_2byte_mode: bool,

    /// The byte order for the camera data.
    byte_order: ByteOrder,

    /// The bit order for the camera data.
    bit_order: BitOrder,

    /// If this is set to true, VSYNC + HSYNC mode is selected, in this mode,
    /// the signals of VSYNC, HSYNC and DE are used to control the data.
    /// For this case, users need to wire the three signal lines.
    ///
    /// If this is set to false, DE mode is selected, the signals of VSYNC and
    /// DE are used to control the data. For this case, wiring HSYNC signal
    /// line is not a must. But in this case, the YUV-RGB conversion
    /// function of camera module is not available.
    vh_de_mode_en: bool,

    /// The Vsync filter threshold.
    vsync_filter_threshold: Option<VsyncFilterThreshold>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: Rate::from_mhz(20),
            enable_2byte_mode: false,
            byte_order: Default::default(),
            bit_order: Default::default(),
            vh_de_mode_en: false,
            vsync_filter_threshold: None,
        }
    }
}
