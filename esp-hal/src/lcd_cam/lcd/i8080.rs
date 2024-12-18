//! # LCD - I8080/MOTO6800 Mode.
//!
//! ## Overview
//!
//! The LCD_CAM peripheral I8080 driver provides support for the I8080
//! format/timing. The driver mandates DMA (Direct Memory Access) for
//! efficient data transfer.
//!
//! ## Examples
//!
//! ### MIPI-DSI Display
//!
//! The following example shows how to send a command to a MIPI-DSI display over
//! the I8080 protocol.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::lcd_cam::{LcdCam, lcd::i8080::{Config, I8080, TxEightBits}};
//! # use esp_hal::dma_tx_buffer;
//! # use esp_hal::dma::DmaTxBuf;
//!
//! # let mut dma_buf = dma_tx_buffer!(32678).unwrap();
//!
//! let tx_pins = TxEightBits::new(
//!     peripherals.GPIO9,
//!     peripherals.GPIO46,
//!     peripherals.GPIO3,
//!     peripherals.GPIO8,
//!     peripherals.GPIO18,
//!     peripherals.GPIO17,
//!     peripherals.GPIO16,
//!     peripherals.GPIO15,
//! );
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//!
//! let mut config = Config::default();
//! config.frequency = 20.MHz();
//!
//! let mut i8080 = I8080::new(
//!     lcd_cam.lcd,
//!     peripherals.DMA_CH0,
//!     tx_pins,
//!     config,
//! )
//! .unwrap()
//! .with_ctrl_pins(peripherals.GPIO0, peripherals.GPIO47);
//!
//! dma_buf.fill(&[0x55]);
//! let transfer = i8080.send(0x3Au8, 0, dma_buf).unwrap(); // RGB565
//! transfer.wait();
//! # }
//! ```

use core::{
    fmt::Formatter,
    marker::PhantomData,
    mem::{size_of, ManuallyDrop},
    ops::{Deref, DerefMut},
};

use fugit::{HertzU32, RateExtU32};

use crate::{
    clock::Clocks,
    dma::{ChannelTx, DmaError, DmaPeripheral, DmaTxBuffer, PeripheralTxChannel, Tx, TxChannelFor},
    gpio::{
        interconnect::{OutputConnection, PeripheralOutput},
        OutputSignal,
    },
    lcd_cam::{
        calculate_clkm,
        lcd::{ClockMode, DelayMode, Phase, Polarity},
        BitOrder,
        ByteOrder,
        ClockError,
        Instance,
        Lcd,
        LCD_DONE_WAKER,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
    Blocking,
    Mode,
};

/// A configuration error.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {
    /// Clock configuration error.
    Clock(ClockError),
}

/// Represents the I8080 LCD interface.
pub struct I8080<'d, Dm: DriverMode> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    tx_channel: ChannelTx<'d, Blocking, PeripheralTxChannel<LCD_CAM>>,
    _mode: PhantomData<Dm>,
}

impl<'d, Dm> I8080<'d, Dm>
where
    Dm: DriverMode,
{
    /// Creates a new instance of the I8080 LCD interface.
    pub fn new<P, CH>(
        lcd: Lcd<'d, Dm>,
        channel: impl Peripheral<P = CH> + 'd,
        mut pins: P,
        config: Config,
    ) -> Result<Self, ConfigError>
    where
        CH: TxChannelFor<LCD_CAM>,
        P: TxPins,
    {
        let tx_channel = ChannelTx::new(channel.map(|ch| ch.degrade()));

        let mut this = Self {
            lcd_cam: lcd.lcd_cam,
            tx_channel,
            _mode: PhantomData,
        };

        this.apply_config(&config)?;
        pins.configure();

        Ok(this)
    }

    /// Applies configuration.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        let clocks = Clocks::get();
        // Due to https://www.espressif.com/sites/default/files/documentation/esp32-s3_errata_en.pdf
        // the LCD_PCLK divider must be at least 2. To make up for this the user
        // provided frequency is doubled to match.
        let (i, divider) = calculate_clkm(
            (config.frequency.to_Hz() * 2) as _,
            &[
                clocks.xtal_clock.to_Hz() as _,
                clocks.cpu_clock.to_Hz() as _,
                clocks.crypto_pwm_clock.to_Hz() as _,
            ],
        )
        .map_err(ConfigError::Clock)?;

        self.lcd_cam.lcd_clock().write(|w| unsafe {
            // Force enable the clock for all configuration registers.
            w.clk_en().set_bit();
            w.lcd_clk_sel().bits((i + 1) as _);
            w.lcd_clkm_div_num().bits(divider.div_num as _);
            w.lcd_clkm_div_b().bits(divider.div_b as _);
            w.lcd_clkm_div_a().bits(divider.div_a as _); // LCD_PCLK = LCD_CLK / 2
            w.lcd_clk_equ_sysclk().clear_bit();
            w.lcd_clkcnt_n().bits(2 - 1); // Must not be 0.
            w.lcd_ck_idle_edge()
                .bit(config.clock_mode.polarity == Polarity::IdleHigh);
            w.lcd_ck_out_edge()
                .bit(config.clock_mode.phase == Phase::ShiftHigh)
        });

        self.lcd_cam
            .lcd_ctrl()
            .write(|w| w.lcd_rgb_mode_en().clear_bit());
        self.lcd_cam
            .lcd_rgb_yuv()
            .write(|w| w.lcd_conv_bypass().clear_bit());

        self.lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_8bits_order().bit(false);
            w.lcd_bit_order().bit(false);
            w.lcd_byte_order().bit(false);
            w.lcd_2byte_en().bit(false)
        });
        self.lcd_cam.lcd_misc().write(|w| unsafe {
            // Set the threshold for Async Tx FIFO full event. (5 bits)
            w.lcd_afifo_threshold_num().bits(0);
            // Configure the setup cycles in LCD non-RGB mode. Setup cycles
            // expected = this value + 1. (6 bit)
            w.lcd_vfk_cyclelen()
                .bits(config.setup_cycles.saturating_sub(1) as _);
            // Configure the hold time cycles in LCD non-RGB mode. Hold
            // cycles expected = this value + 1.
            w.lcd_vbk_cyclelen()
                .bits(config.hold_cycles.saturating_sub(1) as _);
            // 1: Send the next frame data when the current frame is sent out.
            // 0: LCD stops when the current frame is sent out.
            w.lcd_next_frame_en().clear_bit();
            // Enable blank region when LCD sends data out.
            w.lcd_bk_en().set_bit();
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DOUT phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_data_set()
                .bit(config.cd_data_edge != config.cd_idle_edge);
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DUMMY phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_dummy_set()
                .bit(config.cd_dummy_edge != config.cd_idle_edge);
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in CMD phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_cmd_set()
                .bit(config.cd_cmd_edge != config.cd_idle_edge);
            // The default value of LCD_CD
            w.lcd_cd_idle_edge().bit(config.cd_idle_edge)
        });
        self.lcd_cam
            .lcd_dly_mode()
            .write(|w| unsafe { w.lcd_cd_mode().bits(config.cd_mode as u8) });
        self.lcd_cam.lcd_data_dout_mode().write(|w| unsafe {
            w.dout0_mode().bits(config.output_bit_mode as u8);
            w.dout1_mode().bits(config.output_bit_mode as u8);
            w.dout2_mode().bits(config.output_bit_mode as u8);
            w.dout3_mode().bits(config.output_bit_mode as u8);
            w.dout4_mode().bits(config.output_bit_mode as u8);
            w.dout5_mode().bits(config.output_bit_mode as u8);
            w.dout6_mode().bits(config.output_bit_mode as u8);
            w.dout7_mode().bits(config.output_bit_mode as u8);
            w.dout8_mode().bits(config.output_bit_mode as u8);
            w.dout9_mode().bits(config.output_bit_mode as u8);
            w.dout10_mode().bits(config.output_bit_mode as u8);
            w.dout11_mode().bits(config.output_bit_mode as u8);
            w.dout12_mode().bits(config.output_bit_mode as u8);
            w.dout13_mode().bits(config.output_bit_mode as u8);
            w.dout14_mode().bits(config.output_bit_mode as u8);
            w.dout15_mode().bits(config.output_bit_mode as u8)
        });

        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit());

        Ok(())
    }

    /// Configures the byte order for data transmission in 16-bit mode.
    /// This must be set to [ByteOrder::default()] when transmitting in 8-bit
    /// mode.
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        let is_inverted = byte_order != ByteOrder::default();
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_byte_order().bit(is_inverted));
        self
    }

    /// Configures the byte order for data transmission in 8-bit mode.
    /// This must be set to [ByteOrder::default()] when transmitting in 16-bit
    /// mode.
    pub fn set_8bits_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        let is_inverted = byte_order != ByteOrder::default();
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_8bits_order().bit(is_inverted));
        self
    }

    /// Configures the bit order for data transmission.
    pub fn set_bit_order(&mut self, bit_order: BitOrder) -> &mut Self {
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_bit_order().bit(bit_order != BitOrder::default()));
        self
    }

    /// Associates a CS pin with the I8080 interface.
    pub fn with_cs<CS: PeripheralOutput>(self, cs: impl Peripheral<P = CS> + 'd) -> Self {
        crate::into_mapped_ref!(cs);
        cs.set_to_push_pull_output(crate::private::Internal);
        OutputSignal::LCD_CS.connect_to(cs);

        self
    }

    /// Configures the control pins for the I8080 interface.
    pub fn with_ctrl_pins<DC: PeripheralOutput, WRX: PeripheralOutput>(
        self,
        dc: impl Peripheral<P = DC> + 'd,
        wrx: impl Peripheral<P = WRX> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(dc, wrx);

        dc.set_to_push_pull_output(crate::private::Internal);
        OutputSignal::LCD_DC.connect_to(dc);

        wrx.set_to_push_pull_output(crate::private::Internal);
        OutputSignal::LCD_PCLK.connect_to(wrx);

        self
    }

    /// Sends a command and data to the LCD using DMA.
    ///
    /// Passing a `Command<u8>` will make this an 8-bit transfer and a
    /// `Command<u16>` will make this a 16-bit transfer.
    ///
    /// Note: A 16-bit transfer on an 8-bit bus will silently truncate the 2nd
    /// byte and an 8-bit transfer on a 16-bit bus will silently pad each
    /// byte to 2 bytes.
    pub fn send<W: Into<u16> + Copy, BUF: DmaTxBuffer>(
        mut self,
        cmd: impl Into<Command<W>>,
        dummy: u8,
        mut data: BUF,
    ) -> Result<I8080Transfer<'d, BUF, Dm>, (DmaError, Self, BUF)> {
        let cmd = cmd.into();

        // Reset LCD control unit and Async Tx FIFO
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.lcd_cam
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        // Set cmd value
        match cmd {
            Command::None => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().clear_bit());
            }
            Command::One(value) => {
                self.lcd_cam.lcd_user().modify(|_, w| {
                    w.lcd_cmd().set_bit();
                    w.lcd_cmd_2_cycle_en().clear_bit()
                });
                self.lcd_cam
                    .lcd_cmd_val()
                    .write(|w| unsafe { w.lcd_cmd_value().bits(value.into() as _) });
            }
            Command::Two(first, second) => {
                self.lcd_cam.lcd_user().modify(|_, w| {
                    w.lcd_cmd().set_bit();
                    w.lcd_cmd_2_cycle_en().set_bit()
                });
                let cmd = first.into() as u32 | (second.into() as u32) << 16;
                self.lcd_cam
                    .lcd_cmd_val()
                    .write(|w| unsafe { w.lcd_cmd_value().bits(cmd) });
            }
        }

        let is_2byte_mode = size_of::<W>() == 2;

        self.lcd_cam.lcd_user().modify(|_, w| unsafe {
            // Set dummy length
            if dummy > 0 {
                // Enable DUMMY phase in LCD sequence when LCD starts.
                w.lcd_dummy()
                    .set_bit()
                    // Configure DUMMY cycles. DUMMY cycles = this value + 1. (2 bits)
                    .lcd_dummy_cyclelen()
                    .bits((dummy - 1) as _)
            } else {
                w.lcd_dummy().clear_bit()
            }
            .lcd_2byte_en()
            .bit(is_2byte_mode)
        });

        // Use continous mode for DMA. FROM the S3 TRM:
        // > In a continuous output, LCD module keeps sending data till:
        // > i. LCD_CAM_LCD_START is cleared;
        // > ii. or LCD_CAM_LCD_RESET is set;
        // > iii. or all the data in GDMA is sent out.
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_always_out_en().set_bit().lcd_dout().set_bit());

        let result = unsafe {
            self.tx_channel
                .prepare_transfer(DmaPeripheral::LcdCam, &mut data)
        }
        .and_then(|_| self.tx_channel.start_transfer());
        if let Err(err) = result {
            return Err((err, self, data));
        }

        // Setup interrupts.
        self.lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());

        // Before issuing lcd_start need to wait shortly for fifo to get data
        // Otherwise, some garbage data will be sent out
        crate::rom::ets_delay_us(1);

        self.lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_update().set_bit();
            w.lcd_start().set_bit()
        });

        Ok(I8080Transfer {
            i8080: ManuallyDrop::new(self),
            buf_view: ManuallyDrop::new(data.into_view()),
        })
    }
}

impl<Dm: DriverMode> core::fmt::Debug for I8080<'_, Dm> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I8080").finish()
    }
}

/// Represents an ongoing (or potentially finished) transfer using the I8080 LCD
/// interface
pub struct I8080Transfer<'d, BUF: DmaTxBuffer, Dm: DriverMode> {
    i8080: ManuallyDrop<I8080<'d, Dm>>,
    buf_view: ManuallyDrop<BUF::View>,
}

impl<'d, BUF: DmaTxBuffer, Dm: DriverMode> I8080Transfer<'d, BUF, Dm> {
    /// Returns true when [Self::wait] will not block.
    pub fn is_done(&self) -> bool {
        self.i8080
            .lcd_cam
            .lcd_user()
            .read()
            .lcd_start()
            .bit_is_clear()
    }

    /// Stops this transfer on the spot and returns the peripheral and buffer.
    pub fn cancel(mut self) -> (I8080<'d, Dm>, BUF) {
        self.stop_peripherals();
        let (_, i8080, buf) = self.wait();
        (i8080, buf)
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    ///
    /// Note: This also clears the transfer interrupt so it can be used in
    /// interrupt handlers to "handle" the interrupt.
    pub fn wait(mut self) -> (Result<(), DmaError>, I8080<'d, Dm>, BUF) {
        while !self.is_done() {}

        // Clear "done" interrupt.
        self.i8080
            .lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());

        // SAFETY: Since forget is called on self, we know that self.i8080 and
        // self.buf_view won't be touched again.
        let (i8080, view) = unsafe {
            let i8080 = ManuallyDrop::take(&mut self.i8080);
            let view = ManuallyDrop::take(&mut self.buf_view);
            core::mem::forget(self);
            (i8080, view)
        };

        let result = if i8080.tx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        };

        (result, i8080, BUF::from_view(view))
    }

    fn stop_peripherals(&mut self) {
        // Stop the LCD_CAM peripheral.
        self.i8080
            .lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());

        // Stop the DMA
        self.i8080.tx_channel.stop_transfer();
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> Deref for I8080Transfer<'_, BUF, Dm> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buf_view
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> DerefMut for I8080Transfer<'_, BUF, Dm> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buf_view
    }
}

impl<'d, BUF: DmaTxBuffer> I8080Transfer<'d, BUF, crate::Async> {
    /// Waits for [Self::is_done] to return true.
    pub async fn wait_for_done(&mut self) {
        use core::{
            future::Future,
            pin::Pin,
            task::{Context, Poll},
        };

        struct LcdDoneFuture {}

        impl Future for LcdDoneFuture {
            type Output = ();

            fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
                LCD_DONE_WAKER.register(cx.waker());
                if Instance::is_lcd_done_set() {
                    // Interrupt bit will be cleared in Self::wait.
                    // This allows `wait_for_done` to be called more than once.
                    //
                    // Instance::clear_lcd_done();
                    Poll::Ready(())
                } else {
                    Instance::listen_lcd_done();
                    Poll::Pending
                }
            }
        }

        impl Drop for LcdDoneFuture {
            fn drop(&mut self) {
                Instance::unlisten_lcd_done();
            }
        }

        LcdDoneFuture {}.await
    }
}

impl<BUF: DmaTxBuffer, Dm: DriverMode> Drop for I8080Transfer<'_, BUF, Dm> {
    fn drop(&mut self) {
        self.stop_peripherals();

        // SAFETY: This is Drop, we know that self.i8080 and self.buf_view
        // won't be touched again.
        let view = unsafe {
            ManuallyDrop::drop(&mut self.i8080);
            ManuallyDrop::take(&mut self.buf_view)
        };
        let _ = BUF::from_view(view);
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Configuration settings for the I8080 interface.
pub struct Config {
    /// Specifies the clock mode, including polarity and phase settings.
    pub clock_mode: ClockMode,

    /// The frequency of the pixel clock.
    pub frequency: HertzU32,

    /// Setup cycles expected, must be at least 1. (6 bits)
    pub setup_cycles: usize,

    /// Hold cycles expected, must be at least 1. (13 bits)
    pub hold_cycles: usize,

    /// The default value of LCD_CD.
    pub cd_idle_edge: bool,
    /// The value of LCD_CD during CMD phase.
    pub cd_cmd_edge: bool,
    /// The value of LCD_CD during dummy phase.
    pub cd_dummy_edge: bool,
    /// The value of LCD_CD during data phase.
    pub cd_data_edge: bool,

    /// The output LCD_CD is delayed by module clock LCD_CLK.
    pub cd_mode: DelayMode,
    /// The output data bits are delayed by module clock LCD_CLK.
    pub output_bit_mode: DelayMode,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_mode: Default::default(),
            frequency: 20.MHz(),
            setup_cycles: 1,
            hold_cycles: 1,
            cd_idle_edge: false,
            cd_cmd_edge: false,
            cd_dummy_edge: false,
            cd_data_edge: true,
            cd_mode: Default::default(),
            output_bit_mode: Default::default(),
        }
    }
}

/// LCD_CAM I8080 command.
///
/// Can be [Command::None] if command phase should be suppressed.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command<T> {
    /// Suppresses the command phase. No command is sent.
    None,
    /// Sends a single-word command.
    One(T),
    /// Sends a two-word command.
    Two(T, T),
}

impl From<u8> for Command<u8> {
    fn from(value: u8) -> Self {
        Command::One(value)
    }
}

impl From<u16> for Command<u16> {
    fn from(value: u16) -> Self {
        Command::One(value)
    }
}

/// Represents a group of 8 output pins configured for 8-bit parallel data
/// transmission.
pub struct TxEightBits<'d> {
    pins: [PeripheralRef<'d, OutputConnection>; 8],
}

impl<'d> TxEightBits<'d> {
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxEightBits` instance with the provided output pins.
    pub fn new(
        pin_0: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_1: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_2: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_3: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_4: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_5: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_6: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_7: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(pin_0);
        crate::into_mapped_ref!(pin_1);
        crate::into_mapped_ref!(pin_2);
        crate::into_mapped_ref!(pin_3);
        crate::into_mapped_ref!(pin_4);
        crate::into_mapped_ref!(pin_5);
        crate::into_mapped_ref!(pin_6);
        crate::into_mapped_ref!(pin_7);

        Self {
            pins: [pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7],
        }
    }
}

impl TxPins for TxEightBits<'_> {
    fn configure(&mut self) {
        const SIGNALS: [OutputSignal; 8] = [
            OutputSignal::LCD_DATA_0,
            OutputSignal::LCD_DATA_1,
            OutputSignal::LCD_DATA_2,
            OutputSignal::LCD_DATA_3,
            OutputSignal::LCD_DATA_4,
            OutputSignal::LCD_DATA_5,
            OutputSignal::LCD_DATA_6,
            OutputSignal::LCD_DATA_7,
        ];

        for (pin, signal) in self.pins.iter_mut().zip(SIGNALS.into_iter()) {
            pin.set_to_push_pull_output(crate::private::Internal);
            signal.connect_to(pin);
        }
    }
}

/// Represents a group of 16 output pins configured for 16-bit parallel data
/// transmission.
pub struct TxSixteenBits<'d> {
    pins: [PeripheralRef<'d, OutputConnection>; 16],
}

impl<'d> TxSixteenBits<'d> {
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxSixteenBits` instance with the provided output pins.
    pub fn new(
        pin_0: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_1: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_2: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_3: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_4: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_5: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_6: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_7: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_8: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_9: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_10: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_11: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_12: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_13: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_14: impl Peripheral<P = impl PeripheralOutput> + 'd,
        pin_15: impl Peripheral<P = impl PeripheralOutput> + 'd,
    ) -> Self {
        crate::into_mapped_ref!(
            pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7, pin_8, pin_9, pin_10, pin_11,
            pin_12, pin_13, pin_14, pin_15
        );

        Self {
            pins: [
                pin_0, pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7, pin_8, pin_9, pin_10,
                pin_11, pin_12, pin_13, pin_14, pin_15,
            ],
        }
    }
}

impl TxPins for TxSixteenBits<'_> {
    fn configure(&mut self) {
        const SIGNALS: [OutputSignal; 16] = [
            OutputSignal::LCD_DATA_0,
            OutputSignal::LCD_DATA_1,
            OutputSignal::LCD_DATA_2,
            OutputSignal::LCD_DATA_3,
            OutputSignal::LCD_DATA_4,
            OutputSignal::LCD_DATA_5,
            OutputSignal::LCD_DATA_6,
            OutputSignal::LCD_DATA_7,
            OutputSignal::LCD_DATA_8,
            OutputSignal::LCD_DATA_9,
            OutputSignal::LCD_DATA_10,
            OutputSignal::LCD_DATA_11,
            OutputSignal::LCD_DATA_12,
            OutputSignal::LCD_DATA_13,
            OutputSignal::LCD_DATA_14,
            OutputSignal::LCD_DATA_15,
        ];

        for (pin, signal) in self.pins.iter_mut().zip(SIGNALS.into_iter()) {
            pin.set_to_push_pull_output(crate::private::Internal);
            signal.connect_to(pin);
        }
    }
}

#[doc(hidden)]
pub trait TxPins {
    fn configure(&mut self);
}
