//! # LCD - I8080/MOTO6800 Mode.
//!
//! ## Overview
//!
//! The LCD_CAM peripheral I8080 driver provides support for the I8080
//! format/timing. The driver mandates DMA (Direct Memory Access) for
//! efficient data transfer.
//!
//! ## Example
//!
//! ### MIPI-DSI Display
//!
//! The following example shows how to send a command to a MIPI-DSI display over
//! the I8080 protocol.
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::gpio::Io;
//! # use esp_hal::lcd_cam::{LcdCam, lcd::i8080::{Config, I8080, TxEightBits}};
//! # use esp_hal::dma_buffers;
//! # use esp_hal::dma::{Dma, DmaPriority, DmaTxBuf};
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//!
//! # let dma = Dma::new(peripherals.DMA);
//! # let channel = dma.channel0;
//!
//! # let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, 32678);
//! # let mut dma_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
//!
//! # let channel = channel.configure(
//! #     false,
//! #     DmaPriority::Priority0,
//! # );
//!
//! let tx_pins = TxEightBits::new(
//!     io.pins.gpio9,
//!     io.pins.gpio46,
//!     io.pins.gpio3,
//!     io.pins.gpio8,
//!     io.pins.gpio18,
//!     io.pins.gpio17,
//!     io.pins.gpio16,
//!     io.pins.gpio15,
//! );
//! let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
//!
//! let mut i8080 = I8080::new(
//!     lcd_cam.lcd,
//!     channel.tx,
//!     tx_pins,
//!     20.MHz(),
//!     Config::default(),
//! )
//! .with_ctrl_pins(io.pins.gpio0, io.pins.gpio47);
//!
//! dma_buf.fill(&[0x55]);
//! let transfer = i8080.send(0x3Au8, 0, dma_buf).unwrap(); // RGB565
//! transfer.wait();
//! # }
//! ```

use core::{fmt::Formatter, marker::PhantomData, mem::size_of};

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{
        ChannelTx,
        DmaChannel,
        DmaError,
        DmaPeripheral,
        DmaTxBuffer,
        LcdCamPeripheral,
        TxPrivate,
    },
    gpio::{OutputSignal, PeripheralOutput},
    lcd_cam::{
        asynch::LCD_DONE_WAKER,
        lcd::{i8080::private::TxPins, ClockMode, DelayMode, Phase, Polarity},
        private::{calculate_clkm, Instance},
        BitOrder,
        ByteOrder,
        Lcd,
    },
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
    Mode,
};

/// Represents the I8080 LCD interface.
pub struct I8080<'d, CH: DmaChannel, DM: Mode> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    tx_channel: ChannelTx<'d, CH>,
    _phantom: PhantomData<DM>,
}

impl<'d, CH: DmaChannel, DM: Mode> I8080<'d, CH, DM>
where
    CH::P: LcdCamPeripheral,
{
    /// Creates a new instance of the I8080 LCD interface.
    pub fn new<P: TxPins>(
        lcd: Lcd<'d, DM>,
        mut channel: ChannelTx<'d, CH>,
        mut pins: P,
        frequency: HertzU32,
        config: Config,
    ) -> Self {
        let lcd_cam = lcd.lcd_cam;

        let clocks = Clocks::get();
        // Due to https://www.espressif.com/sites/default/files/documentation/esp32-s3_errata_en.pdf
        // the LCD_PCLK divider must be at least 2. To make up for this the user
        // provided frequency is doubled to match.
        let (i, divider) = calculate_clkm(
            (frequency.to_Hz() * 2) as _,
            &[
                clocks.xtal_clock.to_Hz() as _,
                clocks.cpu_clock.to_Hz() as _,
                clocks.crypto_pwm_clock.to_Hz() as _,
            ],
        );

        lcd_cam.lcd_clock().write(|w| unsafe {
            // Force enable the clock for all configuration registers.
            w.clk_en()
                .set_bit()
                .lcd_clk_sel()
                .bits((i + 1) as _)
                .lcd_clkm_div_num()
                .bits(divider.div_num as _)
                .lcd_clkm_div_b()
                .bits(divider.div_b as _)
                .lcd_clkm_div_a()
                .bits(divider.div_a as _)
                // LCD_PCLK = LCD_CLK / 2
                .lcd_clk_equ_sysclk()
                .clear_bit()
                .lcd_clkcnt_n()
                .bits(2 - 1) // Must not be 0.
                .lcd_ck_idle_edge()
                .bit(config.clock_mode.polarity == Polarity::IdleHigh)
                .lcd_ck_out_edge()
                .bit(config.clock_mode.phase == Phase::ShiftHigh)
        });

        lcd_cam
            .lcd_ctrl()
            .write(|w| w.lcd_rgb_mode_en().clear_bit());
        lcd_cam
            .lcd_rgb_yuv()
            .write(|w| w.lcd_conv_bypass().clear_bit());

        lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_8bits_order()
                .bit(false)
                .lcd_bit_order()
                .bit(false)
                .lcd_byte_order()
                .bit(false)
                .lcd_2byte_en()
                .bit(false)
        });
        lcd_cam.lcd_misc().write(|w| unsafe {
            // Set the threshold for Async Tx FIFO full event. (5 bits)
            w.lcd_afifo_threshold_num()
                .bits(0)
                // Configure the setup cycles in LCD non-RGB mode. Setup cycles
                // expected = this value + 1. (6 bit)
                .lcd_vfk_cyclelen()
                .bits(config.setup_cycles.saturating_sub(1) as _)
                // Configure the hold time cycles in LCD non-RGB mode. Hold
                // cycles expected = this value + 1.
                .lcd_vbk_cyclelen()
                .bits(config.hold_cycles.saturating_sub(1) as _)
                // 1: Send the next frame data when the current frame is sent out.
                // 0: LCD stops when the current frame is sent out.
                .lcd_next_frame_en()
                .clear_bit()
                // Enable blank region when LCD sends data out.
                .lcd_bk_en()
                .set_bit()
                // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DOUT phase.
                // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
                .lcd_cd_data_set()
                .bit(config.cd_data_edge != config.cd_idle_edge)
                // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DUMMY phase.
                // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
                .lcd_cd_dummy_set()
                .bit(config.cd_dummy_edge != config.cd_idle_edge)
                // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in CMD phase.
                // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
                .lcd_cd_cmd_set()
                .bit(config.cd_cmd_edge != config.cd_idle_edge)
                // The default value of LCD_CD
                .lcd_cd_idle_edge()
                .bit(config.cd_idle_edge)
        });
        lcd_cam
            .lcd_dly_mode()
            .write(|w| unsafe { w.lcd_cd_mode().bits(config.cd_mode as u8) });
        lcd_cam.lcd_data_dout_mode().write(|w| unsafe {
            w.dout0_mode()
                .bits(config.output_bit_mode as u8)
                .dout1_mode()
                .bits(config.output_bit_mode as u8)
                .dout2_mode()
                .bits(config.output_bit_mode as u8)
                .dout3_mode()
                .bits(config.output_bit_mode as u8)
                .dout4_mode()
                .bits(config.output_bit_mode as u8)
                .dout5_mode()
                .bits(config.output_bit_mode as u8)
                .dout6_mode()
                .bits(config.output_bit_mode as u8)
                .dout7_mode()
                .bits(config.output_bit_mode as u8)
                .dout8_mode()
                .bits(config.output_bit_mode as u8)
                .dout9_mode()
                .bits(config.output_bit_mode as u8)
                .dout10_mode()
                .bits(config.output_bit_mode as u8)
                .dout11_mode()
                .bits(config.output_bit_mode as u8)
                .dout12_mode()
                .bits(config.output_bit_mode as u8)
                .dout13_mode()
                .bits(config.output_bit_mode as u8)
                .dout14_mode()
                .bits(config.output_bit_mode as u8)
                .dout15_mode()
                .bits(config.output_bit_mode as u8)
        });

        lcd_cam.lcd_user().modify(|_, w| w.lcd_update().set_bit());

        channel.init_channel();
        pins.configure();

        Self {
            lcd_cam,
            tx_channel: channel,
            _phantom: PhantomData,
        }
    }
}

impl<'d, CH: DmaChannel, DM: Mode> I8080<'d, CH, DM> {
    /// Configures the byte order for data transmission.
    pub fn set_byte_order(&mut self, byte_order: ByteOrder) -> &mut Self {
        let is_inverted = byte_order != ByteOrder::default();
        self.lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_byte_order()
                .bit(is_inverted)
                .lcd_8bits_order()
                .bit(is_inverted)
        });
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
        crate::into_ref!(cs);
        cs.set_to_push_pull_output(crate::private::Internal);
        cs.connect_peripheral_to_output(OutputSignal::LCD_CS, crate::private::Internal);

        self
    }

    /// Configures the control pins for the I8080 interface.
    pub fn with_ctrl_pins<DC: PeripheralOutput, WRX: PeripheralOutput>(
        self,
        dc: impl Peripheral<P = DC> + 'd,
        wrx: impl Peripheral<P = WRX> + 'd,
    ) -> Self {
        crate::into_ref!(dc);
        crate::into_ref!(wrx);

        dc.set_to_push_pull_output(crate::private::Internal);
        dc.connect_peripheral_to_output(OutputSignal::LCD_DC, crate::private::Internal);

        wrx.set_to_push_pull_output(crate::private::Internal);
        wrx.connect_peripheral_to_output(OutputSignal::LCD_PCLK, crate::private::Internal);

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
    ) -> Result<I8080Transfer<'d, CH, DM, BUF>, (DmaError, Self, BUF)> {
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
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().set_bit().lcd_cmd_2_cycle_en().clear_bit());
                self.lcd_cam
                    .lcd_cmd_val()
                    .write(|w| unsafe { w.lcd_cmd_value().bits(value.into() as _) });
            }
            Command::Two(first, second) => {
                self.lcd_cam
                    .lcd_user()
                    .modify(|_, w| w.lcd_cmd().set_bit().lcd_cmd_2_cycle_en().set_bit());
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

        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit().lcd_start().set_bit());

        Ok(I8080Transfer {
            i8080: self,
            tx_buf: data,
        })
    }
}

impl<'d, CH: DmaChannel, DM: Mode> core::fmt::Debug for I8080<'d, CH, DM> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("I8080").finish()
    }
}

/// Represents an ongoing (or potentially finished) transfer using the I8080 LCD
/// interface
pub struct I8080Transfer<'d, CH: DmaChannel, DM: Mode, BUF> {
    i8080: I8080<'d, CH, DM>,
    tx_buf: BUF,
}

impl<'d, CH: DmaChannel, DM: Mode, BUF> I8080Transfer<'d, CH, DM, BUF> {
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
    pub fn cancel(mut self) -> (I8080<'d, CH, DM>, BUF) {
        self.stop_peripherals();
        let (_, i8080, buf) = self.wait();
        (i8080, buf)
    }

    /// Waits for the transfer to finish and returns the peripheral and buffer.
    ///
    /// Note: This also clears the transfer interrupt so it can be used in
    /// interrupt handlers to "handle" the interrupt.
    pub fn wait(self) -> (Result<(), DmaError>, I8080<'d, CH, DM>, BUF) {
        while !self.is_done() {}

        // Clear "done" interrupt.
        self.i8080
            .lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());

        unsafe {
            let i8080 = core::ptr::read(&self.i8080);
            let tx_buf = core::ptr::read(&self.tx_buf);
            core::mem::forget(self);

            let result = if i8080.tx_channel.has_error() {
                Err(DmaError::DescriptorError)
            } else {
                Ok(())
            };

            (result, i8080, tx_buf)
        }
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

impl<'d, CH: DmaChannel, BUF> I8080Transfer<'d, CH, crate::Async, BUF> {
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

impl<'d, CH: DmaChannel, DM: Mode, BUF> Drop for I8080Transfer<'d, CH, DM, BUF> {
    fn drop(&mut self) {
        self.stop_peripherals();
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Configuration settings for the I8080 interface.
pub struct Config {
    /// Specifies the clock mode, including polarity and phase settings.
    pub clock_mode: ClockMode,

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
pub struct TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7> {
    pin_0: PeripheralRef<'d, P0>,
    pin_1: PeripheralRef<'d, P1>,
    pin_2: PeripheralRef<'d, P2>,
    pin_3: PeripheralRef<'d, P3>,
    pin_4: PeripheralRef<'d, P4>,
    pin_5: PeripheralRef<'d, P5>,
    pin_6: PeripheralRef<'d, P6>,
    pin_7: PeripheralRef<'d, P7>,
}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
{
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxEightBits` instance with the provided output pins.
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

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> TxPins for TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
{
    fn configure(&mut self) {
        self.pin_0.set_to_push_pull_output(crate::private::Internal);
        self.pin_0
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_0, crate::private::Internal);
        self.pin_1.set_to_push_pull_output(crate::private::Internal);
        self.pin_1
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_1, crate::private::Internal);
        self.pin_2.set_to_push_pull_output(crate::private::Internal);
        self.pin_2
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_2, crate::private::Internal);
        self.pin_3.set_to_push_pull_output(crate::private::Internal);
        self.pin_3
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_3, crate::private::Internal);
        self.pin_4.set_to_push_pull_output(crate::private::Internal);
        self.pin_4
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_4, crate::private::Internal);
        self.pin_5.set_to_push_pull_output(crate::private::Internal);
        self.pin_5
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_5, crate::private::Internal);
        self.pin_6.set_to_push_pull_output(crate::private::Internal);
        self.pin_6
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_6, crate::private::Internal);
        self.pin_7.set_to_push_pull_output(crate::private::Internal);
        self.pin_7
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_7, crate::private::Internal);
    }
}

/// Represents a group of 16 output pins configured for 16-bit parallel data
/// transmission.
pub struct TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> {
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
    TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
    P8: PeripheralOutput,
    P9: PeripheralOutput,
    P10: PeripheralOutput,
    P11: PeripheralOutput,
    P12: PeripheralOutput,
    P13: PeripheralOutput,
    P14: PeripheralOutput,
    P15: PeripheralOutput,
{
    #[allow(clippy::too_many_arguments)]
    /// Creates a new `TxSixteenBits` instance with the provided output pins.
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

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15> TxPins
    for TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
where
    P0: PeripheralOutput,
    P1: PeripheralOutput,
    P2: PeripheralOutput,
    P3: PeripheralOutput,
    P4: PeripheralOutput,
    P5: PeripheralOutput,
    P6: PeripheralOutput,
    P7: PeripheralOutput,
    P8: PeripheralOutput,
    P9: PeripheralOutput,
    P10: PeripheralOutput,
    P11: PeripheralOutput,
    P12: PeripheralOutput,
    P13: PeripheralOutput,
    P14: PeripheralOutput,
    P15: PeripheralOutput,
{
    fn configure(&mut self) {
        self.pin_0.set_to_push_pull_output(crate::private::Internal);
        self.pin_0
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_0, crate::private::Internal);
        self.pin_1.set_to_push_pull_output(crate::private::Internal);
        self.pin_1
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_1, crate::private::Internal);
        self.pin_2.set_to_push_pull_output(crate::private::Internal);
        self.pin_2
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_2, crate::private::Internal);
        self.pin_3.set_to_push_pull_output(crate::private::Internal);
        self.pin_3
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_3, crate::private::Internal);
        self.pin_4.set_to_push_pull_output(crate::private::Internal);
        self.pin_4
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_4, crate::private::Internal);
        self.pin_5.set_to_push_pull_output(crate::private::Internal);
        self.pin_5
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_5, crate::private::Internal);
        self.pin_6.set_to_push_pull_output(crate::private::Internal);
        self.pin_6
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_6, crate::private::Internal);
        self.pin_7.set_to_push_pull_output(crate::private::Internal);
        self.pin_7
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_7, crate::private::Internal);
        self.pin_8.set_to_push_pull_output(crate::private::Internal);
        self.pin_8
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_8, crate::private::Internal);
        self.pin_9.set_to_push_pull_output(crate::private::Internal);
        self.pin_9
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_9, crate::private::Internal);
        self.pin_10
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_10
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_10, crate::private::Internal);
        self.pin_11
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_11
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_11, crate::private::Internal);
        self.pin_12
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_12
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_12, crate::private::Internal);
        self.pin_13
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_13
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_13, crate::private::Internal);
        self.pin_14
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_14
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_14, crate::private::Internal);
        self.pin_15
            .set_to_push_pull_output(crate::private::Internal);
        self.pin_15
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_15, crate::private::Internal);
    }
}

mod private {
    pub trait TxPins {
        fn configure(&mut self);
    }
}
