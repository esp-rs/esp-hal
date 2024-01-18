use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{DmaError, DmaPeripheral, Tx},
    gpio::{OutputPin, OutputSignal},
    lcd_cam::{private::calculate_clkm, Lcd},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::LCD_CAM,
};

pub struct I8080<'d, TX> {
    lcd_cam: PeripheralRef<'d, LCD_CAM>,
    tx_channel: TX,
}

impl<'d, TX: Tx> I8080<'d, TX> {
    pub fn new(lcd: Lcd<'d>, mut channel: TX, frequency: HertzU32, clocks: &Clocks) -> Self {
        let lcd_cam = lcd.lcd_cam;

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

        lcd_cam.lcd_clock().modify(|_, w| {
            // Force enable the clock for all configuration register
            w.clk_en().set_bit();

            w.lcd_clk_sel().variant((i + 1) as _);
            w.lcd_clkm_div_num().variant(divider.div_num as _);
            w.lcd_clkm_div_b().variant(divider.div_b as _);
            w.lcd_clkm_div_a().variant(divider.div_a as _);

            // LCD_PCLK = LCD_CLK / 2
            w.lcd_clk_equ_sysclk().clear_bit();
            w.lcd_clkcnt_n().variant(2 - 1); // Must not be 0.

            w
        });
        lcd_cam.lcd_user().modify(|_, w| w.lcd_reset().set_bit());
        lcd_cam.lcd_clock().modify(|_, w| {
            // TODO: Expose as config.
            w.lcd_ck_idle_edge().clear_bit();
            w.lcd_ck_out_edge().clear_bit();
            w
        });

        lcd_cam
            .lcd_ctrl()
            .modify(|_, w| w.lcd_rgb_mode_en().clear_bit());
        lcd_cam
            .lcd_rgb_yuv()
            .modify(|_, w| w.lcd_conv_bypass().clear_bit());

        lcd_cam.lcd_user().modify(|_, w| {
            w.lcd_8bits_order().bit(false);
            w.lcd_bit_order().bit(false);
            w.lcd_byte_order().bit(false);
            w.lcd_2byte_en().bit(false);

            // Be able to send command in LCD sequence when LCD starts.
            w.lcd_cmd().set_bit();
            // The cycle length of command phase. 1: 2 cycles. 0: 1 cycle
            w.lcd_cmd_2_cycle_en().clear_bit();

            // Enable DUMMY phase in LCD sequence when LCD starts.
            w.lcd_dummy().clear_bit();
            // Configure DUMMY cycles. DUMMY cycles = this value + 1. (2 bits)
            w.lcd_dummy_cyclelen().variant(1);

            // Be able to send data out in LCD sequence when LCD starts.
            w.lcd_dout().set_bit();
            // Data length in fixed mode. (13 bits)
            w.lcd_dout_cyclelen().variant(0);
            // Disable continuous output.
            w.lcd_always_out_en().clear_bit();

            // w.lcd_update().set_bit();
            // w.lcd_start().set_bit();
            // w.lcd_reset().set_bit();
            w
        });
        lcd_cam.lcd_misc().modify(|_, w| {
            // Set the threshold for Async Tx FIFO full event. (5 bits)
            w.lcd_afifo_threshold_num().variant(0);

            // Configure the setup cycles in LCD non-RGB mode. Setup cycles
            // expected = this value + 1. (6 bit)
            w.lcd_vfk_cyclelen().variant(0);

            // Configure the hold time cycles in LCD non-RGB mode. Hold
            // cycles expected = this value + 1.
            w.lcd_vbk_cyclelen().variant(0);

            // 1: Send the next frame data when the current frame is sent out.
            // 0: LCD stops when the current frame is sent out.
            w.lcd_next_frame_en().clear_bit();

            // Enable blank region when LCD sends data out.
            w.lcd_bk_en().set_bit();

            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DOUT phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_data_set().set_bit();
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in DUMMY phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_dummy_set().clear_bit();
            // 1: LCD_CD = !LCD_CAM_LCD_CD_IDLE_EDGE when LCD is in CMD phase.
            // 0: LCD_CD = LCD_CAM_LCD_CD_IDLE_EDGE.
            w.lcd_cd_cmd_set().clear_bit();

            // The default value of LCD_CD
            w.lcd_cd_idle_edge().clear_bit();

            // w.lcd_afifo_reset().set_bit();
            w
        });

        lcd_cam.lcd_user().modify(|_, w| w.lcd_update().set_bit());

        channel.init_channel();

        Self {
            lcd_cam,
            tx_channel: channel,
        }
    }

    pub fn with_cs<CS: OutputPin>(self, cs: impl Peripheral<P = CS> + 'd) -> Self {
        crate::into_ref!(cs);
        cs.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_CS);

        self
    }

    pub fn with_ctrl_pins<DC: OutputPin, WRX: OutputPin>(
        self,
        dc: impl Peripheral<P = DC> + 'd,
        wrx: impl Peripheral<P = WRX> + 'd,
    ) -> Self {
        crate::into_ref!(dc);
        crate::into_ref!(wrx);

        dc.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DC);

        wrx.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_PCLK);

        self
    }

    pub fn with_data_pins<
        D0: OutputPin,
        D1: OutputPin,
        D2: OutputPin,
        D3: OutputPin,
        D4: OutputPin,
        D5: OutputPin,
        D6: OutputPin,
        D7: OutputPin,
    >(
        self,
        d0: impl Peripheral<P = D0> + 'd,
        d1: impl Peripheral<P = D1> + 'd,
        d2: impl Peripheral<P = D2> + 'd,
        d3: impl Peripheral<P = D3> + 'd,
        d4: impl Peripheral<P = D4> + 'd,
        d5: impl Peripheral<P = D5> + 'd,
        d6: impl Peripheral<P = D6> + 'd,
        d7: impl Peripheral<P = D7> + 'd,
    ) -> Self {
        crate::into_ref!(d0);
        crate::into_ref!(d1);
        crate::into_ref!(d2);
        crate::into_ref!(d3);
        crate::into_ref!(d4);
        crate::into_ref!(d5);
        crate::into_ref!(d6);
        crate::into_ref!(d7);

        d0.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_0);
        d1.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_1);
        d2.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_2);
        d3.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_3);
        d4.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_4);
        d5.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_5);
        d6.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_6);
        d7.set_to_push_pull_output()
            .connect_peripheral_to_output(OutputSignal::LCD_DATA_7);

        self
    }
}

impl<'d, TX: Tx> I8080<'d, TX> {
    pub fn send(&mut self, cmd: u8, data: &[u8]) -> Result<(), DmaError> {
        // Reset LCD control unit and Async Tx FIFO
        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_reset().set_bit());
        self.lcd_cam
            .lcd_misc()
            .modify(|_, w| w.lcd_afifo_reset().set_bit());

        if data.is_empty() {
            // Set transfer length.
            self.lcd_cam
                .lcd_user()
                .modify(|_, w| w.lcd_dout().clear_bit());
        } else {
            // Set transfer length.
            self.lcd_cam.lcd_user().modify(|_, w| {
                w.lcd_dout()
                    .set_bit()
                    .lcd_dout_cyclelen()
                    .variant((data.len() - 1) as _)
            });

            self.tx_channel.prepare_transfer_without_start(
                DmaPeripheral::LcdCam,
                false,
                data.as_ptr(),
                data.len(),
            )?;
            self.tx_channel.start_transfer()?;
        }

        // Set cmd value
        self.lcd_cam
            .lcd_cmd_val()
            .write(|w| w.lcd_cmd_value().variant(cmd as _));

        // Setup interrupts.
        self.lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());
        self.lcd_cam
            .lc_dma_int_ena()
            .modify(|_, w| w.lcd_trans_done_int_ena().set_bit());

        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_update().set_bit().lcd_start().set_bit());

        while self
            .lcd_cam
            .lc_dma_int_st()
            .read()
            .lcd_trans_done_int_st()
            .bit_is_clear()
        {}

        self.lcd_cam
            .lcd_user()
            .modify(|_, w| w.lcd_start().clear_bit());

        self.lcd_cam
            .lc_dma_int_ena()
            .modify(|_, w| w.lcd_trans_done_int_ena().clear_bit());
        self.lcd_cam
            .lc_dma_int_clr()
            .write(|w| w.lcd_trans_done_int_clr().set_bit());

        Ok(())
    }
}
