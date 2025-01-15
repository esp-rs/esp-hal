//! Helper functions for TRNG functionality

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    let rtc_cntl = unsafe { &*crate::peripherals::LPWR::ptr() };
    let sens = unsafe { &*crate::peripherals::SENS::ptr() };
    let dport = unsafe { &*crate::peripherals::DPORT::ptr() };
    let apb_ctrl = unsafe { &*crate::peripherals::APB_CTRL::ptr() };
    let i2s0 = unsafe { &*crate::peripherals::I2S0::ptr() };

    unsafe {
        rtc_cntl.test_mux().modify(|_, w| w.dtest_rtc().bits(2));

        rtc_cntl.test_mux().modify(|_, w| w.ent_rtc().set_bit());

        sens.sar_start_force()
            .modify(|_, w| w.sar2_en_test().set_bit());

        // periph_module_enable(PERIPH_I2S0_MODULE);
        dport
            .perip_clk_en()
            .modify(|_, w| w.i2c0_ext0_clk_en().set_bit());

        dport
            .perip_rst_en()
            .modify(|_, w| w.i2c0_ext0_rst().clear_bit());

        sens.sar_start_force()
            .modify(|_, w| w.ulp_cp_force_start_top().clear_bit());

        sens.sar_start_force()
            .modify(|_, w| w.ulp_cp_start_top().clear_bit());

        // Test pattern configuration byte 0xAD:
        //--[7:4] channel_sel: 10-->en_test
        //--[3:2] bit_width  : 3-->12bit
        //--[1:0] atten      : 1-->3dB attenuation
        apb_ctrl
            .apb_saradc_sar2_patt_tab1()
            .write(|w| w.bits(0xADADADAD));
        apb_ctrl
            .apb_saradc_sar2_patt_tab2()
            .write(|w| w.bits(0xADADADAD));
        apb_ctrl
            .apb_saradc_sar2_patt_tab3()
            .write(|w| w.bits(0xADADADAD));
        apb_ctrl
            .apb_saradc_sar2_patt_tab4()
            .write(|w| w.bits(0xADADADAD));

        sens.sar_meas_wait2()
            .modify(|_, w| w.force_xpd_sar().bits(3));

        sens.sar_read_ctrl()
            .modify(|_, w| w.sar1_dig_force().set_bit());

        sens.sar_read_ctrl2()
            .modify(|_, w| w.sar2_dig_force().set_bit());

        apb_ctrl
            .apb_saradc_ctrl()
            .modify(|_, w| w.saradc_sar2_mux().set_bit());

        apb_ctrl
            .apb_saradc_ctrl()
            .modify(|_, w| w.saradc_sar_clk_div().bits(4));

        apb_ctrl
            .apb_saradc_fsm()
            .modify(|_, w| w.saradc_rstb_wait().bits(8));

        apb_ctrl
            .apb_saradc_fsm()
            .modify(|_, w| w.saradc_start_wait().bits(10));

        apb_ctrl
            .apb_saradc_ctrl()
            .modify(|_, w| w.saradc_work_mode().bits(0));

        apb_ctrl
            .apb_saradc_ctrl()
            .modify(|_, w| w.saradc_sar_sel().set_bit());

        apb_ctrl
            .apb_saradc_ctrl()
            .modify(|_, w| w.saradc_data_sar_sel().clear_bit());

        i2s0.sample_rate_conf()
            .modify(|_, w| w.rx_bck_div_num().bits(20));

        apb_ctrl
            .apb_saradc_ctrl()
            .modify(|_, w| w.saradc_data_to_i2s().set_bit());

        i2s0.conf2().modify(|_, w| w.camera_en().set_bit());

        i2s0.conf2().modify(|_, w| w.lcd_en().set_bit());

        i2s0.conf2().modify(|_, w| w.data_enable().set_bit());

        i2s0.conf2()
            .modify(|_, w| w.data_enable_test_en().set_bit());

        i2s0.conf().modify(|_, w| w.rx_start().set_bit());
    }
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
    let sens = unsafe { &*crate::peripherals::SENS::ptr() };
    let i2s0 = unsafe { &*crate::peripherals::I2S0::ptr() };
    let apb_ctrl = unsafe { &*crate::peripherals::APB_CTRL::ptr() };

    unsafe {
        i2s0.conf().modify(|_, w| w.rx_start().clear_bit());

        i2s0.conf().modify(|_, w| w.rx_reset().set_bit());

        i2s0.conf().modify(|_, w| w.rx_reset().clear_bit());

        i2s0.conf2().modify(|_, w| {
            w.camera_en()
                .clear_bit()
                .lcd_en()
                .clear_bit()
                .data_enable_test_en()
                .clear_bit()
                .data_enable()
                .clear_bit()
        });

        sens.sar_read_ctrl()
            .modify(|_, w| w.sar1_dig_force().clear_bit());

        sens.sar_read_ctrl2()
            .modify(|_, w| w.sar2_dig_force().clear_bit());

        sens.sar_start_force()
            .modify(|_, w| w.sar2_en_test().clear_bit());

        apb_ctrl.apb_saradc_ctrl().modify(|_, w| {
            w.saradc_sar2_mux()
                .clear_bit()
                .saradc_sar_sel()
                .clear_bit()
                .saradc_data_to_i2s()
                .clear_bit()
        });

        sens.sar_meas_wait2()
            .modify(|_, w| w.force_xpd_sar().bits(0));

        apb_ctrl
            .apb_saradc_fsm()
            .modify(|_, w| w.saradc_start_wait().bits(8));
    }
}
