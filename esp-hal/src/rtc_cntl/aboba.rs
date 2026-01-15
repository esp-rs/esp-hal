/// Calibration of RTC_SLOW_CLK is performed using a special feature of
/// TIMG0. This feature counts the number of XTAL clock cycles within a
/// given number of RTC_SLOW_CLK cycles.
#[cfg(any(esp32c5, esp32c6, esp32h2))]
pub(crate) fn calibrate_internal(mut cal_clk: RtcCalSel, slowclk_cycles: u32) -> u32 {
    use crate::peripherals::{LP_CLKRST, PCR, PMU};

    #[derive(Clone, Copy)]
    enum RtcCaliClkSel {
        CaliClkRcSlow = 0,
        CaliClkRcFast = 1,
        CaliClk32k    = 2,
    }

    if cal_clk == RtcCalSel::RtcMux {
        cal_clk = match cal_clk {
            RtcCalSel::RtcMux => match RtcClock::slow_freq() {
                RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
                #[cfg(not(esp32c5))]
                RtcSlowClock::_32kRc => RtcCalSel::_32kRc,
                _ => cal_clk,
            },
            RtcCalSel::_32kOscSlow => RtcCalSel::RtcMux,
            _ => cal_clk,
        };
    }

    let clk_src = RtcClock::slow_freq();

    if cal_clk == RtcCalSel::RtcMux {
        cal_clk = match clk_src {
            RtcSlowClock::RcSlow => RtcCalSel::RcSlow,
            RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
            #[cfg(not(esp32c5))]
            RtcSlowClock::_32kRc => RtcCalSel::_32kRc,
            RtcSlowClock::OscSlow => RtcCalSel::_32kOscSlow,
        };
    }

    let cali_clk_sel;
    if cal_clk == RtcCalSel::RtcMux {
        cal_clk = match clk_src {
            RtcSlowClock::RcSlow => RtcCalSel::RcSlow,
            RtcSlowClock::_32kXtal => RtcCalSel::_32kXtal,
            #[cfg(not(esp32c5))]
            RtcSlowClock::_32kRc => RtcCalSel::_32kRc,
            RtcSlowClock::OscSlow => RtcCalSel::RcSlow,
        }
    }

    if cal_clk == RtcCalSel::RcFast {
        cali_clk_sel = RtcCaliClkSel::CaliClkRcFast;
    } else if cal_clk == RtcCalSel::RcSlow {
        cali_clk_sel = RtcCaliClkSel::CaliClkRcSlow;
    } else {
        cali_clk_sel = RtcCaliClkSel::CaliClk32k;

        // clk_ll_freq_calulation_set_target
        // #[cfg(not(esp32c5))]
        match cal_clk {
            RtcCalSel::RtcMux | RtcCalSel::RcSlow | RtcCalSel::RcFast => {}
            #[cfg(not(esp32c5))]
            RtcCalSel::_32kRc => {
                PCR::regs()
                    .ctrl_32k_conf()
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(0) });
            }
            RtcCalSel::_32kXtal => {
                PCR::regs()
                    .ctrl_32k_conf()
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(1) });
            }
            RtcCalSel::_32kOscSlow => {
                PCR::regs()
                    .ctrl_32k_conf()
                    .modify(|_, w| unsafe { w.clk_32k_sel().bits(2) });
            }
        }

        // #[cfg(esp32c5)]
        // match cal_clk {
        //     RtcCalSel::RtcMux => {}
        //     RtcCalSel::_32kRc => {
        //         PCR::regs()
        //             .ctrl_32k_conf()
        //             .modify(|_, w| unsafe { w.clk_32k_sel().bits(0) });
        //     }
        //     RtcCalSel::_32kXtal => {
        //         PCR::regs()
        //             .ctrl_32k_conf()
        //             .modify(|_, w| unsafe { w.clk_32k_sel().bits(1) });
        //     }
        //     RtcCalSel::_32kOscSlow => {
        //         PCR::regs()
        //             .ctrl_32k_conf()
        //             .modify(|_, w| unsafe { w.clk_32k_sel().bits(2) });
        //     }
        //     RtcCalSel::RcSlow => {
        //         PCR::regs()
        //             .ctrl_32k_conf()
        //             .modify(|_, w| unsafe { w.clk_32k_sel().bits(3) });
        //     }

        //     RtcCalSel::RcFast => {
        //         PCR::regs()
        //             .ctrl_32k_conf()
        //             .modify(|_, w| unsafe { w.clk_32k_sel().bits(4) });
        //     }
        // }
    }

    // Enable requested clock (150k is always on)
    // Some delay is required before the time is stable
    // Only enable if originaly was disabled
    // If clock is already on, do nothing

    let dig_32k_xtal_enabled = LP_CLKRST::regs()
        .clk_to_hp()
        .read()
        .icg_hp_xtal32k()
        .bit_is_set();

    if cal_clk == RtcCalSel::_32kXtal && !dig_32k_xtal_enabled {
        LP_CLKRST::regs()
            .clk_to_hp()
            .modify(|_, w| w.icg_hp_xtal32k().set_bit());
    }

    // TODO: very hacky - icg_hp_xtal32k is already set in the above condition?
    // in ESP-IDF these are not called in this function but the fields are set
    LP_CLKRST::regs()
        .clk_to_hp()
        .modify(|_, w| w.icg_hp_xtal32k().set_bit());
    PMU::regs().hp_sleep_lp_ck_power().modify(|_, w| {
        w.hp_sleep_xpd_xtal32k().set_bit();
        #[cfg(not(esp32c5))]
        w.hp_sleep_xpd_rc32k().set_bit()
    });

    let rc_fast_enabled = PMU::regs()
        .hp_sleep_lp_ck_power()
        .read()
        .hp_sleep_xpd_fosc_clk()
        .bit_is_set();
    let dig_rc_fast_enabled = LP_CLKRST::regs()
        .clk_to_hp()
        .read()
        .icg_hp_fosc()
        .bit_is_set();

    if cal_clk == RtcCalSel::RcFast {
        if !rc_fast_enabled {
            PMU::regs()
                .hp_sleep_lp_ck_power()
                .modify(|_, w| w.hp_sleep_xpd_fosc_clk().set_bit());
            crate::rom::ets_delay_us(50);
        }

        if !dig_rc_fast_enabled {
            LP_CLKRST::regs()
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_fosc().set_bit());
            crate::rom::ets_delay_us(5);
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(not(esp32c5))] {
            let rc32k_enabled = PMU::regs()
                .hp_sleep_lp_ck_power()
                .read()
                .hp_sleep_xpd_rc32k()
                .bit_is_set();
            let dig_rc32k_enabled = LP_CLKRST::regs()
                .clk_to_hp()
                .read()
                .icg_hp_osc32k()
                .bit_is_set();

            if cal_clk == RtcCalSel::_32kRc {
                if !rc32k_enabled {
                    PMU::regs()
                        .hp_sleep_lp_ck_power()
                        .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
                    crate::rom::ets_delay_us(300);
                }

                if !dig_rc32k_enabled {
                    LP_CLKRST::regs()
                        .clk_to_hp()
                        .modify(|_, w| w.icg_hp_osc32k().set_bit());
                }
            }
        } else {
            PCR::regs().timergroup_xtal_conf().modify(|_, w| w.tg0_xtal_clk_en().set_bit());
        }
    }

    // Check if there is already running calibration process
    // TODO: &mut TIMG0 for calibration
    let timg0 = TIMG0::regs();

    if timg0
        .rtccalicfg()
        .read()
        .rtc_cali_start_cycling()
        .bit_is_set()
    {
        timg0
            .rtccalicfg2()
            .modify(|_, w| unsafe { w.rtc_cali_timeout_thres().bits(1) });

        // Set small timeout threshold to accelerate the generation of timeot
        // Internal circuit will be reset when timeout occurs and will not affect the
        // next calibration
        while !timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set()
            && !timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set()
        {}
    }

    // Prepare calibration
    timg0
        .rtccalicfg()
        .modify(|_, w| unsafe { w.rtc_cali_clk_sel().bits(cali_clk_sel as u8) });
    timg0
        .rtccalicfg()
        .modify(|_, w| w.rtc_cali_start_cycling().clear_bit());
    timg0
        .rtccalicfg()
        .modify(|_, w| unsafe { w.rtc_cali_max().bits(slowclk_cycles as u16) });

    // Set timeout reg and expect time delay
    // REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES, CLK_CAL_TIMEOUT_THRES(cal_clk_sel, slowclk_cycles));
    // #define CLK_CAL_TIMEOUT_THRES(cal_clk_sel, cycles) ((cal_clk_sel == CLK_CAL_32K_XTAL || cal_clk_sel == CLK_CAL_32K_OSC_SLOW) ? (cycles << 12) : (cycles << 10))
    let expected_frequency = match cali_clk_sel {
        RtcCaliClkSel::CaliClk32k => {
            timg0.rtccalicfg2().modify(|_, w| unsafe {
                w.rtc_cali_timeout_thres().bits(slowclk_cycles << 12)
            });
            RtcSlowClock::_32kXtal.frequency()
        }
        RtcCaliClkSel::CaliClkRcFast => {
            timg0.rtccalicfg2().modify(|_, w| unsafe {
                w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
            });
            cfg_if::cfg_if! {
                if #[cfg(esp32c5)] {
                    RtcFastClock::RcFast.frequency() >> 5
                } else {
                    RtcFastClock::RcFast.frequency()
                }
            }
        }
        RtcCaliClkSel::CaliClkRcSlow => {
            timg0.rtccalicfg2().modify(|_, w| unsafe {
                w.rtc_cali_timeout_thres().bits(slowclk_cycles << 10)
            });
            RtcSlowClock::RcSlow.frequency()
        }
    };

    let us_time_estimate = Rate::from_mhz(slowclk_cycles) / expected_frequency;

    // Start calibration
    timg0
        .rtccalicfg()
        .modify(|_, w| w.rtc_cali_start().clear_bit());
    timg0
        .rtccalicfg()
        .modify(|_, w| w.rtc_cali_start().set_bit());

    // Wait for calibration to finish up to another us_time_estimate
    crate::rom::ets_delay_us(us_time_estimate);

    let cal_val = loop {
        if timg0.rtccalicfg().read().rtc_cali_rdy().bit_is_set() {
            // The Fosc CLK of calibration circuit is divided by 32 for ECO1.
            // So we need to multiply the frequency of the Fosc for ECO1 and above chips by
            // 32 times. And ensure that this modification will not affect
            // ECO0.
            // https://github.com/espressif/esp-idf/commit/e3148369f32fdc6de62c35a67f7adb6f4faef4e3
            #[cfg(esp32c6)]
            if Efuse::chip_revision() > 0 && cal_clk == RtcCalSel::RcFast {
                break timg0.rtccalicfg1().read().rtc_cali_value().bits() >> 5;
            }
            #[cfg(esp32c5)]
            if cal_clk == RtcCalSel::RcFast {
                break timg0.rtccalicfg1().read().rtc_cali_value().bits() >> 5;
            }
            break timg0.rtccalicfg1().read().rtc_cali_value().bits();
        }

        if timg0.rtccalicfg2().read().rtc_cali_timeout().bit_is_set() {
            // Timed out waiting for calibration
            break 0;
        }
    };

    timg0
        .rtccalicfg()
        .modify(|_, w| w.rtc_cali_start().clear_bit());

    #[cfg(esp32c5)]
    PCR::regs().timergroup_xtal_conf().modify(|_, w| w.tg0_xtal_clk_en().clear_bit());

    if cal_clk == RtcCalSel::_32kXtal && !dig_32k_xtal_enabled {
        LP_CLKRST::regs()
            .clk_to_hp()
            .modify(|_, w| w.icg_hp_xtal32k().clear_bit());
    }

    if cal_clk == RtcCalSel::RcFast {
        if !rc_fast_enabled {
            PMU::regs()
                .hp_sleep_lp_ck_power()
                .modify(|_, w| w.hp_sleep_xpd_fosc_clk().clear_bit());
            crate::rom::ets_delay_us(50);
        }

        if !dig_rc_fast_enabled {
            LP_CLKRST::regs()
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_fosc().clear_bit());
            crate::rom::ets_delay_us(5);
        }
    }

    #[cfg(not(esp32c5))]
    if cal_clk == RtcCalSel::_32kRc {
        let rc32k_enabled = PMU::regs()
            .hp_sleep_lp_ck_power()
            .read()
            .hp_sleep_xpd_rc32k()
            .bit_is_set();
        let dig_rc32k_enabled = LP_CLKRST::regs()
            .clk_to_hp()
            .read()
            .icg_hp_osc32k()
            .bit_is_set();

        if rc32k_enabled {
            PMU::regs()
                .hp_sleep_lp_ck_power()
                .modify(|_, w| w.hp_sleep_xpd_rc32k().set_bit());
            crate::rom::ets_delay_us(300);
        }
        if dig_rc32k_enabled {
            LP_CLKRST::regs()
                .clk_to_hp()
                .modify(|_, w| w.icg_hp_osc32k().set_bit());
        }
    }

    cal_val
}
