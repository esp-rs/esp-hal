//! Chip-specific LCD_CAM register helpers.

use crate::pac;

pub(super) fn set_lcd_conv_bypass(regs: &pac::lcd_cam::RegisterBlock) {
    regs.lcd_rgb_yuv().write(|w| {
        cfg_select! {
            esp32s31 => w.lcd_conv_enable().clear_bit(),
            _ => w.lcd_conv_bypass().clear_bit(),
        }
    });
}

pub(super) fn set_cam_conv_bypass(regs: &pac::lcd_cam::RegisterBlock) {
    regs.cam_rgb_yuv().write(|w| {
        cfg_select! {
            esp32s31 => w.cam_conv_enable().clear_bit(),
            _ => w.cam_conv_bypass().clear_bit(),
        }
    });
}

pub(super) fn set_rgb_mode_en(regs: &pac::lcd_cam::RegisterBlock, enable: bool) {
    let reg = cfg_select! {
        esp32s3 => regs.lcd_ctrl(),
        esp32s31 => regs.lcd_misc(),
    };
    reg.modify(|_, w| w.lcd_rgb_mode_en().bit(enable));
}

pub(super) fn set_2byte_mode(regs: &pac::lcd_cam::RegisterBlock, enable: bool) {
    regs.lcd_user().modify(|_, w| {
        cfg_select! {
            esp32s31 => unsafe { w.lcd_byte_mode().bits(if enable { 1 } else { 0 }) },
            _ => w.lcd_2byte_en().bit(enable),
        }
    });
}

pub(super) fn set_8bits_order(regs: &pac::lcd_cam::RegisterBlock, inverted: bool) {
    regs.lcd_user().modify(|_, w| {
        cfg_select! {
            esp32s31 => unsafe {
                w.lcd_dout_byte_swizzle_enable().bit(inverted);
                w.lcd_dout_byte_swizzle_mode().bits(0)
            },
            _ => w.lcd_8bits_order().bit(inverted),
        }
    });
}

pub(super) fn set_cd_delay(regs: &pac::lcd_cam::RegisterBlock, mode: u8) {
    let reg = cfg_select! {
        esp32s31 => regs.lcd_dly_mode_cfg1(),
        _ => regs.lcd_dly_mode(),
    };
    reg.write(|w| unsafe { w.lcd_cd_mode().bits(mode) });
}

pub(super) fn set_data_bit_delay(regs: &pac::lcd_cam::RegisterBlock, mode: u8) {
    let dout_0_15 = cfg_select! {
        esp32s31 => regs.lcd_dly_mode_cfg2(),
        _ => regs.lcd_data_dout_mode(),
    };

    dout_0_15.write(|w| unsafe {
        w.dout0_mode().bits(mode);
        w.dout1_mode().bits(mode);
        w.dout2_mode().bits(mode);
        w.dout3_mode().bits(mode);
        w.dout4_mode().bits(mode);
        w.dout5_mode().bits(mode);
        w.dout6_mode().bits(mode);
        w.dout7_mode().bits(mode);
        w.dout8_mode().bits(mode);
        w.dout9_mode().bits(mode);
        w.dout10_mode().bits(mode);
        w.dout11_mode().bits(mode);
        w.dout12_mode().bits(mode);
        w.dout13_mode().bits(mode);
        w.dout14_mode().bits(mode);
        w.dout15_mode().bits(mode)
    });

    #[cfg(rgb_display_output_lines = "24")]
    regs.lcd_dly_mode_cfg1().modify(|_, w| unsafe {
        w.dout16_mode().bits(mode);
        w.dout17_mode().bits(mode);
        w.dout18_mode().bits(mode);
        w.dout19_mode().bits(mode);
        w.dout20_mode().bits(mode);
        w.dout21_mode().bits(mode);
        w.dout22_mode().bits(mode);
        w.dout23_mode().bits(mode)
    });
}

pub(super) fn set_sync_delay(
    regs: &pac::lcd_cam::RegisterBlock,
    de_mode: u8,
    hsync_mode: u8,
    vsync_mode: u8,
) {
    let reg = cfg_select! {
        esp32s31 => regs.lcd_dly_mode_cfg1(),
        _ => regs.lcd_dly_mode(),
    };

    reg.modify(|_, w| unsafe {
        w.lcd_de_mode().bits(de_mode);
        w.lcd_hsync_mode().bits(hsync_mode);
        w.lcd_vsync_mode().bits(vsync_mode)
    });
}

pub(super) fn write_command(regs: &pac::lcd_cam::RegisterBlock, first: u32, second: Option<u32>) {
    cfg_select! {
        esp32s3 => {
            let value = match second {
                Some(second) => first | (second << 16),
                None => first,
            };
            regs.lcd_cmd_val()
                .write(|w| unsafe { w.lcd_cmd_value().bits(value) });
        }
        _ => {
            regs.lcd_first_cmd_val()
                .write(|w| unsafe { w.lcd_first_cmd_value().bits(first) });
            if let Some(second) = second {
                regs.lcd_latter_cmd_val()
                    .write(|w| unsafe { w.lcd_latter_cmd_value().bits(second) });
            }
        }
    }
}

#[expect(clippy::too_many_arguments)]
pub(super) fn configure_rgb_timing(
    regs: &pac::lcd_cam::RegisterBlock,
    hb_front: u16,
    va_height: u16,
    vt_height: u16,
    vb_front: u16,
    ha_width: u16,
    ht_width: u16,
    vsync_width: u16,
    vsync_idle_high: bool,
    de_idle_high: bool,
    hs_blank_en: bool,
    hsync_width: u8,
    hsync_idle_high: bool,
    hsync_position: u8,
) {
    cfg_select! {
        esp32s3 => {
            regs.lcd_ctrl().modify(|_, w| unsafe {
                w.lcd_rgb_mode_en().set_bit();
                w.lcd_hb_front().bits(hb_front);
                w.lcd_va_height().bits(va_height);
                w.lcd_vt_height().bits(vt_height)
            });
            regs.lcd_ctrl1().modify(|_, w| unsafe {
                w.lcd_vb_front().bits(vb_front as u8);
                w.lcd_ha_width().bits(ha_width);
                w.lcd_ht_width().bits(ht_width)
            });
            regs.lcd_ctrl2().modify(|_, w| unsafe {
                w.lcd_vsync_width().bits(vsync_width as u8);
                w.lcd_vsync_idle_pol().bit(vsync_idle_high);
                w.lcd_de_idle_pol().bit(de_idle_high);
                w.lcd_hs_blank_en().bit(hs_blank_en);
                w.lcd_hsync_width().bits(hsync_width);
                w.lcd_hsync_idle_pol().bit(hsync_idle_high);
                w.lcd_hsync_position().bits(hsync_position)
            });
        }
        _ => {
            regs.lcd_misc().modify(|_, w| w.lcd_rgb_mode_en().set_bit());
            regs.lcd_rgb_blank().write(|w| unsafe {
                w.lcd_hb_front().bits(hb_front);
                w.lcd_vb_front().bits(vb_front)
            });
            regs.lcd_rgb_vertical().write(|w| unsafe {
                w.lcd_va_height().bits(va_height);
                w.lcd_vt_height().bits(vt_height)
            });
            regs.lcd_rgb_horizontal().write(|w| unsafe {
                w.lcd_ha_width().bits(ha_width);
                w.lcd_ht_width().bits(ht_width)
            });
            regs.lcd_rgb_ctrl().write(|w| unsafe {
                w.lcd_vsync_width().bits(vsync_width);
                w.lcd_vsync_idle_pol().bit(vsync_idle_high);
                w.lcd_de_idle_pol().bit(de_idle_high);
                w.lcd_hs_blank_en().bit(hs_blank_en);
                w.lcd_hsync_width().bits(hsync_width);
                w.lcd_hsync_idle_pol().bit(hsync_idle_high);
                w.lcd_hsync_position().bits(hsync_position)
            });
        }
    }
}
