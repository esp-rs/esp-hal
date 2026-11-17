use crate::peripherals::DAC;

pub(super) fn enable(index: usize) {
    let dac = DAC::regs();
    dac.data_output_cfg().modify(|_, w| match index {
        0 => w.pad_0_data_sel().clear_bit(),
        1 => w.pad_1_data_sel().clear_bit(),
        _ => unreachable!(),
    });

    // Direct output uses the shared Sintx timer with its phase accumulator disabled.
    // See <https://github.com/espressif/esp-idf/blob/d445e947156f276dfe0db5804e4edf6ae90fe6fb/components/esp_hal_ana_conv/esp32s31/include/hal/dac_ll.h#L412-L422>.
    dac.sintx_cfg().modify(|_, w| w.sw_tone().clear_bit());
    dac.sintx_timer_cfg()
        .modify(|_, w| w.sintx_timer_en().set_bit());

    dac.pad_cfg().modify(|_, w| match index {
        0 => w.xpd_pad_0().set_bit(),
        1 => w.xpd_pad_1().set_bit(),
        _ => unreachable!(),
    });
}

pub(super) fn write(index: usize, value: u16) {
    DAC::regs().sintx_data().modify(|_, w| unsafe {
        match index {
            0 => w.dc_1().bits(value),
            1 => w.dc_2().bits(value),
            _ => unreachable!(),
        }
    });
}

pub(super) fn disable(index: usize) {
    let dac = DAC::regs();
    dac.pad_cfg().modify(|_, w| match index {
        0 => w.xpd_pad_0().clear_bit().dbuf_ls_pad_0().clear_bit(),
        1 => w.xpd_pad_1().clear_bit().dbuf_ls_pad_1().clear_bit(),
        _ => unreachable!(),
    });
    let pads = dac.pad_cfg().read();
    if pads.xpd_pad_0().bit_is_clear() && pads.xpd_pad_1().bit_is_clear() {
        dac.sintx_timer_cfg()
            .modify(|_, w| w.sintx_timer_en().clear_bit());
    }
}
