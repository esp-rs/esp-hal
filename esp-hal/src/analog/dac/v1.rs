use crate::peripherals::{RTC_IO, SENS};

pub(super) fn enable(index: usize) {
    #[cfg(esp32s2)]
    SENS::regs()
        .sar_dac_ctrl1()
        .modify(|_, w| w.dac_clkgate_en().set_bit());

    select_direct_source(index);

    // Prevent ADC sampling from powering the DAC back up after drop.
    // See <https://github.com/espressif/esp-idf/blob/4d80c1b29aeabc8e735ffc6e1be9bb23e95d1d5d/components/esp_hal_ana_conv/esp32/include/hal/dac_ll.h#L94-L104>.
    let sensors = SENS::regs();
    let control = cfg_select! {
        esp32 => sensors.sar_meas_ctrl2(),
        esp32s2 => sensors.sar_amp_ctrl3(),
    };
    control.modify(|_, w| unsafe { w.sar1_dac_xpd_fsm().bits(0) });

    RTC_IO::regs()
        .pad_dac(index)
        .modify(|_, w| w.dac_xpd_force().set_bit().xpd_dac().set_bit());
}

pub(super) fn write(index: usize, value: u8) {
    select_direct_source(index);
    RTC_IO::regs()
        .pad_dac(index)
        .modify(|_, w| unsafe { w.dac().bits(value) });
}

pub(super) fn disable(index: usize) {
    let rtc_io = RTC_IO::regs();
    rtc_io
        .pad_dac(index)
        .modify(|_, w| w.xpd_dac().clear_bit().dac_xpd_force().clear_bit());

    // Both channels share the clock gate.
    // See <https://github.com/espressif/esp-idf/blob/4d80c1b29aeabc8e735ffc6e1be9bb23e95d1d5d/components/esp_hal_ana_conv/esp32s2/include/hal/dac_ll.h#L49-L56>.
    #[cfg(esp32s2)]
    if rtc_io.pad_dac(0).read().xpd_dac().bit_is_clear()
        && rtc_io.pad_dac(1).read().xpd_dac().bit_is_clear()
    {
        SENS::regs()
            .sar_dac_ctrl1()
            .modify(|_, w| w.dac_clkgate_en().clear_bit());
    }
}

fn select_direct_source(index: usize) {
    // Select the channel source before releasing the shared DMA override.
    // See <https://github.com/espressif/esp-idf/blob/4d80c1b29aeabc8e735ffc6e1be9bb23e95d1d5d/components/esp_hal_ana_conv/esp32s2/include/hal/dac_ll.h#L66-L81>.
    SENS::regs()
        .sar_dac_ctrl2()
        .modify(|_, w| w.dac_cw_en(index as u8).clear_bit());
    SENS::regs()
        .sar_dac_ctrl1()
        .modify(|_, w| w.dac_dig_force().clear_bit());
    #[cfg(esp32s2)]
    crate::peripherals::APB_SARADC::regs()
        .apb_dac_ctrl()
        .modify(|_, w| w.trans().clear_bit());
}
