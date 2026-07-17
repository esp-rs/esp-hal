use super::{RtcioWakeupSource, WakeupLevel};
use crate::{
    gpio::{RtcFunction, RtcPinWithResistors},
    peripherals::{GPIO, IO_MUX, LPWR},
    rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource},
};

const GPIO_INTR_LOW_LEVEL: u8 = 4;
const GPIO_INTR_HIGH_LEVEL: u8 = 5;
const SIG_GPIO_OUT_IDX: u32 = 128;
const GPIO_NUM_MAX: usize = 22;

impl RtcioWakeupSource<'_, '_> {
    fn apply_pin(&self, pin: &mut dyn RtcPinWithResistors, level: WakeupLevel) {
        // The pullup/pulldown part is like in gpio_deep_sleep_wakeup_prepare
        let level = match level {
            WakeupLevel::High => {
                pin.rtcio_pullup(false);
                pin.rtcio_pulldown(true);
                GPIO_INTR_HIGH_LEVEL
            }
            WakeupLevel::Low => {
                pin.rtcio_pullup(true);
                pin.rtcio_pulldown(false);
                GPIO_INTR_LOW_LEVEL
            }
        };
        pin.rtcio_pad_hold(true);

        // apply_wakeup does the same as idf's esp_deep_sleep_enable_gpio_wakeup
        unsafe {
            pin.apply_wakeup(true, level);
        }
    }
}

fn isolate_digital_gpio() {
    // like esp_sleep_isolate_digital_gpio
    let rtc_cntl = LPWR::regs();
    let io_mux = IO_MUX::regs();
    let gpio = GPIO::regs();

    let dig_iso = &rtc_cntl.dig_iso().read();
    let deep_sleep_hold_is_en =
        !dig_iso.dg_pad_force_unhold().bit() && dig_iso.dg_pad_autohold_en().bit();
    if !deep_sleep_hold_is_en {
        return;
    }

    // TODO: assert that the task stack is not in external ram

    for pin_num in 0..GPIO_NUM_MAX {
        let pin_hold = rtc_cntl.dig_pad_hold().read().bits() & (1 << pin_num) != 0;
        if !pin_hold {
            // input disable, like gpio_ll_input_disable
            io_mux.gpio(pin_num).modify(|_, w| w.fun_ie().clear_bit());
            // output disable, like gpio_ll_output_disable
            unsafe {
                gpio.func_out_sel_cfg(pin_num)
                    .modify(|_, w| w.bits(SIG_GPIO_OUT_IDX));
            }

            // disable pull-up and pull-down
            io_mux.gpio(pin_num).modify(|_, w| w.fun_wpu().clear_bit());
            io_mux.gpio(pin_num).modify(|_, w| w.fun_wpd().clear_bit());

            // make pad work as gpio (otherwise, deep_sleep bottom current will rise)
            io_mux
                .gpio(pin_num)
                .modify(|_, w| unsafe { w.mcu_sel().bits(RtcFunction::Digital as u8) });
        }
    }
}

fn prepare_gpio_wakeup() {
    let reg = cfg_select! {
        esp32c2 => LPWR::regs().cntl_gpio_wakeup(),
        esp32c3 => LPWR::regs().gpio_wakeup(),
    };

    reg.modify(|_, w| w.gpio_pin_clk_gate().set_bit());

    LPWR::regs()
        .ext_wakeup_conf()
        .modify(|_, w| w.gpio_wakeup_filter().set_bit());
}

fn clear_gpio_wakeup_status() {
    let reg = cfg_select! {
        esp32c2 => LPWR::regs().cntl_gpio_wakeup(),
        esp32c3 => LPWR::regs().gpio_wakeup(),
    };

    reg.modify(|_, w| w.gpio_wakeup_status_clr().set_bit());
    reg.modify(|_, w| w.gpio_wakeup_status_clr().clear_bit());
}

impl WakeSource for RtcioWakeupSource<'_, '_> {
    fn apply(
        &self,
        _rtc: &Rtc<'_>,
        triggers: &mut WakeTriggers,
        sleep_config: &mut RtcSleepConfig,
    ) {
        let mut pins = self.pins.borrow_mut();

        if pins.is_empty() {
            return;
        }

        triggers.insert(WakeupSource::Gpio);

        // If deep sleep is enabled, esp_start_sleep calls
        // gpio_deep_sleep_wakeup_prepare which sets these pullup and
        // pulldown values. But later in esp_start_sleep it calls
        // esp_sleep_isolate_digital_gpio, which disables the pullup and pulldown (but
        // only if it isn't held).
        // But it looks like gpio_deep_sleep_wakeup_prepare enables hold for all pins
        // in the wakeup mask.
        //
        // So: all pins in the wake mask should get this treatment here, and all pins
        // not in the wake mask should get
        // - pullup and pulldowns disabled
        // - input and output disabled, and
        // - their func should get set to GPIO.
        // But this last block of things gets skipped if hold is disabled globally (see
        // gpio_ll_deep_sleep_hold_is_en)

        prepare_gpio_wakeup();

        if sleep_config.deep_slp() {
            for (pin, level) in pins.iter_mut() {
                self.apply_pin(*pin, *level);
            }

            isolate_digital_gpio();
        }

        // like rtc_cntl_ll_gpio_clear_wakeup_status, as called from
        // gpio_deep_sleep_wakeup_prepare
        clear_gpio_wakeup_status();
    }
}
