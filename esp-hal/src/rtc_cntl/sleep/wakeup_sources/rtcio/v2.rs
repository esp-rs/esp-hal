use super::RtcioWakeupSource;
use crate::{
    gpio::{AlternateFunction, Level, LpPin, OutputSignal, WakeEvent, lp_io::low_level},
    peripherals::{GPIO, IO_MUX, LPWR},
    rtc_cntl::{Rtc, RtcSleepConfig, WakeSource, WakeTriggers, WakeupSource},
};

impl RtcioWakeupSource<'_, '_> {
    fn apply_pin(&self, pin: &mut dyn LpPin, level: Level) {
        let lp_pin = pin.lp_number();

        // The pullup/pulldown part is like in gpio_deep_sleep_wakeup_prepare
        let level = match level {
            Level::High => {
                low_level::pullup_enable(lp_pin, false);
                low_level::pulldown_enable(lp_pin, true);
                WakeEvent::HighLevel
            }
            Level::Low => {
                low_level::pullup_enable(lp_pin, true);
                low_level::pulldown_enable(lp_pin, false);
                WakeEvent::LowLevel
            }
        };
        low_level::pad_hold(lp_pin, true);

        // apply_wakeup does the same as idf's esp_deep_sleep_enable_gpio_wakeup
        low_level::apply_wakeup(lp_pin, true, level);
    }
}

fn isolate_digital_gpio() {
    let dig_iso = LPWR::regs().dig_iso().read();
    let deep_sleep_hold_is_en =
        !dig_iso.dg_pad_force_unhold().bit() && dig_iso.dg_pad_autohold_en().bit();
    if !deep_sleep_hold_is_en {
        return;
    }

    let pin_hold = LPWR::regs().dig_pad_hold().read().bits();
    for_each_gpio! {
        ($n:literal, $($rest:tt)*) => {
            if pin_hold & (1 << $n) == 0 {
                isolate_one_gpio($n);
            }
        };
    };
}

fn isolate_one_gpio(pin_num: usize) {
    // output disable, like gpio_ll_output_disable
    GPIO::regs()
        .func_out_sel_cfg(pin_num)
        .modify(|_, w| unsafe { w.bits(OutputSignal::GPIO as u32) });

    IO_MUX::regs().gpio(pin_num).modify(|_, w| unsafe {
        // disable pull-up and pull-down
        w.fun_wpu().clear_bit();
        w.fun_wpd().clear_bit();

        // input disable, like gpio_ll_input_disable
        w.fun_ie().clear_bit();
        // make pad work as gpio (otherwise, deep_sleep bottom current will rise)
        w.mcu_sel().bits(AlternateFunction::GPIO as u8)
    });
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
