//! What a previous run's `ext1` configuration leaves behind.
//!
//! `ext1` holds its pads through a deep sleep, so the pads still hold their sleep levels when the
//! chip boots again. Releasing that hold is a boot-time job, and the only part of `ext1` this
//! module owns; arming the pads belongs to the GPIO driver.

use crate::gpio::LpPin;

/// Releases the pad hold that a previous sleep took, so the pads follow their drivers again.
///
/// This releases the hold and nothing else. Reassigning the pad function would steal a pad that
/// the application has since handed to an LP core, which owns its pads for as long as it runs.
pub(crate) fn wake_io_reset() {
    cfg_select! {
        sleep_ext1_version = "3" => {
            fn release(pin: impl LpPin, armed: u32) {
                // The selection is by digital pin number on this generation.
                if armed & (1 << pin.number()) != 0 {
                    pin.lp_pad_hold(false);
                }
            }

            let armed = crate::peripherals::PMU::regs()
                .ext_wakeup_sel()
                .read()
                .ext_wakeup_sel()
                .bits();
        }
        _ => {
            fn release(pin: impl LpPin, armed: u8) {
                if armed & (1 << pin.lp_number()) != 0 {
                    pin.lp_pad_hold(false);
                }
            }

            let armed = crate::peripherals::LP_AON::regs()
                .ext_wakeup_cntl()
                .read()
                .ext_wakeup_sel()
                .bits();
        }
    }

    for_each_lp_function! {
        (($_lp:ident, LP_GPIOn, $_pin:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
            release(unsafe { crate::peripherals::$gpio::steal() }, armed);
        };
    }

    // The pads are free now, so the selection that armed them can go too.
    #[cfg(sleep_ext1_version = "3")]
    crate::peripherals::PMU::regs().ext_wakeup_sel().reset();
}
