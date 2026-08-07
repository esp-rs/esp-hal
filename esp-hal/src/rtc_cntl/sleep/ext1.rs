//! The `ext1` configuration that a previous run leaves in the hardware.
//!
//! `ext1` holds its pads through a deep sleep, so the pads still keep their sleep levels when the
//! chip boots again. The boot must release that hold. This module does only that part of `ext1`.
//! The GPIO driver arms the pads.

use crate::gpio::lp_io::low_level;

/// Releases the pad hold of the previous sleep, so that the pads follow their drivers again.
///
/// The function releases the hold, and changes nothing else. A change of the pad function can take
/// a pad away from an LP core, because the application can give a pad to that core, and the core
/// keeps the pad while it runs.
pub(crate) fn wake_io_reset() {
    cfg_select! {
        sleep_ext1_version = "3" => {
            // This generation selects the pads by digital pin number.
            fn release(gpio: u8, lp: u8, armed: u32) {
                if armed & (1 << gpio) != 0 {
                    low_level::pad_hold(lp, false);
                }
            }

            let armed = crate::peripherals::PMU::regs()
                .ext_wakeup_sel()
                .read()
                .ext_wakeup_sel()
                .bits();
        }
        _ => {
            fn release(_gpio: u8, lp: u8, armed: u8) {
                if armed & (1 << lp) != 0 {
                    low_level::pad_hold(lp, false);
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
        (($_lp:ident, LP_GPIOn, $lp:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) => {
            release(crate::peripherals::$gpio::NUMBER, $lp, armed);
        };
    }

    // The pads are free now, so clear the selection that armed them.
    #[cfg(sleep_ext1_version = "3")]
    crate::peripherals::PMU::regs().ext_wakeup_sel().reset();
}
