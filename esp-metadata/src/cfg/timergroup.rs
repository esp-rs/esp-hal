use quote::quote;

use crate::{cfg::GenericProperty, number};

#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RcFastCalibrationProperties {
    divider: u32,
    #[serde(default)]
    min_rev: u32,
    #[serde(default)]
    tick_enable: bool,
}

/// The RC_SLOW calibration input may be tapped ahead of the divider that produces the RTC slow
/// clock, in which case the calibration counts `multiplier` input cycles per measured cycle.
#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RcSlowCalibrationProperties {
    multiplier: u32,
}

impl GenericProperty for RcSlowCalibrationProperties {
    fn cfgs(&self) -> Option<Vec<String>> {
        Some(vec![String::from(
            "timergroup_rc_slow_calibration_multiplier",
        )])
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        let multiplier = number(self.multiplier);
        quote! {
            ("timergroup.rc_slow_calibration_multiplier") => {
                #multiplier
            };
        }
    }
}

impl GenericProperty for RcFastCalibrationProperties {
    fn cfgs(&self) -> Option<Vec<String>> {
        let mut cfgs = vec![String::from("timergroup_rc_fast_calibration_divider")];
        if self.tick_enable {
            cfgs.push(String::from("timergroup_rc_fast_calibration_tick_enable"));
        }
        Some(cfgs)
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        let divider = number(self.divider);
        let min_rev = number(self.min_rev);
        let tick_enable = self.tick_enable;
        quote! {
            ("timergroup.rc_fast_calibration_divider_min_rev") => {
                #min_rev
            };
            ("timergroup.rc_fast_calibration_divider") => {
                #divider
            };
            ("timergroup.rc_fast_calibration_tick_enable") => {
                #tick_enable
            };
        }
    }
}
