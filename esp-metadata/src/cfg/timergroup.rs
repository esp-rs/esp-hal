use quote::quote;

use crate::{cfg::GenericProperty, number};

#[derive(Debug, Clone, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct RcFastCalibrationProperties {
    divider: u32,
    #[serde(default)]
    min_rev: u32,
}

impl GenericProperty for RcFastCalibrationProperties {
    fn cfgs(&self) -> Option<Vec<String>> {
        Some(vec![String::from("timergroup_rc_fast_calibration_divider")])
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        let divider = number(self.divider);
        let min_rev = number(self.min_rev);
        quote! {
            ("timergroup.rc_fast_calibration_divider_min_rev") => {
                #min_rev
            };
            ("timergroup.rc_fast_calibration_divider") => {
                #divider
            };
        }
    }
}
