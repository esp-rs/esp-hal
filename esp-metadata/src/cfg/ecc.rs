use serde::{Deserialize, Serialize};

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct EccDriverProperties {
    working_modes: Vec<WorkingModeEntry>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct WorkingModeEntry {
    id: u32,
    mode: WorkingMode,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, strum::Display)]
#[serde(rename_all = "snake_case")]
enum WorkingMode {
    AffinePointMultiplication,
    AffinePointVerification,
    AffinePointVerificationAndMultiplication,
    AffinePointAddition,
    JacobianPointMultiplication,
    JacobianPointVerification,
    AffinePointVerificationAndJacobianPointMultiplication,
    FiniteFieldDivision,
    ModularAddition,
    ModularSubtraction,
    ModularMultiplication,
    ModularDivision,
}

impl super::GenericProperty for EccDriverProperties {
    fn cfgs(&self) -> Option<Vec<String>> {
        Some(vec![format!(
            "ecc_working_modes = \"{}\"",
            self.working_modes.len()
        )])
    }

    fn macros(&self) -> Option<proc_macro2::TokenStream> {
        let branches = self
            .working_modes
            .iter()
            .map(|entry| {
                let id = crate::number(entry.id);
                let mode = quote::format_ident!("{}", entry.mode.to_string());

                quote::quote! {
                    #id, #mode
                }
            })
            .collect::<Vec<_>>();
        let for_each_working_mode =
            crate::generate_for_each_macro("working_mode", &[("all", &branches)]);
        Some(quote::quote! {
            #for_each_working_mode
        })
    }
}
