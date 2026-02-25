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

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, strum::Display)]
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
        let mut cfgs = vec![];

        let features = [
            (WorkingMode::FiniteFieldDivision, "finite_field_division"),
            (WorkingMode::ModularAddition, "modular_arithmetic"),
            (WorkingMode::ModularSubtraction, "modular_arithmetic"),
            (WorkingMode::ModularMultiplication, "modular_arithmetic"),
            (WorkingMode::ModularDivision, "modular_arithmetic"),
            (WorkingMode::AffinePointAddition, "point_addition"),
        ];

        for feature in features {
            if self.working_modes.iter().any(|mode| mode.mode == feature.0) {
                let cfg = format!("ecc_has_{}", feature.1);
                if !cfgs.contains(&cfg) {
                    cfgs.push(cfg);
                }
            }
        }

        Some(cfgs)
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
            crate::generate_for_each_macro("ecc_working_mode", &[("all", &branches)]);
        Some(quote::quote! {
            #for_each_working_mode
        })
    }
}
