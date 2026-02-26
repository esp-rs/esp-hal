use serde::{Deserialize, Serialize};

#[derive(Default, Debug, Clone, Serialize, Deserialize)]
pub struct EccDriverProperties {
    working_modes: Vec<WorkingModeEntry>,
    curves: Vec<CurveEntry>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct WorkingModeEntry {
    id: u32,
    mode: WorkingMode,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct CurveEntry {
    id: u32,
    curve: u32,
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

        for curve in self.curves.iter() {
            cfgs.push(format!("ecc_has_curve_p{}", curve.curve));
        }

        Some(cfgs)
    }

    fn property_macro_branches(&self) -> proc_macro2::TokenStream {
        let mem_block_size = self
            .curves
            .iter()
            .map(|entry| entry.curve / 8)
            .max()
            .unwrap();

        let mem_block_size = crate::number(mem_block_size);

        quote::quote! {
            ("ecc.mem_block_size") => {
                #mem_block_size
            };
        }
    }

    fn macros(&self) -> Option<proc_macro2::TokenStream> {
        let working_mode_branches = self
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

        let curve_branches = self
            .curves
            .iter()
            .map(|entry| {
                let name = quote::format_ident!("P{}", entry.curve);
                let id = crate::number(entry.id);
                let curve = crate::number(entry.curve);
                quote::quote! {
                    #id, #name, #curve
                }
            })
            .collect::<Vec<_>>();

        let for_each_working_mode =
            crate::generate_for_each_macro("ecc_working_mode", &[("all", &working_mode_branches)]);
        let for_each_curve =
            crate::generate_for_each_macro("ecc_curve", &[("all", &curve_branches)]);
        Some(quote::quote! {
            #for_each_working_mode
            #for_each_curve
        })
    }
}
