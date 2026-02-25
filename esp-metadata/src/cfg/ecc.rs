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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum WorkingMode {
    AffinePointMultiplication,
    AffinePointVerification,
    AffinePointVerificationAndMultiplication,
    AffinePointAddition,
    JacobianPointMultiplication,
    JacobianPointVerification,
    JacobianPointVerificationAndMultiplication,
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
}
