use serde::{Deserialize, Serialize};

/// ESP module (SoM) definition.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct Module {
    pub name: String,
    pub display_name: String,
    #[serde(default)]
    pub reserved_gpios: Vec<u8>,
    #[serde(default)]
    pub octal_psram: bool,
}

/// Collection of modules for a chip.
#[derive(Debug, Default, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct Modules {
    #[serde(default)]
    pub modules: Vec<Module>,
}
