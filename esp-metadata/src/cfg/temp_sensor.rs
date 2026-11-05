use super::GenericProperty;

#[derive(Debug, Default, Clone, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum TempSensorRegisterBlock {
    #[default]
    None,
    #[serde(rename = "apb_saradc")]
    ApbSarAdc,
    Sens,
    LpTsens,
}

impl GenericProperty for TempSensorRegisterBlock {
    fn cfgs(&self) -> Option<Vec<String>> {
        let block = match self {
            Self::None => return None,
            Self::ApbSarAdc => "apb_saradc",
            Self::Sens => "sens",
            Self::LpTsens => "lp_tsens",
        };
        Some(vec![format!("temp_sensor_register_block_{block}")])
    }
}
