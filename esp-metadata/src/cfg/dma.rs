use super::GenericProperty;

/// A single DMA channel definition.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct DmaChannelDef {
    /// The name of the peripheral singleton for this channel.
    pub name: String,
    /// Whether this channel supports memory-to-memory transfers.
    #[serde(default)]
    #[expect(unused)]
    pub mem2mem: bool,
    /// PAC peripheral type backing this channel's register block.
    /// When absent the singleton is virtual (GDMA channels).
    /// When present the singleton maps to the given PAC type (PDMA channels).
    #[serde(default)]
    pub pac: Option<String>,
}

/// A single DMA engine with its channels.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct DmaEngineDef {
    /// The name of the engine (e.g. `"GDMA"`, `"SPI2"`, `"I2S0"`).
    #[expect(unused)]
    pub name: String,
    /// The channels belonging to this engine.
    pub channels: Vec<DmaChannelDef>,
}

/// All DMA engines on a device.
#[derive(Debug, Default, Clone, serde::Deserialize)]
#[serde(transparent)]
pub struct DmaEngines(pub Vec<DmaEngineDef>);

impl GenericProperty for DmaEngines {
    fn cfgs(&self) -> Option<Vec<String>> {
        let cfgs = self
            .0
            .iter()
            .flat_map(|e| e.channels.iter())
            .map(|ch| format!("soc_has_{}", ch.name.to_lowercase()))
            .collect();
        Some(cfgs)
    }
}
