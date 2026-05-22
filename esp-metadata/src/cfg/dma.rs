use super::GenericProperty;

/// A single GDMA channel definition.
#[derive(Debug, Clone, serde::Deserialize)]
pub struct DmaChannelDef {
    /// The name of the peripheral singleton for this channel.
    pub name: String,
    /// Whether this channel supports memory-to-memory transfers.
    #[serde(default)]
    pub mem2mem: bool,
}

/// The list of GDMA channels for a DMA engine.
#[derive(Debug, Default, Clone, serde::Deserialize)]
#[serde(transparent)]
pub struct DmaChannels(pub Vec<DmaChannelDef>);

impl GenericProperty for DmaChannels {
    fn cfgs(&self) -> Option<Vec<String>> {
        let cfgs = self
            .0
            .iter()
            .map(|ch| format!("soc_has_{}", ch.name.to_lowercase()))
            .collect();
        Some(cfgs)
    }
}
