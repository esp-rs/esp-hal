pub mod registry;
pub mod server;

/// Default value for `packages` fields when deserializing from JSON.
/// Matches the Clap `default_values_t = Package::iter()` behavior.
pub fn default_packages() -> Vec<crate::Package> {
    use strum::IntoEnumIterator;
    crate::Package::iter().collect()
}

/// Default value for `chips` fields when deserializing from JSON.
/// Matches the Clap `default_values_t = Chip::iter()` behavior.
pub fn default_chips() -> Vec<esp_metadata::Chip> {
    use strum::IntoEnumIterator;
    esp_metadata::Chip::iter().collect()
}

/// Default value for `ExamplesArgs::package` when deserializing from JSON.
/// Matches the Clap `default_value_t = Package::Examples` behavior.
pub fn default_package_examples() -> crate::Package {
    crate::Package::Examples
}

/// Default value for `TestsArgs::repeat` when deserializing from JSON.
/// Matches the Clap `default_value_t = 1` behavior.
pub fn default_repeat() -> usize {
    1
}
