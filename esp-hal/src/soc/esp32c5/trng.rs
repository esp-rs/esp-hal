use crate::{
    peripherals::{APB_SARADC, PCR, PMU},
    soc::regi2c,
};

/// Enable true randomness by enabling the entropy source.
/// Blocks `ADC` usage.
pub(crate) fn ensure_randomness() {
    todo!();
}

/// Disable true randomness. Unlocks `ADC` peripheral.
pub(crate) fn revert_trng() {
   todo!();
}
