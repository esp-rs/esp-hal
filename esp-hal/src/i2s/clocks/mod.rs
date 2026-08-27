//! Clock tree node implementations for the I2S peripherals.
//!
//! The I2S driver is available only with the `unstable` feature, but the clock tree contains the
//! I2S nodes in every build. `lib.rs` therefore includes this module directly, instead of through
//! `i2s/mod.rs`.
//!
//! Each version module implements the nodes of one register layout:
//!
//! - `v1`: a single `MCLK` node in the I2S register block, shared by both directions.
//! - `v2v3`: separate `TX_CLK` and `RX_CLK` nodes, and the `MCLK_OUT` selector, in the I2S register
//!   block.
//! - `v2v3_pcr`: the same nodes, in the PCR register block.
//! - `v2v3_esp32p4`: the same nodes, in the HP_SYS_CLKRST register block.
//! - `v2v3_esp32s31`: the same nodes, in the HP_SYS_CLKRST register block, but with a per-instance
//!   register layout.

#[cfg_attr(i2s_version = "1", path = "v1.rs")]
#[cfg_attr(i2s_clock_configured_by_pcr, path = "v2v3_pcr.rs")]
#[cfg_attr(
    all(i2s_clock_configured_by_hp_sys_clkrst, esp32p4),
    path = "v2v3_esp32p4.rs"
)]
#[cfg_attr(
    all(i2s_clock_configured_by_hp_sys_clkrst, esp32s31),
    path = "v2v3_esp32s31.rs"
)]
#[cfg_attr(
    all(
        not(i2s_version = "1"),
        not(i2s_clock_configured_by_pcr),
        not(i2s_clock_configured_by_hp_sys_clkrst)
    ),
    path = "v2v3.rs"
)]
mod version;

/// The fractional part of an MCLK divider, in the X/Y/Z/YN1 encoding of the hardware.
///
/// The clock tree describes the divider as `div_num + div_b / div_a`. This type converts the
/// `div_b / div_a` fraction, as `i2s_ll_tx_set_mclk` does in ESP-IDF.
#[cfg(not(i2s_version = "1"))]
struct MclkFraction {
    x: u16,
    y: u16,
    z: u16,
    yn1: bool,
}

#[cfg(not(i2s_version = "1"))]
impl MclkFraction {
    fn new(div_a: u32, div_b: u32) -> Self {
        let (x, y, z, yn1) = if div_a == 0 || div_b == 0 {
            // The divider is an integer one.
            (0, 0, 0, true)
        } else if div_b > div_a / 2 {
            // Fractions over one half are encoded as the complement.
            let z = div_a - div_b;
            (div_a / z - 1, div_a % z, z, true)
        } else {
            (div_a / div_b - 1, div_a % div_b, div_b, false)
        };

        Self {
            x: x as u16,
            y: y as u16,
            z: z as u16,
            yn1,
        }
    }
}
