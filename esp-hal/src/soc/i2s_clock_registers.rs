//! I2S fractional MCLK divider register helpers for I2S v2+.

#[cfg(any(
    soc_has_clock_node_i2s_tx_clock,
    soc_has_clock_node_i2s_rx_clock
))]
pub(crate) fn fractional_mclk_registers(div_a: u32, div_b: u32) -> (u16, u16, u16, bool) {
    if div_b == 0 {
        // Integer divider: clear X and Z, set Y = 1 (TRM).
        return (0, 1, 0, false);
    }

    if div_b > div_a / 2 {
        let z = div_a.overflowing_sub(div_b).0;
        let x = div_a.overflowing_div(z).0.overflowing_sub(1).0;
        let y = div_a % z;
        (x as u16, y as u16, z as u16, true)
    } else {
        let x = div_a / div_b - 1;
        let y = div_a % div_b;
        (x as u16, y as u16, div_b as u16, false)
    }
}
