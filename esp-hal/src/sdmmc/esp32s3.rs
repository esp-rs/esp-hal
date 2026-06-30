use super::*;

/// Programs the shared module clock register (divider, source, phases).
///
/// Field encodings differ per chip (see each chip's `sdmmc_ll`).
pub fn set_module_clock(source: ClockSource, div: u8) {
    // S3: l == period, h == high pulse, n == l; phase dout=90°, din=0°.
    let l = div - 1;
    let h = (div / 2).saturating_sub(1);
    let n = l;
    SDHOST::regs().clk_edge_sel().write(|w| unsafe {
        w.cclkin_edge_drv_sel().bits(1); // output phase 90°
        w.cclkin_edge_sam_sel().bits(0); // input phase 0°
        w.cclkin_edge_slf_sel().bits(0); // core phase 0°
        w.ccllkin_edge_h().bits(h);
        w.ccllkin_edge_l().bits(l);
        w.ccllkin_edge_n().bits(n);
        w.cclk_en().bit(matches!(source, ClockSource::Pll160m))
    });
}
