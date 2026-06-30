use super::*;

/// Programs the shared module clock register (divider, phases).
pub fn set_module_clock(_source: ClockSource, div: u8) {
    // ESP32: h == period, l == high pulse, n == h; phase dout=din=180°. The
    // module clock source is fixed at PLL160M (no `clk_sel`).
    let h = div - 1;
    let l = (div / 2).saturating_sub(1);
    let n = h;
    SDHOST::regs().clk_edge_sel().write(|w| unsafe {
        w.cclkin_edge_drv_sel().bits(4); // output phase 180°
        w.cclkin_edge_sam_sel().bits(4); // input phase 180°
        w.cclkin_edge_slf_sel().bits(0); // core phase 0°
        w.ccllkin_edge_h().bits(h);
        w.ccllkin_edge_l().bits(l);
        w.ccllkin_edge_n().bits(n)
    });
}
