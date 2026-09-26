#[cfg_attr(esp32, path = "phy_clocks_ll_esp32.rs")]
#[cfg_attr(esp32c2, path = "phy_clocks_ll_esp32c2.rs")]
#[cfg_attr(esp32c3, path = "phy_clocks_ll_esp32c3.rs")]
#[cfg_attr(esp32c5, path = "phy_clocks_ll_esp32c5.rs")]
#[cfg_attr(esp32c6, path = "phy_clocks_ll_esp32c6.rs")]
#[cfg_attr(esp32c61, path = "phy_clocks_ll_esp32c61.rs")]
#[cfg_attr(esp32h2, path = "phy_clocks_ll_esp32h2.rs")]
#[cfg_attr(esp32s2, path = "phy_clocks_ll_esp32s2.rs")]
#[cfg_attr(esp32s3, path = "phy_clocks_ll_esp32s3.rs")]
#[cfg_attr(esp32s31, path = "phy_clocks_ll_esp32s31.rs")]
mod phy_clocks_ll;

pub(crate) fn enable_phy(enable: bool) {
    debug!("enable_phy({:?})", enable);
    phy_clocks_ll::enable_phy(enable);
}

/// Runs `f` with the modem clocks that the PHY initialization and wake-up sequences access.
///
/// Without them, `phy_wakeup_init` on the ESP32-C61 stalls on every register access for about
/// 11 ms. The previous state of the clocks is restored afterwards, because Wi-Fi and BLE own some
/// of them.
#[cfg(any(esp32c5, esp32c6, esp32c61))]
pub(crate) fn with_calibration_clocks<R>(f: impl FnOnce() -> R) -> R {
    use phy_clocks_ll::{CALIBRATION_CLK_CONF_MASK, CALIBRATION_CLK_CONF1_MASK};

    let syscon = regs!(MODEM_SYSCON);
    let clk_conf = syscon.clk_conf().read().bits() & CALIBRATION_CLK_CONF_MASK;
    let clk_conf1 = syscon.clk_conf1().read().bits() & CALIBRATION_CLK_CONF1_MASK;

    syscon
        .clk_conf()
        .modify(|r, w| unsafe { w.bits(r.bits() | CALIBRATION_CLK_CONF_MASK) });
    syscon
        .clk_conf1()
        .modify(|r, w| unsafe { w.bits(r.bits() | CALIBRATION_CLK_CONF1_MASK) });

    let result = f();

    syscon
        .clk_conf()
        .modify(|r, w| unsafe { w.bits(r.bits() & !CALIBRATION_CLK_CONF_MASK | clk_conf) });
    syscon
        .clk_conf1()
        .modify(|r, w| unsafe { w.bits(r.bits() & !CALIBRATION_CLK_CONF1_MASK | clk_conf1) });

    result
}

/// Runs `f`. On this chip, other code already keeps the clocks running that the PHY accesses.
#[cfg(not(any(esp32c5, esp32c6, esp32c61)))]
pub(crate) fn with_calibration_clocks<R>(f: impl FnOnce() -> R) -> R {
    f()
}
