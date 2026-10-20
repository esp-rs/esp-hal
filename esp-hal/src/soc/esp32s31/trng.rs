use crate::peripherals::{LP_PERI, RNG};

// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/hal/esp32s31/include/hal/rng_ll.h#L163-L178>
pub(crate) fn rng_ll_enable() {
    let lp_peri = LP_PERI::regs();
    let rng = RNG::regs();
    lp_peri
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_clk_en().set_bit());
    rng.date().modify(|_, w| w.clk_en().set_bit());
    lp_peri
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_rst_en().set_bit());
    lp_peri
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_rst_en().clear_bit());
    rng.conf().modify(|_, w| unsafe {
        w.noise_source_sel().bits(1 << 4);
        w.noise_pos_sel().bits(1 << 4);
        w.repetition_value_c().bits(0x1f);
        w.adpative_value_c().bits(0x12)
    });
    rng.debug_conf().modify(|_, w| unsafe {
        w.startup_test_limit().bits(1024);
        w.health_test_bypass().clear_bit()
    });
    rng.conf().modify(|_, w| {
        w.random_output_mode().set_bit();
        w.noise_crc_en().set_bit();
        w.sample_enable().set_bit()
    });
    rng.debug_conf()
        .modify(|_, w| w.startup_test_start().set_bit());
}

/// Enables true randomness by enabling the entropy source.
// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32s31.c#L10-L13>
pub(crate) fn ensure_randomness() {
    rng_ll_enable();
}

/// Disables true randomness.
// See <https://github.com/espressif/esp-idf/blob/8d7d8aef588/components/bootloader_support/src/bootloader_random_esp32s31.c#L15-L18>
pub(crate) fn revert_trng() {
    // `rng_ll_disable` is skipped: `Rng` keeps using the RNG.
}
