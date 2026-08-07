//! ESP32-H2 register data for TOP-domain regDMA retention and CPU-domain
//! software retention.

// References (ESP-IDF `v5.4`): `soc/esp32h2/system_retention_periph.c`,
// `esp_hw_support/.../esp32h2/sleep_clock.c`, `.../esp32h2/sleep_cpu.c`.

use super::{PeriphOp, Region, SysOp};

/// The TOP-domain SYS_PERIPH retention program, in retention-priority order
/// (system clock first). Interpreted by `retention::sys_periph::build_link`.
pub(super) const OPS: &[SysOp] = &[
    // PRI_0: system clock/reset. The bus-clock update bit has to be pulsed on
    // restore for the restored clock config to take effect.
    continuous!(PCR, [uart(0).conf()]..=[pwdet_sar_clk_conf()]),
    continuous!(PCR, [reset_event_bypass()], 1),
    write_reg!(PCR, [bus_clk_update()], 0x1, 0x1),
    wait_reg!(PCR, [bus_clk_update()], 0, 0x1), // wait for it to self-clear
    // PRI_2: unlock TEE/APM before restoring them.
    write_reg!(TEE, [m_mode_ctrl(4)], 0, 0xFFFF_FFFF),
    // PRI_4/5: TEE/APM, interrupt matrix, HP system.
    continuous!(HP_APM, [region_filter_en()]..=[clock_gate()]),
    continuous!(TEE, [m_mode_ctrl(0)]..=[clock_gate()]),
    continuous!(INTERRUPT_CORE0, [core_0_intr_map(0)]..=[clock_gate()]),
    continuous!(
        HP_SYS,
        [external_device_encrypt_decrypt_control()]..=[mem_test_conf()]
    ),
    // PRI_5: console UART0.
    uart_seq!(UART0),
    // PRI_6: IO MUX + GPIO matrix.
    continuous!(IO_MUX, [pin_ctrl()]..=[gpio(27)]),
    continuous!(GPIO, [func_out_sel_cfg(0)], 32),
    continuous!(GPIO, [status_next()]..=[func_in_sel_cfg(124)]),
    continuous!(GPIO, [bt_select()]..=[pin(31)]),
    continuous_split!(GPIO, [enable()] => [enable_w1ts()], 1),
    // PRI_6: flash SPI mem, SPIMEM1 (SPI1) before SPIMEM0 (SPI0). The MMU
    // content/index registers are intentionally excluded,
    // which is why each controller's last window starts at MMU_POWER_CTRL.
    continuous!(SPI1, [cmd()]..=[spi_smem_ddr()]),
    continuous!(SPI1, [spi_fmem_pms_attr(0)]..=[spi_smem_ac()]),
    continuous!(SPI1, [clock_gate()], 1),
    continuous!(SPI1, [mmu_power_ctrl()]..=[date()]),
    continuous!(SPI0, [cmd()]..=[spi_smem_ddr()]),
    continuous!(SPI0, [spi_fmem_pms_attr(0)]..=[spi_smem_ac()]),
    continuous!(SPI0, [clock_gate()], 1),
    continuous!(SPI0, [mmu_power_ctrl()]..=[date()]),
    // PRI_6: SysTimer.
    systimer_seq!(SYSTIMER),
];

// UART: ESP-IDF v5.4 `uart_periph.c` `UART_SLEEP_RETENTION_ENTRIES`, `uart_reg.h`.
pub(super) const UART_OPS: &[PeriphOp] = &[
    periph_continuous!(UART0, [int_ena()]),
    periph_continuous!(UART0, [clkdiv()]..=[rx_filt()]),
    periph_continuous!(UART0, [conf0()]..=[conf1()]),
    periph_continuous!(UART0, [hwfc_conf()]..=[tout_conf()]),
    periph_continuous!(UART0, [id()]),
    // Restore-only: pulse REG_UPDATE to latch the shadow (`_SYNC`) registers.
    periph_write!(UART0, [reg_update()], 1 << 0, 1 << 0),
    periph_wait!(UART0, [reg_update()], 0, 1 << 0),
];

// I2C: ESP-IDF v5.4 `i2c_periph.c` `i2c0_regs_retention`, `i2c_reg.h`.
pub(super) const I2C_OPS: &[PeriphOp] = &[
    periph_continuous!(I2C0, [scl_low_period()]..=[ctr()]),
    periph_continuous!(I2C0, [to()]..=[slave_addr()]),
    periph_continuous!(I2C0, [fifo_conf()]),
    periph_continuous!(I2C0, [int_ena()]),
    periph_continuous!(I2C0, [sda_hold()]..=[sda_sample()]),
    periph_continuous!(I2C0, [scl_start_hold()]..=[clk_conf()]),
    periph_continuous!(I2C0, [scl_st_time_out()]..=[scl_stretch_conf()]),
    // Restore-only: pulse FSM reset, request a config update, wait for it to latch.
    periph_write!(I2C0, [ctr()], 1 << 10, 1 << 10), // I2C_FSM_RST
    periph_write!(I2C0, [ctr()], 0, 1 << 10),
    periph_write!(I2C0, [ctr()], 1 << 11, 1 << 11), // I2C_CONF_UPGATE
    periph_wait!(I2C0, [ctr()], 0, 1 << 11),
];

// GPSPI2: ESP-IDF v5.4 `spi_periph.c` `spi2_regs_retention`, `spi_reg.h`. The
// config registers are only reachable while the SPI function clock runs;
// `Spi::with_retention_memory` holds that clock across the retention lifetime.
pub(super) const SPI_OPS: &[PeriphOp] = &[
    periph_continuous!(SPI2, [cmd()]..=[misc()]),
    periph_continuous!(SPI2, [dma_conf()]..=[dma_int_ena()]),
    periph_continuous!(SPI2, [slave()]),
];

// Interrupt matrix priorities.
pub(crate) const INTPRI_REGIONS: [Region; 2] = [
    region!(INTPRI, [cpu_int_enable()]..=[rnd_eco_low()]),
    region!(INTPRI, [rnd_eco_high()]),
];

// L1 cache control.
pub(crate) const CACHE_REGIONS: [Region; 2] = [
    region!(CACHE, [l1_cache_ctrl()]),
    region!(CACHE, [l1_cache_wrap_around_ctrl()]),
];

// PLIC machine/user interrupt controllers.
pub(crate) const PLIC_REGIONS: [Region; 4] = [
    region!(PLIC_MX, [mxint_enable()]..=[mxint_claim()]),
    region!(PLIC_MX, [mxint_conf()]),
    region!(PLIC_UX, [uxint_enable()]..=[uxint_claim()]),
    region!(PLIC_UX, [uxint_conf()]),
];

// CLINT machine/user timers. The comparators are 64-bit, which `span!` accounts
// for.
pub(crate) const CLINT_REGIONS: [Region; 2] = [
    region!(CLINT, [msip()]..=[mtimecmp()]),
    region!(CLINT, [usip()]..=[utimecmp()]),
];
