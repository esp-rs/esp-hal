//! ESP32-H2 register data for TOP-domain regDMA retention and CPU-domain
//! software retention.
//!
//! Pure data consumed by the chip-agnostic logic in `retention` and
//! `cpu_retention`. Region sizes note the sizing end register
//! (`count = ((end - base) / 4) + 1`).

// References (ESP-IDF `v5.4`): `soc/esp32h2/system_retention_periph.c`,
// `esp_hw_support/.../esp32h2/sleep_clock.c`, `.../esp32h2/sleep_cpu.c`.

use super::SysOp::{self, Continuous, ContinuousSplit, Systimer, Uart, Wait, Write};

/// The TOP-domain SYS_PERIPH retention program, in retention-priority order
/// (system clock first). Interpreted by `retention::sys_periph::build_link`.
pub(super) const OPS: &[SysOp] = &[
    // PRI_0: system clock/reset (PCR). The H2 must also pulse the bus-clock
    // update bit on restore for the new clock config to take effect.
    Continuous {
        base: 0x6009_6000,
        count: 85,
    }, // PCR ..= PCR_PWDET_SAR_CLK_CONF_REG (+0x150)
    Continuous {
        base: 0x6009_6FF0,
        count: 1,
    }, // PCR_RESET_EVENT_BYPASS_REG
    Write {
        addr: 0x6009_6148,
        value: 0x1,
        mask: 0x1,
    }, // PCR_BUS_CLK_UPDATE (BUS_CLOCK_UPDATE)
    Wait {
        addr: 0x6009_6148,
        value: 0,
        mask: 0x1,
    }, // wait for it to self-clear
    // PRI_2: unlock TEE/APM (clear TEE_M4_MODE_CTRL) before restoring them.
    Write {
        addr: 0x6009_8010,
        value: 0,
        mask: 0xFFFF_FFFF,
    }, // TEE_M4_MODE_CTRL_REG
    // PRI_4/5: TEE/APM, interrupt matrix, HP system.
    Continuous {
        base: 0x6009_9000,
        count: 68,
    }, // HP_APM ..= HP_APM_CLOCK_GATE_REG (+0x10c)
    Continuous {
        base: 0x6009_8000,
        count: 33,
    }, // TEE ..= TEE_CLOCK_GATE_REG (+0x80)
    Continuous {
        base: 0x6001_0000,
        count: 69,
    }, // INTMTX ..= INTMTX_CORE0_CLOCK_GATE_REG (+0x110)
    Continuous {
        base: 0x6009_5000,
        count: 12,
    }, // HP_SYSTEM ..= HP_SYSTEM_MEM_TEST_CONF_REG (+0x2c)
    // PRI_5: console UART0.
    Uart { base: 0x6000_0000 },
    // PRI_6: IO MUX + GPIO matrix (fewer pins than the C6).
    Continuous {
        base: 0x6009_0000,
        count: 29,
    }, // IO_MUX ..= IO_MUX_GPIO27_REG (+0x70)
    Continuous {
        base: 0x6009_1554,
        count: 32,
    }, // GPIO_FUNC0_OUT_SEL ..= GPIO_FUNC31_OUT_SEL
    Continuous {
        base: 0x6009_114C,
        count: 127,
    }, // GPIO_STATUS_NEXT ..= GPIO_FUNC124_IN_SEL
    Continuous {
        base: 0x6009_1000,
        count: 61,
    }, // GPIO ..= GPIO_PIN31_REG (+0xf0)
    ContinuousSplit {
        backup: 0x6009_1020,
        restore: 0x6009_1024,
        count: 1,
    }, // GPIO_ENABLE_REG .. GPIO_ENABLE_W1TS_REG
    // PRI_6: Flash SPI mem (SPIMEM1 then SPIMEM0), identical layout to the C6.
    // MMU content/index registers are intentionally excluded (see ESP-IDF note).
    Continuous {
        base: 0x6000_3000,
        count: 55,
    }, // SPIMEM1 ..= SPI_MEM_SPI_SMEM_DDR (+0xd8)
    Continuous {
        base: 0x6000_3100,
        count: 41,
    }, // SPIMEM1 FMEM_PMS0_ATTR ..= SMEM_AC (+0x1a0)
    Continuous {
        base: 0x6000_3200,
        count: 1,
    }, // SPIMEM1 CLOCK_GATE
    Continuous {
        base: 0x6000_3384,
        count: 31,
    }, // SPIMEM1 MMU_POWER_CTRL ..= DATE (+0x3fc)
    Continuous {
        base: 0x6000_2000,
        count: 55,
    }, // SPIMEM0 ..= SPI_MEM_SPI_SMEM_DDR
    Continuous {
        base: 0x6000_2100,
        count: 41,
    }, // SPIMEM0 FMEM_PMS0_ATTR ..= SMEM_AC
    Continuous {
        base: 0x6000_2200,
        count: 1,
    }, // SPIMEM0 CLOCK_GATE
    Continuous {
        base: 0x6000_2384,
        count: 31,
    }, // SPIMEM0 MMU_POWER_CTRL ..= DATE
    // PRI_6: SysTimer (base differs from the C6).
    Systimer { base: 0x6000_B000 },
];

// ESP32-H2 use software to trigger REGDMA to restore instead of PMU, because regdma has power bug.
pub(super) const SW_TRIGGER_REGDMA: bool = true;

// CPU-domain device-register bases lost when `pd_cpu` powers down (consumed by
// `cpu_retention`). Identical to the C6 (same RISC-V core/cache/PLIC/CLINT).
pub(crate) const INTPRI_BASE: u32 = 0x600C_5000; // interrupt priority (INTPRI)
pub(crate) const CACHE_BASE: u32 = 0x600C_8000; // L1 cache control (CACHE)
pub(crate) const PLIC_MX_BASE: u32 = 0x2000_1000; // PLIC machine interrupts
pub(crate) const PLIC_UX_BASE: u32 = 0x2000_1400; // PLIC user interrupts
pub(crate) const CLINT_MINT_BASE: u32 = 0x2000_1800; // CLINT machine timer
pub(crate) const CLINT_UINT_BASE: u32 = 0x2000_1C00; // CLINT user timer
