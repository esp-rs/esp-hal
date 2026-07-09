//! ESP32-C6 register data for TOP-domain regDMA retention and CPU-domain
//! software retention.
//!
//! Pure data consumed by the chip-agnostic logic in `retention` and
//! `cpu_retention`. Region sizes note the sizing end
//! register (`count = ((end - base) / 4) + 1`).

// References (ESP-IDF `v5.4`): `soc/esp32c6/system_retention_periph.c`,
// `esp_hw_support/.../esp32c6/sleep_clock.c`, `.../esp32c6/sleep_cpu.c`.

use super::SysOp::{self, Continuous, ContinuousSplit, Systimer, Uart, Write};

/// The TOP-domain SYS_PERIPH retention program, in retention-priority order
/// (system clock first). Interpreted by `retention::sys_periph::build_link`.
pub(super) const OPS: &[SysOp] = &[
    // PRI_0: system clock/reset (PCR).
    Continuous {
        base: 0x6009_6000,
        count: 79,
    }, // PCR ..= PCR_SRAM_POWER_CONF_REG (+0x138)
    Continuous {
        base: 0x6009_6FF0,
        count: 1,
    }, // PCR_RESET_EVENT_BYPASS_REG
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
        count: 81,
    }, // INTMTX ..= INTMTX_CORE0_CLOCK_GATE_REG (+0x140)
    Continuous {
        base: 0x6009_5000,
        count: 18,
    }, // HP_SYSTEM ..= HP_SYSTEM_MEM_TEST_CONF_REG (+0x44)
    // PRI_5: console UART0.
    Uart { base: 0x6000_0000 },
    // PRI_6: IO MUX + GPIO matrix.
    Continuous {
        base: 0x6009_0000,
        count: 32,
    }, // IO_MUX ..= IO_MUX_GPIO30_REG (+0x7c)
    Continuous {
        base: 0x6009_1554,
        count: 35,
    }, // GPIO_FUNC0_OUT_SEL ..= GPIO_FUNC34_OUT_SEL
    Continuous {
        base: 0x6009_114C,
        count: 127,
    }, // GPIO_STATUS_NEXT ..= GPIO_FUNC124_IN_SEL
    Continuous {
        base: 0x6009_1000,
        count: 64,
    }, // GPIO ..= GPIO_PIN34_REG (+0xfc)
    ContinuousSplit {
        backup: 0x6009_1020,
        restore: 0x6009_1024,
        count: 1,
    }, // GPIO_ENABLE_REG .. GPIO_ENABLE_W1TS_REG
    ContinuousSplit {
        backup: 0x6009_102c,
        restore: 0x6009_1030,
        count: 1,
    }, // GPIO_ENABLE1_REG .. GPIO_ENABLE1_W1TS_REG (pins 32..=34)
    // PRI_6: Flash SPI mem (SPIMEM1 then SPIMEM0). MMU content/index registers
    // are intentionally excluded (see ESP-IDF note).
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
    // PRI_6: SysTimer.
    Systimer { base: 0x6000_A000 },
];

/// The C6's PAU survives the `TOP` power-down (it is in `HP_AON`), so the PMU
/// drives the regDMA backup/restore in hardware on the sleep/wake transition.
pub(super) const SW_TRIGGER_REGDMA: bool = false;

// CPU-domain device-register bases lost when `pd_cpu` powers down (consumed by
// `cpu_retention`; the region layout around them is shared).
pub(crate) const INTPRI_BASE: u32 = 0x600C_5000; // interrupt priority (INTPRI)
pub(crate) const CACHE_BASE: u32 = 0x600C_8000; // L1 cache control (EXTMEM/CACHE)
pub(crate) const PLIC_MX_BASE: u32 = 0x2000_1000; // PLIC machine interrupts
pub(crate) const PLIC_UX_BASE: u32 = 0x2000_1400; // PLIC user interrupts
pub(crate) const CLINT_MINT_BASE: u32 = 0x2000_1800; // CLINT machine timer
pub(crate) const CLINT_UINT_BASE: u32 = 0x2000_1C00; // CLINT user timer
