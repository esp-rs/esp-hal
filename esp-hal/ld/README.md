# Linker scripts

esp-hal provides `hal-defaults.x`: runtime defaults (interrupt handlers, riscv-rt symbols, the
`__esp_bootloader_init` hook) and the PAC's `device.x`. `fragments/` holds the parts of the
memory layout that depend on esp-hal's configuration and features. They are templates: `#IF` /
`${...}` are evaluated against the chip's esp-metadata symbols, esp-hal's configuration
(`ESP_HAL_CONFIG_*`) and esp-hal's cargo features (`CARGO_FEATURE(...)`).

The memory layout belongs to the bootloader support crate, see
`esp-bootloader-esp-idf/ld/README.md`.
