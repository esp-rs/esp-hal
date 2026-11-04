# Linker scripts

The memory layout depends on how the application gets started, so it is owned by the bootloader
support crate rather than by esp-hal. These scripts describe the layout the ESP-IDF 2nd-stage
bootloader expects. `build.rs` copies them into `OUT_DIR` and puts that directory on the linker
search path; the application passes `-Tlinkall.x` as usual.

- `<chip>/`: `memory.x`, `<chip>.x` and `linkall.x`, the entry point of the scripts.
- `sections/`: output sections shared by all chips.
- `riscv/`, `xtensa/`: `esp_idf_boot.x` (entry point, `.data` handling) and the parts of the shared
  sections that differ per architecture.

`esp-bootloader-direct-boot` is the other bootloader support crate: the ROM starts the
application directly, and its entry stub copies the RAM sections from flash. Both crates set
`links = "esp-bootloader"`, so Cargo rejects a dependency graph with both.

## Contract for bootloader support crates

esp-hal's `OUT_DIR` provides these files, so no script in a bootloader support crate may share a
name with them:

- `hal-defaults.x`: interrupt and runtime defaults, includes the PAC's `device.x`.
- `esp_hal_config.x`: esp-hal's configuration as symbols, plus `__stack_chk_guard` and the main stack
  size check. It defines `ESP_HAL_INSTRUCTION_CACHE_SIZE`, `ESP_HAL_DATA_CACHE_SIZE` and
  `ESP_HAL_L2_CACHE_SIZE` (bytes of RAM used as cache) and, on the ESP32,
  `ESP_HAL_BT_RESERVED_DRAM`.
- `esp_hal_rwtext.x` / `esp_hal_rwdata.x`: input sections that esp-hal's configuration places in
  RAM. They contain input section descriptions only.
- `esp_hal_rwtext_sections.x`: output sections in `RWTEXT` that esp-hal defines.

A bootloader support crate provides `linkall.x`, which must:

- `INCLUDE "esp_hal_config.x"` before `MEMORY`, `INCLUDE "hal-defaults.x"`, and include
  `esp_hal_rwtext.x` inside `.rwtext` and `esp_hal_rwdata.x` inside `.data`.
- Include `esp_hal_rwtext_sections.x` inside `SECTIONS`, after `_rwtext_start` and in front of the
  output section that holds `*(.rwtext.*)`. Derive sizes and addresses from `_rwtext_len` and from
  the crate's own sections, not from a sum of section sizes, so that new esp-hal sections need no
  change here.
- The sections in `esp_hal_rwtext_sections.x` have no `AT`. A crate that copies RAM code from flash
  must give an `AT` only to the output section in front of them and not to the sections that follow.
  The load addresses then continue from that section, and one copy range from that section to the
  end of `.rwtext` holds all RAM code. GNU ld does not continue the load addresses from an empty
  output section, so that section must contain data, for example `LONG(0)`. `ASSERT` that the
  load addresses are contiguous.
- Name the output sections `.text` (code in flash), `.rodata` (data in flash) and `.rwtext` (code
  in RAM). esp-hal's scripts can refer to them, for example in `NOCROSSREFS_TO`.
- Set `ENTRY(_esp_bootloader_entry)` and define `_esp_bootloader_entry`. Here it is an alias of the
  runtime entry (`_start` from riscv-rt, `Reset` from xtensa-lx-rt), because the 2nd-stage
  bootloader has already loaded every segment. A crate whose boot ROM only maps flash defines its
  own entry stub instead: it runs without a stack, copies the RAM sections from flash and then
  jumps to `_start` / `Reset`. On Xtensa `Reset` lives in `.rwtext`, so the copy must happen first.
- Decide who initializes `.data`: on RISC-V define `__sdata`, `__edata` and `__sidata` (all `0`
  here, so riscv-rt skips the copy); on Xtensa provide `__init_data` (returns `false` here, see
  `src/lib.rs`).
- Define the symbols esp-hal and the runtime crates read: `_stack_start`, `_stack_start_cpu0`,
  `_stack_end`, `_stack_end_cpu0`, `_trap_section_origin`, `_rwtext_start`, `_rwtext_end`,
  `_rwtext_len`, `_data_start`, `_data_end`, `_sidata`, `_bss_start`, `_bss_end`, `_noinit_end`,
  `_dram_data_start` (RISC-V), `_dram2_uninit_bss_start` / `_end`, `_rtc_fast_bss_*` /
  `_rtc_fast_persistent_*`, `_rtc_slow_bss_*` / `_rtc_slow_persistent_*` (chips with RTC slow
  memory) and `_uncached_*` (ESP32-P4).
- Place the input sections esp-hal and the runtime crates emit, including `.trap*`, `.rwtext*`,
  `.rtc_fast.*` / `.rtc_slow.*`, `.dram2_uninit*`, `.uncached.*` and `.espressif.metadata`.
- Respect the memory protection esp-hal sets up from these symbols (below).

### Memory protection

`esp_hal::init` derives protected ranges from the layout symbols, so the layout has to keep each
range homogeneous:

- PMP, on RISC-V chips other than the ESP32-C2/C3 (`ESP_HAL_CONFIG_ENABLE_PMP`, on by default). Each
  range becomes a locked TOR region. The start is rounded up and the end rounded down to the PMP
  granularity.
  - `_rwtext_start` .. `_rwtext_end`: read and execute, no write. Nothing in it may be written after
    `esp_hal::init`, so keep runtime-written data out. The scripts here keep `.trap` (trap entry and
    vector tables) in front of `_rwtext_start`, and the sections from `esp_hal_rwtext_sections.x`
    behind it.
  - `_stack_end_cpu0` .. `_stack_start_cpu0`: read and write, no execute.
  - `_data_start` .. `_noinit_end`: read and write, no execute. `.data`, `.bss` and `.noinit` must be
    contiguous, with no code in between.
- Data watchpoints:
  - `__stack_chk_guard` (defined by `esp_hal_config.x`), with `ESP_HAL_CONFIG_STACK_GUARD_MONITORING`.
  - From `_trap_section_origin`, the largest power of two not exceeding `_rwtext_len`, with
    `ESP_HAL_CONFIG_WRITE_VEC_TABLE_MONITORING` (ESP32-C2/C3/C6/H2). `_trap_section_origin` must be
    the start of `.trap`, and `_rwtext_len` the size of the code from `ORIGIN(RWTEXT)`.
- esp-riscv-rt's trap entry treats a stack pointer below `_dram_data_start` as a stack overflow, so
  set it to the DRAM address where the RAM code ends (here: the end of `.rwtext`).

Optionally, the crate defines `extern "C" fn __esp_bootloader_init()`. esp-hal calls it right before
`main`, once `.data` and `.bss` are initialized: from `hal_main` on RISC-V and at the end of
`__post_init` on Xtensa. It defaults to a no-op through `hal-defaults.x`. Use it for hardware setup
the bootloader would otherwise have done and which needs statics.
