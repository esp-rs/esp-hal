# Linker scripts

These scripts lay the application out for direct boot and follow the contract for bootloader
support crates in `esp-bootloader-esp-idf/ld/README.md`. `build.rs` copies them into `OUT_DIR`
and puts that directory on the linker search path; the application passes `-Tlinkall.x` as usual.
Every file except `linkall.x` starts with `direct_boot_`, so none shares a name with a file from
esp-hal's `OUT_DIR`.

- `<chip>/linkall.x`: the entry point. `direct_boot_memory.x` holds the chip's magic offset, the
  addresses the ROM maps flash at, and the memory regions.
- `riscv/`: the layout shared by the RISC-V chips. The ESP32-S3 has its own
  `esp32s3/direct_boot_layout.x`.

Flash layout, with each section's load address equal to its flash offset:

```text
magic | entry stub, copy table | RAM section images | .text | .rodata
```

- The magic is `0xaedb041d` twice, at `DIRECT_BOOT_MAGIC_OFFSET`. The ROM jumps to the
  instruction after it.
- The RAM section images (RAM code, `.data`, and on the ESP32-P4 `.uncached.data`) are listed in
  the copy table as `(source, destination start, destination end)`. The entry stub copies them
  before it starts the runtime, so `.data` is not copied again by riscv-rt or xtensa-lx-rt.
- RAM code is one copy range: `.trap` (on the ESP32-S3 `.direct_boot.rwtext_start`), the output
  sections from esp-hal's `esp_hal_rwtext_sections.x`, and `.rwtext`. Only the first section has an
  `AT`, so the others continue its load addresses, and an `ASSERT` checks that they do. On the
  ESP32-S3, `.vectors` is a separate range.
- The RTC memory images are listed in a second table of the same format. `early_init` copies them
  unless the chip woke up from deep sleep, as the ESP-IDF bootloader does, so initialized RTC
  statics keep their values over deep sleep.
- `.text` and `.rodata` execute in place: their addresses are `IROM_BASE` / `DROM_BASE` plus their
  flash offset.

RAM extends over everything the ESP-IDF 2nd-stage bootloader would occupy, the ROM's download-mode
buffers and the ROM's stack, up to the ROM's own `.bss`/`.data`. There is no 2nd-stage bootloader,
and the entry stub copies without a stack. On chips with two cores, RAM ends below the second
core's ROM stack: the ROM runs on it when esp-hal starts that core. `dram2_seg` is empty, as
`#[ram(reclaimed)]` is not available.

The RAM sections come before `.text` in the scripts. The linker assigns an input section to the
first output section in the script that matches it, so this way `.rwtext` claims what esp-hal's
`esp_hal_rwtext.x` places in RAM before the `.text` catch-all does.
