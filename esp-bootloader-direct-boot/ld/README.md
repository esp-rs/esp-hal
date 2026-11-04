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
- The RAM section images (`.rwtext`, `.data`, RTC memory, and on the ESP32-P4 `.uncached.data`)
  are listed in the copy table as `(source, destination start, destination end)`. The entry stub
  copies them before it starts the runtime, so `.data` is not copied again by riscv-rt or
  xtensa-lx-rt.
- `.text` and `.rodata` execute in place: their addresses are `IROM_BASE` / `DROM_BASE` plus their
  flash offset.

The RAM sections come before `.text` in the scripts. The linker assigns an input section to the
first output section in the script that matches it, so this way `.rwtext` claims what esp-hal's
`esp_hal_rwtext.x` places in RAM before the `.text` catch-all does.
