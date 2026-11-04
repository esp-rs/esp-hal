# Direct boot hello world

A hello world that the ROM starts directly, without the ESP-IDF 2nd-stage bootloader, a partition
table or an app image header. It uses `esp-bootloader-direct-boot` instead of
`esp-bootloader-esp-idf`, and checks what direct boot is responsible for: `.data` copied from
flash, `.bss` zeroed, `#[ram]` code running, and interrupts handled.

Direct boot works on every chip except the ESP32 and the ESP32-S2.

espflash does not support the direct-boot image format yet, so `cargo xtask run`, `cargo run`,
`espflash flash` and `espflash save-image` don't work for this example. Build, convert and flash
it in separate steps instead.

## Prerequisites

- [espflash](https://github.com/esp-rs/espflash): `cargo install espflash --locked`
- `rust-objcopy`: `cargo install cargo-binutils` and `rustup component add llvm-tools`
- For the ESP32-S3, the `esp` Xtensa toolchain installed via [espup](https://github.com/esp-rs/espup)

## Build

From the repository root:

```shell
cargo xtask build direct-boot-hello-world esp32c6
```

The ELF file ends up in `target/<target>/release/direct-boot-hello-world`:

| Chip                                    | `<target>`                      |
| --------------------------------------- | ------------------------------- |
| ESP32-C2, ESP32-C3                      | `riscv32imc-unknown-none-elf`   |
| ESP32-C5, ESP32-C6, ESP32-C61, ESP32-H2 | `riscv32imac-unknown-none-elf`  |
| ESP32-H4, ESP32-P4, ESP32-S31           | `riscv32imafc-unknown-none-elf` |
| ESP32-S3                                | `xtensa-esp32s3-none-elf`       |

## Create the flash image

The image is the ELF file's loadable sections at their load addresses, which are flash offsets.
Fill the gaps with `0xff`, the value of erased flash:

```shell
rust-objcopy -O binary --gap-fill 0xff target/riscv32imac-unknown-none-elf/release/direct-boot-hello-world direct-boot-hello-world.bin
```

## Flash

When the chip ran an ESP-IDF image before, erase the flash once, so a stale bootloader or partition
table can't get in the way:

```shell
espflash erase-flash
```

Write the image to flash offset `0x0`:

```shell
espflash write-bin 0x0 direct-boot-hello-world.bin
```

On the ESP32-S3 the ROM looks for the image at `0x400`, and `objcopy` starts the image file at that
offset, so write it to `0x400` instead:

```shell
espflash write-bin 0x400 direct-boot-hello-world.bin
```

## Monitor

```shell
espflash monitor --elf target/riscv32imac-unknown-none-elf/release/direct-boot-hello-world
```

Expected output:

```text
Hello from direct boot!
[ OK ] .data copied from flash
[ OK ] .bss zeroed
[ OK ] .rwtext copied from flash
[ OK ] software interrupt handled
Bing!
```

`Bing!` repeats every 5 seconds. A reset after a few seconds means a watchdog the ROM armed is still
running.
