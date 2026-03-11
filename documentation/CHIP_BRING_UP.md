# Chip Support Guidelines

This guide aims to reach a chip bring-up state where basic examples build and flash successfully (even if they do nothing) and watchdogs reboot the chip.

In general, comparison and existing knowledge are key to obtaining all required data. As a first step, determine which already supported chip is most similar to the one being added. In the case of the `ESP32-C5`, the closest references were `ESP32-C6` and/or `ESP32-H2`. Even if no clearly similar chip exists — simply choose one as a reference and work based on that.

We prefer to trust `ESP-IDF` first and `TRM` second. The reason is simple: `TRM`, unfortunately, might sometimes contain inaccuracies, while `ESP-IDF` code is proven to work — operability is our main priority. As a side task, you may look for inconsistencies in the TRM and report them via GitHub issues (if you are an external contributor) or in the `Documentation` channel (if you are a team member).

## `espflash` And Test Tooling

For bring-up, you will typically need to add support in tooling, otherwise testing will be blocked.

### espflash Support

Do **not** guess which ESP-IDF revision the bootloaders/stubs come from. Instead, refer to the corresponding [`README`](https://github.com/esp-rs/espflash/blob/main/espflash/resources/README.md) in `espflash`, which documents the exact source commit and related build assumptions:

In particular, this page specifies:
- which ESP-IDF branch/commit the bootloaders were built from
- the current MMU page size assumptions (and any resulting flash size configuration requirements)
- where flasher stubs are sourced from

Also, you will need to implement and define a couple of chip-specific functions in `espflash/src/targets/<chip>.rs`. It is sufficient to use the implementation of already supported chips as a reference.

### probe-rs Support (optional)

If CI testing or debugger are in scope, the new chip must also be supported in [`probe-rs`](https://github.com/probe-rs/probe-rs).

As a reference, see the [PR](https://github.com/probe-rs/probe-rs/pull/3635), which added `ESP32-C5` support.

Adding probe-rs support requires a flash loader. You will need to use `esp-flash-loader` to add one. This project has a [guide](https://github.com/esp-rs/esp-flash-loader?tab=readme-ov-file#adding-new-chips) on how to add new chips and build a flash loader. 

Once completed, probe-rs can be used for flashing, debugging, and automated tests.

## Linker Scripts
The process usually begins with linker scripts.

### `memory.x`

First, determine the source of truth for memory mappings. For example, using `esp32c6` as a reference, you will see the following definition:
```
RAM : ORIGIN = 0x40800000 , LENGTH = 0x6E610
```

In `ESP-IDF` (preferably `master` or the `latest` release), search for `6E610` — it is unique enough to find relevant matches. The search will lead to:

- `components/bootloader/subproject/main/ld/esp32c6/bootloader.ld.in`
- `components/esp_system/ld/esp32c6/memory.ld.in`

Inside `memory.ld.in`, you will find:

```c
#if !CONFIG_SECURE_ENABLE_TEE
#define SRAM_SEG_START        (0x40800000)
#else
#define SRAM_SEG_START        (0x40800000 + CONFIG_SECURE_TEE_IRAM_SIZE + CONFIG_SECURE_TEE_DRAM_SIZE)
#define FLASH_SEG_OFFSET      (CONFIG_SECURE_TEE_IROM_SIZE + CONFIG_SECURE_TEE_DROM_SIZE)
#endif // CONFIG_SECURE_ENABLE_TEE

#define SRAM_SEG_START       0x4086E610  /* 2nd stage bootloader iram_loader_seg start address */
#define SRAM_SEG_SIZE      SRAM_SEG_END - SRAM_SEG_START
```

Here, `SRAM_SEG_START` corresponds to our `ORIGIN`, and `LENGTH` is `SRAM_SEG_END` - `SRAM_SEG_START`.
Now look up the corresponding definitions for your new chip.

As an example, in `memory.ld.in` for C5:

```c
#if !CONFIG_SECURE_ENABLE_TEE
#define SRAM_SEG_START        (0x40800000)
#else
#define SRAM_SEG_START        (0x40800000 + CONFIG_SECURE_TEE_IRAM_SIZE + CONFIG_SECURE_TEE_DRAM_SIZE)
#define FLASH_SEG_OFFSET      (CONFIG_SECURE_TEE_IROM_SIZE + CONFIG_SECURE_TEE_DROM_SIZE)
#endif // CONFIG_SECURE_ENABLE_TEE

#define SRAM_SEG_END       0x4084E5A0  /* 2nd stage bootloader iram_loader_seg start address */
#define SRAM_SEG_SIZE      SRAM_SEG_END - SRAM_SEG_START
```

This means:
- `ORIGIN = 0x40800000`
- `LENGTH = 0x4084E5A0 - 0x40800000 = 0x4E5A0`

To verify correctness, search for the `bootloader_iram_loader_seg_start == assertion`. It should match ORIGIN + LENGTH.

Next, update the `dram2_seg` length formula. You need the value of `SOC_ROM_STACK_START` from ESP-IDF. For C5, this value is `0x4085e5a0`. The final definition becomes:
```
    dram2_seg ( RW )       : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4085e5a0 - (ORIGIN(RAM) + LENGTH(RAM))
```

---

For `ROM` definitions, inspect `drom_seg (R)` and `irom_seg (R)` in `memory.ld.in`. The segment definitions and comments there are sufficient to determine the correct values for `ORIGIN` and `LENGTH`, as well as to understand why an offset of `0x20` is required. The comments in `memory.ld.in` typically explain how flash segments are mapped and why the offset is applied.

For `RTC_FAST` memory mappings, either:

- Look for the `LP RAM (RTC Memory) Layout` comment in `memory.ld.in`, or  
- Inspect the `lp_ram_seg(RW)` definition.

These provide both the origin address and length of the segment.

### `<chip>.x` and `linkall.x`

These linker files are slightly more involved.

It is recommended to copy an existing script (e.g., `esp32c6.x`) and adjust it as needed.

In `linkall.x`, include the required linker scripts as done for other chips, and `PROVIDE_ALIAS` for:

- `ROTEXT` / `RODATA`
- `RWTEXT` / `RWDATA`

Ensure these map correctly to the memory segments defined in `memory.x`.

For C5 specifically:

- `RTC_FAST` exists  
- `IRAM` / `DRAM` share the same address range  
- `IROM` / `DROM` also share the same address range  

For `<chip>.x`, refer to similar chips (`c6`, `h2`, `c2`) and copy as needed, adjusting addresses where required. In recent Espressif chips, linker structures are typically similar.

--- 

If you want to see the fully expanded linker scripts, build an example `ESP-IDF` project (`hello-world` is enough) for the same chip, then search inside the project’s build/ directory for *.ld.

Those *.ld files are the complete, final linker scripts produced by the build system and passed to the linker. Use them as the “ground truth”, and compare them to the corresponding *.ld.in template inputs to verify your #defines, memory regions, and section placement match what ESP-IDF ends up generating.

### esp-rom-sys

To complete the linker-related bring-up, handle the `esp-rom-sys` crate.

1. Copy all `.ld` files from `esp-idf/components/esp-rom/<CHIP>/ld` except files containing `.libc*` or `.newlib*`. It would be ideal to document which `ESP-IDF` commit you are specifically taking these linker scripts from. The future wireless drivers implementation for [esp-wireless-drivers-3rdparty](https://github.com/esp-rs/esp-wireless-drivers-3rdparty/tree/master) should be taken from the same commit.

2. Place them into:
   `esp-rom-sys/ld/<chip>/rom`

3. Copy `additional.ld` from some existing implementation for other chip.

4. In `ESP-IDF`, code-search for all `ROM` functions defined in `additional.ld` and update their addresses for your chip.

## esp-metadata

The next milestone is adding the new device to metadata. Create a new file (`esp-metadata/devices/<chip>.toml`) and paste the template below, filling in the placeholders:

```
# <CHIP> Device Metadata
#
# Empty [`device.driver`] tables imply `partial` support status.
#
# If you modify a driver support status, run `cargo xtask update-metadata` to
# update the table in the esp-hal README.

[device]
name = "<CHIP>"
arch = "riscv"
target = "riscv32im<a/af>c-unknown-none-elf"
cores = <CORES_NUM>
trm = "https://www.espressif.com/sites/default/files/documentation/<CHIP>_technical_reference_manual_en.pdf"

peripherals = [

]

symbols = [
# Additional peripherals defined by us (the developers):

]

# [device.soc]
  
memory_map = { ranges = [
{ name = "dram", start = <FILL_FROM_MEMORY.X>, end = <FILL_FROM_MEMORY.X> },
{ name = "dram2_uninit", start = 0, end = 1 }, # TODO
] }

  

clocks = { system_clocks = { clock_tree = [
# FIXME: these fake-definitions only exist for code to compile
    { name = "<CLK_SRC>", type = "source", output = "1", always_on = true },
] }, peripheral_clocks = { templates = [
	# templates
	
    { name = "default_clk_en_template", value = "{{control}}::regs().{{conf_register}}().modify(|_, w| w.{{clk_en_field}}().bit(enable));" },
    { name = "clk_en_template", value = "{{default_clk_en_template}}" },
    { name = "rst_template", value = "{{control}}::regs().{{conf_register}}().modify(|_, w| w.{{rst_field}}().bit(reset));" },
    # substitutions
    
    { name = "control", value = "crate::peripherals::SYSTEM" },
    { name = "conf_register", value = "{{peripheral}}_conf" },
    { name = "clk_en_field", value = "{{peripheral}}_clk_en" },
    { name = "rst_field", value = "{{peripheral}}_rst_en" },
], peripheral_clocks = [

] } }

```

Then, declare new chip in `esp-metadata-generated`: 
- Define the feature in it's Cargo.toml:
   ```
   <CHIP> = ["_device-selected"]
   ```

- Add it to the other chips in `lib.rs` of this crate.

And ***that's it***. The rest will be generated after you run `cargo xtask update-metadata`.

#### Adding first peripherals

To determine which peripherals to include, try building a simple example and observe compilation failures. For initial bring-up, enable only essential components:

- `MODEM_SYSCON` / `LPCON`
- `SYSTEM`
- `SYSTIMER`
- Interrupt core

Avoid enabling advanced peripherals (e.g., `i2c`, `spi`, LP peripherals) at this stage.

***Important note***: During initial support, it is acceptable to `cfg`-gate or temporarily disable some functionality. For example, many parts depend on DMA, and proper isolation is not yet fully implemented. This will be addressed once issue [#4901](https://github.com/esp-rs/esp-hal/issues/4901) is resolved.


## HAL
### clocks_ll

You don't need to **implement** clocks at this phase, however fake-defining them and at least using some blinding placeholders is required. Refer to the previous [C5 support PR ](https://github.com/esp-rs/esp-hal/pull/4859/changes#diff-75a5e847ec368b58a227b16ea788a0007a569040ed5793bf8cdda6f37676f0f5) as guidance.

### esp-hal/src/efuse

Open `espflash` in a workspace and run:
```bash
cargo xgenerate-efuse-fields /PATH/TO/LOCAL/esptool/
```

This generates Rust definitions for eFuse fields based on `esptool`.

Copy the generated file (without modification) to `efuse/<chip>/fields.rs`. Then implement required eFuse functions in `efuse/<chip>/mod.rs`

Use other chips as reference and verify field names and operations against ESP-IDF and `esptool`.


### Auxiliary crates

For initial bring-up, you may also need to add support in:

- `esp-backtrace`
- `esp-bootloader-esp-idf`
- `esp-sync`
- `esp-println` (optional, for a basic "Hello World")

These crates are mostly platform-independent. In many cases, adding a feature flag in `Cargo.toml` is enough. Where chip-specific adjustments are required (e.g., `esp-println`, `esp-bootloader-esp-idf`), verify values against ESP-IDF.

### Interrupts

Specify the correct RISC-V interrupt controller flavour in `[device.interrupt]` within metadata. Add necessary `cfg` gates for the new chip (refer to existing implementations).

### RTC_CNTL

Identify `SocResetReasons` for the new chip and implement the corresponding enumeration. These are typically defined in ESP-IDF under [`reset_reasons.h`](https://github.com/espressif/esp-idf/blob/release/v6.0/components/soc/esp32c5/include/soc/reset_reasons.h).

In `rtc_cntl/mod.rs`, ensure watchdog-related functionality (`feed()`, `wakeup_cause()`) is enabled for the new chip. Other parts may be `cfg`-gated if necessary (will be simplified after [#4901](https://github.com/esp-rs/esp-hal/issues/4901) is resolved).

### SOC

Add a minimal clocks implementation (placeholder + default clock value) in `soc/<chip>/clocks.rs`.

Also implement `regi2c` functions using `ESP-IDF` as reference.

## Subsequent Addition Of Peripherals
Implementing further peripheral support boils down to a relatively simple loop of actions:
- Define the driver in `metadata`:
   - Describe the peripheral via [device.<peripheral>] following the example of other chips
   - Add the corresponding entry to the clock tree (check if needed clocks are defined).
- Check the driver in `esp-hal` for the presence of `cfg`-gates that restrict your chip, or implement the missing parts of the functionality for it.
- Enable and run the relevant tests and examples.
- If it didn't work and errors were returned, fix them. 

## Troubleshooting

- **Clocks are not enabled**  
  Make sure the required peripheral clocks are enabled and the module is taken out of reset. Ensure the clock source itself is defined correctly in `metadata`

- **Registers do not match hardware (old ECO / TRM issue)**  
  If register layout or behavior does not match expectations, the `TRM` may be outdated or based on an older ECO. Cross-check against `ESP-IDF` and fix the PAC definitions if necessary.

- **Chip does not boot**  
  Verify basic system configuration. Ensure `UART_SCLK` is enabled. 

- **DMA does not work**  
  Confirm that DMA clocks are enabled and that the memory region used is accessible. Check whether `APM` prevents `DMA` from reading or writing memory.

- **A huge number of errors**
  Sometimes this process does indeed cause dozens or even hundreds of errors, but it is worth checking again to see if most of them can be corrected with a couple of `cfg`-gates.
