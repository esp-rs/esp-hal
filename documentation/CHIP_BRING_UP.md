
This guide aims to reach a chip bring-up state where basic examples build and flash successfully (even if they do nothing) and watchdogs reboot the chip.

In general, comparison and existing knowledge are key to obtaining all required data. As a first step, determine which already supported chip is most similar to the one being added. In the case of the `ESP32-C5`, the closest references were `ESP32-C6` and/or `ESP32-H2`. Even if no clearly similar chip exists — simply choose one as a reference and work based on that.

We prefer to trust `ESP-IDF` first and `TRM` second. The reason is simple: `TRM`, unfortunately, might sometimes contain inaccuracies, while `ESP-IDF` code is proven to work — operability is our main priority. As a side task, you may look for inconsistencies in the TRM and report them via GitHub issues (if you are an external contributor) or in the `Documentation` channel (if you are a team member).

---

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

### esp-rom-sys

To complete the linker-related bring-up, handle the `esp-rom-sys` crate.

1. Copy all `.ld` files from `esp-idf/components/esp-rom/<CHIP>/ld` except files containing `.libc*` or `.newlib*`.

2. Place them into:
   `esp-rom-sys/ld/<chip>/rom`

3. Copy `additional.ld` from some existing implementation for other chip.

4. In `ESP-IDF`, code-search for all `ROM` functions defined in `additional.ld` and update their addresses for your chip.

## esp-metadata

Next important milestone in chip bring up in adding the to-be-supported device to the metadata. A good starting point to begin with will be to create a new file in `esp-metadata/devices/<chip>.toml`, paste this template there and fill out all the placeholder values:

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

Then, declare new chip in `esp-metadata-generated` package: define feature 
```
<CHIP> = ["_device-selected"]`
```
 in it's Cargo.toml and add it to the other chips in `lib.rs` of this crate and ***that's it***. The rest will be generated after you run `cargo xtask update-metadata`.

To understand which peripherals should be added into `peripherals` list, you can try building some simple example and see which parts of the esp-hal are failing. Definitely do not enable anything advanced like `i2c` or `spi`, `LP`-peripherals, for initial bringup only essential parts like `MODEM_SYSCON/LPCON`, `SYSTEM`, `SYSTIMER`, interrupt core are necessary. 

***Important note***: for now, while adding chip support, it is fine and you will eventually have to `cfg`-out some parts of the code. For example, many parts of our code depend on `DMA`, and we have not yet fully implemented a mechanism for isolating it. This will be fixed when https://github.com/esp-rs/esp-hal/issues/4901 is closed.

### HAL
##### clocks_ll

You don't need to **implement** clocks at this phase, however fake-defining them and at least using some blinding placeholders is required. You might use our already [existing experience with adding C5 support](https://github.com/esp-rs/esp-hal/pull/4859/changes#diff-75a5e847ec368b58a227b16ea788a0007a569040ed5793bf8cdda6f37676f0f5) as a reference   
##### esp-hal/src/efuse

Open `espflash` in a workspace and use `cargo xgenerate-efuse-fields /PATH/TO/LOCAL/esptool/` to parse `efuse` fields  for your chip from `esptool` to Rust, copy the generated file (without changing it) to `efuse/<chip>/fields.rs` and implement all needed `efuse`-related functions in `efuse/<chip>/mod.rs`. Take already existing implemented functions for other chips as a reference, double check all the operations and correct field names using `esp-idf` and `esptool`. 
### Auxiliary crates

You might also need to add support for your new chip in some of `esp-hal`'s auxiliary crates for initial bring up: `esp-backtrace`, `esp-bootloader-esp-idf`, `esp-sync`, and `esp-println` as a bonus if you desire to achieve a first "Hello World". Thankfully, this crates are mostly responsible for some exact set of tasks and are reasonably platform-independent and usually adding a feature to `Cargo.toml` will be enough. Where some simple chip-specific tweaks needed (e.g. `esp-println` or `esp-bootloader-esp-idf`), please make sure to look up the correct values in `esp-idf`. 

##### Interrupts

Specifying the flavour of RISC-V interrupt controller in `[device.interrupt]` of `metadata` plus a couple of cfg-gates for the new chips (see what already exists for the others) should be enough to get the needed functionality for the bring up. 
##### RTC_CNTL

For this part of the driver, you need to look up for `SocResetReasons` and build an enumeration of them for the new chip. These are usually also enumerated in `esp-idf` in [`reset_reason.h`](https://github.com/espressif/esp-idf/blob/release/v6.0/components/soc/esp32c5/include/soc/reset_reasons.h).
In `rtc_cntl/mod.rs` make sure, that watchdog-related code is active for your new chip (`feed`(), `wakeup_cause()`). You can `cfg`-out the rest, if needed (TODO: update after https://github.com/esp-rs/esp-hal/issues/4901 is closed)

##### SOC

Make sure to add a primitive `clocks` implementation (at least with a placeholder and a default clock value) in `soc/<chip>/clocks.rs` and implement `regi2c` functions, reference is `esp-idf` correct implementation

### Troubleshooting

TO BE FINISHED
