
This guide aims to achieve the state of chip bringup where “basic examples are built and flashed, even if they do nothing, watchdogs reboot the chip”

Overall, comparison and already existing knowledge base is the key to get all the needed data. So as a first step, we should figure out which chip is the closest similar chip to the one being added. In my case of esp32c5, this will be esp32c6 (more) and esp32h2(less). Even if there's no obviously similar chip - it's not a big problem. We will just pick some chip for the reference and work basing off of that. 
I prefer to trust ESP-IDF over any other knowledge source (e.g. TRM) for one simple reason - we all know for a fact that TRM might lie and often does, whilst ESP-IDF code just ***works***, which is our main interest. As a side-quest, you might try to look for inconsistencies in the TRM and report them in `GitHub` issues if you're an external contributor or to `Documentation` channel if you are our team member


## Linker Scripts
The process in  usually starts from linker scripts. 

#### memory.x

Let's start from memory.x. I need to find the source from which I'll get my data for all the memory mappings. I'll pick `esp32c6` as a reference. you see this "definition" first:
```
RAM : ORIGIN = 0x40800000 , LENGTH = 0x6E610
```

In `esp-idf` (preferably `master` or latest release) lookup for `6E610`, it seems to be unique enough. Search will lead you to `components/bootloader/subproject/main/ld/esp32c6/bootloader.ld.in` and `components/esp_system/ld/esp32c6/memory.ld.in`. In `memory.ld.in` file we find these definitions: 
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

SRAM_SEG_START is our ORIGIN and LENGTH its obviously SRAM_SEG_END - SRAM_SEG_START. Now, let's lookup for definitions of these things for our chip (C5 in our case)

`memory.ld.in` says: 
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

which means our ORIGIN is the same (0x40800000), but LENGTH will be 0x4084E5A0 - ORIGIN = ***0x4E5A0***
Just to confirm everything is fine, you can check search for `bootloader_iram_loader_seg_start == ` assertion, it should be your ORIGIN + LENGTH
Next, you need to fill out a correct address in dram2_seg len "formula", there you need to set value of SOC_ROM_STACK_START from `esp-idf`, for `c5` it's 0x4085e5a0, so the final line will be:
```
    dram2_seg ( RW )       : ORIGIN = ORIGIN(RAM) + LENGTH(RAM), len = 0x4085e5a0 - (ORIGIN(RAM) + LENGTH(RAM))
```

For  `ROM` definition look for `drom_seg (R)` and `irom_seg (R)` definitions in `memory.ld.in`. Definitions of the segments and comments there are enough to figure out what are the correct values for `ORIGIN` and `LENGTH` and why we need an offset of `0x20`

For `RTC_FAST` memory mappings, you can either look for `LP RAM (RTC Memory) Layout` comment in the same file or `lp_ram_seg(RW)` segment definition, which maps both origin address and length of the memory segment

Next linker files are a bit trickier: `<chip>.x` and `linkall.x` .

it is recommended to copy-paste this script from already existing one (e.g. `esp32c6.x`). If 
In `linkall.x` you should include the other linker scripts in a way such file does for the other chips and also `PROVIDE_ALIAS` to `ROTEXT/RODATA` and `RWTEXT/RWDATA` for the memory segments you defined in `memory`. In case of C5, it has `RTC_FAST` memory and `IRAM/DRAM` (sitting on the same address) and `IROM/DROM` (also on the same address). 

For `<chip>.x`, it is highly recommended to look at similar scripts from already existing chips (`c6`/`h2`/`c2`) and copying them. Same works with `linkall.x` and `memory.x` , with further tweaks here and there, e.g. in terms of addresses. In following Espressif chips, these linker-related parts are unlikely to be very different from above mentioned chips.


#### esp-rom-sys

In order to finish linker-related part of the bring up, we need to handle `esp-rom-sys` crate. Copy `.ld` all the files from `esp-idf/components/esp-rom/<CHIP>/ld`  except the ones which contain `.libc*` and `.newlib*` in their names to the `esp-rom-sys/ld/<chip>/rom`. Copy the `additional.ld` file from `esp32c6`'s one. Then, in `esp-idf` codebase, code-search for all the defined ROM functions in `additional.ld` for your chip and update the addresses of these functions. Example of search:
![[Pasted image 20260218131822.png]]

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
