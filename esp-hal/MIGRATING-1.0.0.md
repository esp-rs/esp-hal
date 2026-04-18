# Migration Guide from 1.0.0 to 1.1.0-rc.0

## RMT Changes

`ChannelCreator::configure_tx` and `ChannelCreator::configure_rx` have changed in a few ways:
- both methods now take the configuration by reference,
- the pin argument has been removed in favor of `Channel::with_pin`, which is infallible and avoids consuming the pin on error.

```diff
 let tx_config = TxChannelConfig::default();
 let rx_config = RxChannelConfig::default();

 let tx = rmt.channel0
-    .configure_tx(tx_pin, tx_config)
-    .unwrap();
+    .configure_tx(&tx_config)
+    .unwrap()
+    .with_pin(tx_pin);
 
 let rx = rmt.channel2
-    .configure_rx(rx_pin, rx_config)
-    .unwrap();
+    .configure_rx(&rx_config)
+    .unwrap()
+    .with_pin(rx_pin);
```

`SingleShotTxTransaction` has been renamed to `TxTransaction`:

```diff
-let txn: SingleShotTxTransaction<'_, '_, PulseCode> = tx_channel.transmit(&data)?;
+let txn: TxTransaction<'_, '_, PulseCode> = tx_channel.transmit(&data)?;
```

Some blocking `Channel` methods that previously consumed the channel on error now return it in all cases:

```diff
 let mut channel = todo!("set up tx channel");
 
 channel = match tx_channel.transmit(&data) {
     Ok(txn) => {
         match txn.wait() {
             Ok(c) => c,
             Err((e, c)) => {
                 // TODO: Handle the error
                 c
             }
         }
     }
-    Err(e) => {
-        // Channel cannot be re-used here (if it had the 'static lifetime)
-        panic!();
+    Err((e, c)) => {
+        // TODO: Handle the error
+        c
     }
 }
```

This applies to
- `Channel<'_, Blocking, Tx>::transmit`,
- `Channel<'_, Blocking, Tx>::transmit_continuously`,
- `Channel<'_, Blocking, Rx>::receive`.

Configuration methods
- `Rmt::new`,
- `ChannelCreator::configure_tx`,
- `ChannelCreator::configure_rx`,
now return `ConfigError` instead of `Error`.
Corresponding enum variants have been removed from `Error`, and some variants
that are now part of `ConfigError` have been renamed.

### RMT data type changes

Support for `Into<PulseCode>` and `From<PulseCode>` has been removed from Tx and Rx methods, respectively.
Instead, buffers must now contain `PulseCode` directly.
The corresponding generic argument has also been removed from `TxTransaction` and `RxTransaction`:

```diff
-let tx_data: [u32; 8] = todo!();
-let mut rx_data: [u32; 8] = [0u32; 8];
-let tx_transaction: TxTransaction<'_, '_, u32> = tx_channel.transmit(&tx_data)?;
-let rx_transaction: RxTransaction<'_, '_, u32> = rx_channel.receive(&mut rx_data)?;
+let tx_data: [PulseCode; 8] = todo!();
+let mut rx_data: [PulseCode; 8] = [PulseCode::default(); 8];
+let tx_transaction: TxTransaction<'_, '_> = tx_channel.transmit(&tx_data)?;
+let rx_transaction: RxTransaction<'_, '_> = rx_channel.receive(&mut rx_data)?;
```

## SHA Changes

The `ShaDigest` type now requires exclusive access to the SHA peripheral to prevent unsound concurrent access. If you were manually constructing `ShaDigest` instances (not using `Sha::start()` or `Sha::start_owned()`), you need to use a mutable reference:

```diff
 let mut sha = Sha::new(peripherals.SHA);
-let sha_ref = &sha;
+let sha_ref = &mut sha;
 let digest = ShaDigest::<Sha256, _>::new(sha_ref);
```

The recommended `Sha::start()` and `Sha::start_owned()` methods already require `&mut self`, so typical usage is unaffected.

## Clock changes

The `RtcClock::xtal_freq()` function and the `XtalClock` enum have been removed. The currently recommended way to access the crystal clock frequency is through the `Clocks` struct, which returns a `Rate` value:

```rust
let xtal_clock = esp_hal::clock::Clocks::get().xtal_clock;
```

## Interrupt handling changes

### Direct binding interrupt handlers

On Xtensa MCUs (ESP32, ESP32-S2, ESP32-S3) the direct binding option has been removed.

On RISC-V MCUs, the `interrupt::enable_direct` function now takes a `DirectBindableCpuInterrupt` and is infallible:

```diff
 interrupt::enable_direct(
     peripheral_interrupt,
     interrupt_priority,
-    CpuInterrupt::Interrupt25,
+    DirectBindableCpuInterrupt::Interrupt0,
     handler_function,
-).unwrap();
+);
```

`DirectBindableCpuInterrupt` is numbered from 0, and corresponds to CPU interrupt numbers that are not disabled or reserved for vectoring.

## ECC changes

- The `Ecc::new` constructor now takes a configuration parameter.
- All `Ecc` methods now expect little endian input, and produce little endian output.
- `Ecc` methods now return a result handle instead of writing data back directly. These handles can be used to retrieve the result of the operation. Modular arithmetic methods now take the modulus as an argument.

```diff
-let mut ecc = Ecc::new(peripherals.ECC);
+let mut ecc = Ecc::new(peripherals.ECC, Config::default());
-ecc.mod_operations(EllipticCurve::P256, &mut a, &mut b, WorkMode::ModAdd).unwrap();
+let result = ecc.modular_addition(EllipticCurve::P256, EccModBase::OrderOfCurve, &a, &b).unwrap();
+result.read_scalar_result(&mut a).unwrap();
```

The method that performs the operation now only returns an error if the parameters are of incorrect
length. Point verification errors are now returned by the result handle. If a particular operation
only does point verification and does not perform any other operations, the verification result
can be read using the `success` method.

## eFuse Changes

### Module structure change

The `Efuse` struct has been removed. All methods are now free-standing functions
in the `efuse` module. Some functions have been renamed:

- `Efuse::mac_address()` → `efuse::base_mac_address()`
- `Efuse::set_mac_address(mac)` → `efuse::override_mac_address(mac)`
- `Efuse::interface_mac_address(kind)` → `efuse::interface_mac_address(kind)`
- `Efuse::read_field_le(field)` → `efuse::read_field_le(field)`
- `Efuse::read_bit(field)` → `efuse::read_bit(field)`
- `Efuse::chip_revision()` → `efuse::chip_revision()`

`Efuse::read_base_mac_address()` has been removed; use `efuse::base_mac_address()` instead.
`MacAddress::as_bytes_mut()` has been removed; use `MacAddress::as_bytes()` for read access.

### Chip revision change

`chip_revision` now returns a `ChipRevision` structure. This structure allows more efficient
operations compared to the old combined u16 - it does not require integer multiplication/division.

The old `u16` revision number is equivalent to `ChipRevision`'s `combined` representation. You
can use the `from_combined` and `combined` functions to keep working with this representation. A
new, `packed` representation (also encoded as `u16`) is now available which is less computationally
expensive.

```diff
-let revision = chip_revision();
+let revision = chip_revision().combined();
```

## PSRAM configuration changes

The PSRAM configuration has been moved out of `esp_hal::Config`. Instead, PSRAM configuration is now done
when creating the `Psram` driver. If you are using the `esp_alloc::psram_allocator!` macro, a default
configuration is applied for you. However, sometimes you may want to customize this configuration.

```rust
let psram_config = esp_hal::psram::PsramConfig {
    // Set custom configuration options here
    // e.g. (ESP32-S3): `mode: esp_hal::psram::PsramMode::OctalSpi,`
    ..Default::default()
};
```

You have the following options to apply this configuration:

### Pass a `PsramConfig` struct to the macro

```rust
let p = esp_hal::init(Default::default());
esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram, psram_config);
```

### Initialize the driver explicitly and pass it to the macro

```rust
use esp_hal::psram::Psram;

let p = esp_hal::init(Default::default());
let psram = Psram::new(p.PSRAM, psram_config);
esp_alloc::psram_allocator!(&psram);
```

### Initialize the driver explicitly and set up the PSRAM heap without the macro

```rust
use esp_hal::psram::Psram;

let p = esp_hal::init(Default::default());
let psram = Psram::new(p.PSRAM, psram_config);
let (start, size) = psram.raw_parts();
unsafe {
    esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
        start,
        size,
        esp_alloc::MemoryCapability::External.into(),
    ));
}
```

## Build configuration changes

The following options have been removed:

- `ESP_HAL_CONFIG_XTAL_FREQUENCY`
- `ESP_HAL_CONFIG_PSRAM_MODE`

Instead, runtime configuration is available.

### Configuring crystal frequency

The crystal frequency can be configured when initializing `esp-hal`, by providing a custom clock configuration.

```rust
use esp_hal::clock::{CpuClock, ll};

// Obtain a configuration preset for a given CPU clock frequency.
let mut cpu_clock_config: ll::ClockConfig = CpuClock::default().into();

// Change the XTAL frequency in the configuration.
cpu_clock_config.xtal_clk = Some(ll::XtalClkConfig::_40);

// Initialize esp-hal with the custom clock configuration.
let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(cpu_clock_config));
```

### Configuring the PSRAM interface (ESP32-S3)

`esp-hal` will try to auto-detect the PSRAM mode. However, this may not work in all cases. Also, you
may already know exactly what mode your chip's PSRAM can operate in, and want to configure it directly
to save on time or code size.

You can now specify the PSRAM mode via the `mode` field of the `PsramConfig` struct. Read the "PSRAM configuration changes" section for more information on how you need to apply this configuration.
