# Migration Guide from 1.0.0 to {{currentVersion}}

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
