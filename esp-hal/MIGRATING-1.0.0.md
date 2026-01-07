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

## Clock changes

The `RtcClock::xtal_freq()` function and the `XtalClock` enum have been removed. The currently recommended way to access the crystal clock frequency is through the `Clocks` struct, which returns a `Rate` value:

```rust
let xtal_clock = esp_hal::clock::Clocks::get().xtal_clock;
```
