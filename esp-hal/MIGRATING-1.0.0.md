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
