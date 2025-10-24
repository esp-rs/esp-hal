# Migration Guide from 1.0.0-rc.1 to {{currentVersion}}

## RMT Changes

Several details of the driver structs and their methods have changed:

- `SingleShotTxTransaction` has been renamed to `TxTransaction`.
- All `Channel` and `ChannelCreator` methods that take their arguments (channels and pins) by value now return them on failure; this might require changed to error handling. Changed methods are
    - `ChannelCreator<'_, _, _>::configure_tx` and `ChannelCreator<'_, _, _>::configure_rx`,
    - `Channel<'_, Blocking, Tx>::transmit`,
    - `Channel<'_, Blocking, Tx>::transmit_continuously`,
    - `Channel<'_, Blocking, Rx>::receive`.
- The `ChannelCreator::configure_tx` and `ChannelCreator::configure_rx` methods now take the configuration by reference.

```diff
-let mut tx = rmt.channel0.configure_tx(tx_pin, TxChannelConfig::default());
-let mut rx = rmt.channel2.configure_rx(rx_pin, RxChannelConfig::default());
+let mut tx = rmt.channel0.configure_tx(tx_pin, &TxChannelConfig::default());
+let mut rx = rmt.channel2.configure_rx(rx_pin, &RxChannelConfig::default());

-let tx_transaction: SingleShotTxTransaction<'_, PulseCode> = tx.transmit(&data)?;
+let tx_transaction: TxTransaction<'_, PulseCode> = match tx.transmit(&data) {
+    Ok(t) => t,
+    Err((e, channel)) => {
+        // Now, the `channel` is not lost permanently, but can continue to be used
+        // (e.g. to retry configuration with different parameters)
+        e?
+    }
+};
```

