# Migration Guide from 1.0.0 to {{currentVersion}}

## RMT Changes

The `ChannelCreator::configure_tx` and `ChannelCreator::configure_rx` methods now take the configuration by reference:

```diff
let tx_config = TxChannelConfig::default();
let rx_config = RxChannelConfig::default();

-let mut tx = rmt.channel0.configure_tx(tx_pin, tx_config);
-let mut rx = rmt.channel2.configure_rx(rx_pin, rx_config);
+let mut tx = rmt.channel0.configure_tx(tx_pin, &tx_config);
+let mut rx = rmt.channel2.configure_rx(rx_pin, &rx_config);
```
