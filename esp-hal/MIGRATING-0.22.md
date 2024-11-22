# Migration Guide from 0.22.x to v1.0.0-beta.0

## DMA configuration changes

- `configure_for_async` and `configure` have been removed
- PDMA devices (ESP32, ESP32-S2) provide no configurability
- GDMA devices provide `set_priority` to change DMA in/out channel priority

```diff
 let mut spi = Spi::new_with_config(
     peripherals.SPI2,
     Config::default(),
 )
 // other setup
-.with_dma(dma_channel.configure(false, DmaPriority::Priority0));
+.with_dma(dma_channel);
```

```diff
+dma_channel.set_priority(DmaPriority::Priority1);
 let mut spi = Spi::new_with_config(
     peripherals.SPI2,
     Config::default(),
 )
 // other setup
-.with_dma(dma_channel.configure(false, DmaPriority::Priority1));
+.with_dma(dma_channel);
```

## Timer changes

The low level timers, `SystemTimer` and `TimerGroup` are now "dumb". They contain no logic for operating modes or trait implementations (except the low level `Timer` trait).

### SystemTimer

```diff
let systimer = SystemTimer::new(peripherals.SYSTIMER);
- static UNIT0: StaticCell<SpecificUnit<'static, 0>> = StaticCell::new();
- let unit0 = UNIT0.init(systimer.unit0);
- let frozen_unit = FrozenUnit::new(unit0);
- let alarm0 = Alarm::new(systimer.comparator0, &frozen_unit);
- alarm0.set_period(1u32.secs());
+ let alarm0 = systimer.alarm0;
+ let mut timer = PeriodicTimer::new(alarm0);
+ timer.start(1u64.secs());
```
