# Migration Guide from 0.22.x to v1.0.0-beta.0

## DMA changes

### Configuration changes

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

### Usability changes affecting applications

Individual channels are no longer wrapped in `Channel`, but they implement the `DmaChannel` trait.
This means that if you want to split them into an `rx` and a `tx` half (which is only supported on
the H2, C6 and S3 currently), you can't move out of the channel but instead you need to call
the `split` method.

```diff
-let tx = channel.tx;
+use esp_hal::dma::DmaChannel;
+let (rx, tx) = channel.split();
```

The `Channel` types remain available for use in peripheral drivers.

It is now simpler to work with DMA channels in generic contexts. esp-hal now provides convenience
traits and type aliasses to specify peripheral compatibility. The `ChannelCreator` types have been
removed, further simplifying use.

For example, previously you may have needed to write something like this to accept a DMA channel
in a generic function:

```rust
fn new_foo<'d, T>(
    dma_channel: ChannelCreator<2>, // It wasn't possible to accept a generic ChannelCreator.
    peripheral: impl Peripheral<P = T> + 'd,
)
where
    T: SomePeripheralInstance,
    ChannelCreator<2>: DmaChannelConvert<<T as DmaEligible>::Dma>,
{
    let dma_channel = dma_channel.configure_for_async(false, DmaPriority::Priority0);

    let driver = PeripheralDriver::new(peripheral, config).with_dma(dma_channel);

    // ...
}
```

From now on a similar, but more flexible implementation may look like:

```rust
fn new_foo<'d, T, CH>(
    dma_channel: impl Peripheral<P = CH> + 'd,
    peripheral: impl Peripheral<P = T> + 'd,
)
where
    T: SomePeripheralInstance,
    CH: DmaChannelFor<T>,
{
    // Optionally: dma_channel.set_priority(DmaPriority::Priority2);

    let driver = PeripheralDriver::new(peripheral, config).with_dma(dma_channel);

    // ...
}
```

### Usability changes affecting third party peripheral drivers

If you are writing a driver and need to store a channel in a structure, you can use one of the
`ChannelFor` type aliasses.

```diff
 struct Aes<'d> {
-    channel: ChannelTx<'d, Blocking, <AES as DmaEligible>::Dma>,
+    channel: ChannelTx<'d, Blocking, PeripheralTxChannel<AES>>,
 }
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

### TIMG

Timer group timers have been type erased.

```diff
- timg::Timer<timg::Timer0<crate::peripherals::TIMG0>, Blocking>
+ timg::Timer
```
