# Migration Guide from 0.22.x to v1.0.0-beta.0

## DMA changes

### Accessing channel objects

DMA channels are now available through the `Peripherals` struct, which is returned
by `esp_hal::init()`. The channels themselves have been renamed to match other peripheral singletons.

- ESP32-C2, C3, C6, H2 and S3: `channelX -> DMA_CHX`
- ESP32 and S2: `spiXchannel -> DMA_SPIX`, `i2sXchannel -> DMA_I2SX`

```diff
-let dma = Dma::new(peripherals.DMA);
-let channel = dma.channel2;
+let channel = peripherals.DMA_CH2;
```

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

## PSRAM is now initialized automatically

Calling `esp_hal::initialize` will now configure PSRAM if either the `quad-psram` or `octal-psram`
is enabled. To retrieve the address and size of the initialized external memory, use
`esp_hal::psram::psram_raw_parts`, which returns a pointer and a length.

```diff
-let peripherals = esp_hal::init(esp_hal::Config::default());
-let (start, size) = esp_hal::psram::init_psram(peripherals.PSRAM, psram_config);
+let peripherals = esp_hal::init({
+    let mut config = esp_hal::Config::default();
+    config.psram = psram_config;
+    config
+});
+let (start, size) = esp_hal::psram::psram_raw_parts(&peripherals.PSRAM, psram);
```

The usage of `esp_alloc::psram_allocator!` remains unchanged.


### embedded-hal 0.2.* is not supported anymore.

As per https://github.com/rust-embedded/embedded-hal/pull/640, our driver no longer implements traits from `embedded-hal 0.2.x`.
Analogs of all traits from the above mentioned version are available in `embedded-hal 1.x.x`

```diff
- use embedded_hal_02::can::Frame;
+ use embedded_can::Frame;
```

```diff
- use embedded_hal_02::digital::v2::OutputPin;
- use embedded_hal_02::digital::v2::ToggleableOutputPin;
+ use embedded_hal::digital::OutputPin;
+ use embedded_hal::digital::StatefulOutputPin;
```

```diff
- use embedded_hal_02::serial::{Read, Write};
+ use embedded_hal_nb::serial::{Read, Write};
```

You might also want to check the full official `embedded-hal` migration guide:
https://github.com/rust-embedded/embedded-hal/blob/master/docs/migrating-from-0.2-to-1.0.md
