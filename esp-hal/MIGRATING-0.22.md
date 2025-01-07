# Migration Guide from 0.22.x to v1.0.0-beta.0

Starting with this release, unstable parts of esp-hal will be gated behind the `unstable` feature.
The `unstable` feature itself is unstable, we might change the way we hide APIs without notice.
Unstable APIs are not covered by semver guarantees, they may break even between patch releases.

Please refer to the documentation to see which APIs are marked as unstable.

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

### Channel configuration changes

- `configure_for_async` and `configure` have been removed
- PDMA devices (ESP32, ESP32-S2) provide no channel configurability
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

### Burst mode configuration

Burst mode is now a property of buffers, instead of DMA channels. Configuration can be done by
calling `set_burst_config` on buffers that support it. The configuration options and the
corresponding `BurstConfig` type are device specfic.

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

### Timer drivers - `OneShotTimer` & `PeriodicTimer`

Both drivers now have a `Mode` parameter. Both also type erase the underlying driver by default, call `new_typed` to retain the type.

```diff
- OneShotTimer<'static, systimer::Alarm>;
+ OneShotTimer<'static, Blocking>;
- PeriodicTimer<'static, systimer::Alarm>;
+ PeriodicTimer<'static, Blocking>;
```

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

### ETM usage has changed

Timer dependant ETM events should be created _prior_ to initializing the timer with the chosen driver.

```diff
let task = ...; // ETM task
let syst = SystemTimer::new(peripherals.SYSTIMER);
let alarm0 = syst.alarm0;
- alarm0.load_value(1u64.millis()).unwrap();
- alarm0.start();
- let event = Event::new(&mut alarm0);
+ let event = Event::new(&alarm0);
+ let timer = OneShotTimer::new(alarm0);
+ timer.schedule(1u64.millis()).unwrap();
let _configured_channel = channel0.setup(&event, &task);
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

## embedded-hal 0.2.\* is not supported anymore.

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

## Interrupt related reshuffle

```diff
- use esp_hal::InterruptConfigurable;
- use esp_hal::DEFAULT_INTERRUPT_HANDLER;
+ use esp_hal::interrupt::InterruptConfigurable;
+ use esp_hal::interrupt::DEFAULT_INTERRUPT_HANDLER;
```

## Driver constructors now take a configuration and are fallible

The old `new_with_config` constructor have been removed, and `new` constructors now always take
a configuration structure. They have also been updated to return a `ConfigError` if the configuration
is not compatible with the hardware.

```diff
-let mut spi = Spi::new_with_config(
+let mut spi = Spi::new(
     peripherals.SPI2,
     Config {
         frequency: 100.kHz(),
         mode: SpiMode::_0,
         ..Config::default()
     },
-);
+)
+.unwrap();
```

```diff
 let mut spi = Spi::new(
     peripherals.SPI2,
+    Config::default(),
-);
+)
+.unwrap();
```

## Peripheral instance type parameters and `new_typed` constructors have been removed

Call `new` instead and remove the type parameters if you've used them.

```diff
-let mut spi: Spi<'lt, SPI2> = Spi::new_typed(..).unwrap();
+let mut spi: Spi<'lt> = Spi::new(..).unwrap();
```

## LCD_CAM configuration changes

- `cam` now has a `Config` strurct that contains frequency, bit/byte order, VSync filter options.
- DPI, I8080: `frequency` has been moved into `Config`.

```diff
+let mut cam_config = cam::Config::default();
+cam_config.frequency = 1u32.MHz();
 cam::Camera::new(
     lcd_cam.cam,
     dma_rx_channel,
     pins,
-    1u32.MHz(),
+    cam_config,
 )
```

## SpiDma now requires you specify the transfer length explicitly

```diff
  dma_tx_buf.set_length(5 /* or greater */);
- spi_dma.write(dma_tx_buf);
+ spi_dma.write(5, dma_tx_buf);
```

```diff
  dma_rx_buf.set_length(5 /* or greater */);
- spi_dma.read(dma_rx_buf);
+ spi_dma.read(5, dma_rx_buf);
```

```diff
  dma_rx_buf.set_length(5 /* or greater */);
  dma_tx_buf.set_length(5 /* or greater */);
- spi_dma.transfer(dma_rx_buf, dma_tx_buf);
+ spi_dma.transfer(5, dma_rx_buf, 5, dma_tx_buf);
```

## I2C Error changes

To avoid abbreviations and contractions (as per the esp-hal guidelines), some error variants have changed

```diff
- Error::ExecIncomplete
+ Error::ExecutionIncomplete
- Error::CommandNrExceeded
+ Error::CommandNumberExceeded
- Error::ExceedingFifo
+ Error::FifoExceeded
- Error::TimeOut
+ Error::Timeout
- Error::InvalidZeroLength
+ Error::ZeroLengthInvalid
```

The `AckCheckFailed` variant changed to `AcknowledgeCheckFailed(AcknowledgeCheckFailedReason)`

```diff
-            Err(Error::AckCheckFailed)
+            Err(Error::AcknowledgeCheckFailed(reason))
```

## The crate prelude has been removed

The reexports that were previously part of the prelude are available through other paths:

- `nb` is no longer re-exported. Please import the `nb` crate if you need it.
- `ExtU64` and `RateExtU32` have been moved to `esp_hal::time`
- `Clock` and `CpuClock`: `esp_hal::clock::{Clock, CpuClock}`
- The following traits need to be individually imported when needed:
  - `esp_hal::analog::dac::Instance`
  - `esp_hal::gpio::Pin`
  - `esp_hal::ledc::channel::ChannelHW`
  - `esp_hal::ledc::channel::ChannelIFace`
  - `esp_hal::ledc::timer::TimerHW`
  - `esp_hal::ledc::timer::TimerIFace`
  - `esp_hal::timer::timg::TimerGroupInstance`
  - `esp_hal::timer::Timer`
  - `esp_hal::interrupt::InterruptConfigurable`
- The `entry` macro can be imported as `esp_hal::entry`, while other macros are found under `esp_hal::macros`

## `AtCmdConfig` now uses builder-lite pattern

```diff
- uart0.set_at_cmd(AtCmdConfig::new(None, None, None, b'#', None));
+ uart0.set_at_cmd(AtCmdConfig::default().with_cmd_char(b'#'));
```

## Crate configuration changes

To prevent ambiguity between configurations, we had to change the naming format of configuration
keys. Before, we used `{prefix}_{key}`, which meant that esp-hal and esp-hal-\* configuration keys
were impossible to tell apart. To fix this issue, we are changing the separator from one underscore
character to `_CONFIG_`. This also means that users will have to change their `config.toml`
configurations to match the new format.

```diff
 [env]
-ESP_HAL_PLACE_SPI_DRIVER_IN_RAM="true"
+ESP_HAL_CONFIG_PLACE_SPI_DRIVER_IN_RAM="true"
```

## UART changes

The `Config` struct's setters are now prefixed with `with_`. `parity_none`, `parity_even`,
`parity_odd` have been replaced by `with_parity` that takes a `Parity` parameter.

```diff
 let config = Config::default()
-    .rx_fifo_full_threshold(30)
+    .with_rx_fifo_full_threshold(30)
-    .parity_even();
+    .with_parity(Parity::Even);
```

The `DataBits`, `Parity`, and `StopBits` enum variants are no longer prefixed with the name of the enum.

e.g.

```diff
- DataBits::DataBits8
+ DataBits::_8
- Parity::ParityNone
+ Parity::None
- StopBits::Stop1
+ StopBits::_1
```

The previous blocking implementation of `read_bytes` has been removed, and the non-blocking `drain_fifo` has instead been renamed to `read_bytes` in its place.

Any code which was previously using `read_bytes` to fill a buffer in a blocking manner will now need to implement the necessary logic to block until the buffer is filled in their application instead.

The `Error` enum variant uses object+verb naming.

e.g.

```diff
- RxGlichDetected
+ GlitchOccurred
```

RX/TX pin assignment moved from constructors to builder functions.

e.g.

```diff
  let mut uart1 = Uart::new(
      peripherals.UART1,
-     Config::default(),
-     peripherals.GPIO1,
-     peripherals.GPIO2,
- ).unwrap();
+     Config::default())
+     .unwrap()
+     .with_rx(peripherals.GPIO1)
+     .with_tx(peripherals.GPIO2);
```

## Spi `with_miso` has been split

Previously, `with_miso` set up the provided pin as an input and output, which was necessary for half duplex.
Full duplex does not require this, and it also creates an artificial restriction.

If you were using half duplex SPI with `with_miso`,
you should now use `with_sio1` instead to get the previous behavior.

## CPU Clocks

The specific CPU clock variants are renamed from e.g. `Clock80MHz` to `_80MHz`.

```diff
- CpuClock::Clock80MHz
+ CpuClock::_80MHz
```

Additionally the enum is marked as non-exhaustive.

## SPI Changes

The SPI mode variants are renamed from e.g. `Mode0` to `_0`.

```diff
- Mode::Mode0
+ Mode::_0
```

The Address and Command enums have similarly had their variants changed from e.g. `Address1` to `_1Bit` and `Command1` to `_1Bit` respectively:

```diff
- Address::Address1
+ Address::_1Bit
- Command::Command1
+ Command::_1Bit
```

## GPIO Changes

The GPIO drive strength variants are renamed from e.g. `I5mA` to `_5mA`.

```diff
-DriveStrength::I5mA
+DriveStrength::_5mA
```

## ADC Changes

The ADC attenuation variants are renamed from e.g. `Attenuation0dB` to `_0dB`.

```diff
-Attenuation::Attenuation0dB
+Attenuation::_0dB
```

## `macro` module is private now

Macros from `procmacros` crate (`handler`, `ram`, `load_lp_code`) are now imported via `esp-hal`.

```diff
- use esp_hal::macros::{handler, ram, load_lp_code};
+ use esp_hal::{handler, ram, load_lp_code};
```

