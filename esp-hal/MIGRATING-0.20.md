# Migration Guide from 0.20.x to vNext

## Cargo Features

A number of trait implementations which were previously feature-gated are now implemented by default. The following Cargo features have been removed:

- `async`
- `embedded-hal-02`
- `embedded-hal`
- `embedded-io`
- `embedded-io-async`
- `ufmt`

If your project enables any of these features, simply remove them from your Cargo manifest and things should continue to work as expected.

## HAL Initialisation

Instead of manually grabbing peripherals and setting up clocks, you should now call `esp_hal::init`.

```diff
 use esp_hal::{
-    clock::ClockControl,
-    peripherals::Peripherals,
     prelude::*,
-    system::SystemControl,
 };

 #[entry]
 fn main() -> ! {
-    let peripherals = Peripherals::take();
-    let system = SystemControl::new(peripherals.SYSTEM);
-    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
+    let peripherals = esp_hal::init(esp_hal::Config::default());

     // ...
 }
```

## GPIO changes

 - The `GpioN` type aliasses are no longer available. You can use `GpioPin<N>` instead.
 - The `AnyInputOnlyPin` has been removed. Replace any use with `AnyPin`.
 - The `NoPinType` has been removed. You can use `DummyPin` in its place.

### Type-erased GPIO drivers

The `AnyInput`, `AnyOutput`, `AnyOutputOpenDrain` and `AnyFlex` structures have been removed.
Instead, you can use the non-`Any` counterparts. By default, the structures will use `AnyPin` in
their type names, which you don't have to specify.

```diff
-let pin = AnyInput::new(io.gpio0, Pull::Up);
+let pin = Input::new(io.gpio0, Pull::Up); // pin will have the type `Input<'some>` (or `Input<'some, AnyPin>` if you want to be explicit about it)
```

You can use `new_typed` if you want to keep using the typed form:

```rust
let pin = Input::new_typed(io.gpio0, Pull::Up); // pin will have the type `Input<'some, GpioPin<0>>`
```

### Creating an `AnyPin`

Instead of `AnyPin::new`, you can now use the `Pin` trait's `degrade` method to obtain an `AnyPin`.
You can now create an `AnyPin` out of input only pins (like ESP32's IO34), but trying to use such
pins as output (or, generally, trying to use pins in ways they don't support) will panic.

```diff
+ use esp_hal::gpio::Pin;

-let pin = AnyPin::new(io.gpio0);
+let pin = io.gpio0.degrade();
```

### Wakeup using pin drivers

You can now use pin drivers as wakeup sources.

```rust
use esp_hal::peripheral::Peripheral; // needed for `into_ref`

let pin2 = Input::new(io.pins.gpio2, Pull::None);
let mut pin3 = Input::new(io.pins.gpio3, Pull::None);
// ... use pin2 as an input ..

// Ext0
let ext0 = Ext0WakeupSource::new(pin2, WakeupLevel::High);

// Ext1
let mut wakeup_pins: [&mut dyn RtcPin; 2] = [
    &mut *pin_0.into_ref(),
    &mut io.pins.gpio4, // unconfigured pins continue to work, too!
];
let ext1 = Ext1WakeupSource::new(&mut wakeup_pins, WakeupLevel::High);
```

## RX/TX Order

Previously, our API was pretty inconsistent with the RX/TX ordering, and different peripherals had different order. Now, all
the peripherals use rx-tx. Make sure your methods are expecting the right RX/TX order, for example an SPI DMA app should be updated to:

```diff
- let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) = dma_buffers!(4);
+ let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4);
let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();
let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

...

 let transfer = spi
-    .dma_transfer(dma_tx_buf, dma_rx_buf)
+    .dma_transfer(dma_rx_buf, dma_tx_buf)
    .map_err(|e| e.0)
    .unwrap();
```

When using the asymmetric variant of the macro to create DMA buffers and descriptors make sure to swap the order of parameters

```diff
- let (tx_buffer, tx_descriptors, _, _) = dma_buffers!(32000, 0);
+ let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, 32000);
```

## Removed constructors

### UART

The `Uart::new_with_default_pins` and `Uart::new_async_with_default_pins` constructors
have been removed. Use `new` or `new_async` instead.

### I2S1

The `I2s::new_i2s1` constructor has been removed. Use `I2s::new` instead.

## Timer changes

### `ErasedTimer` rename

The `ErasedTimer` has been renamed to `AnyTimer`.

### `esp_hal::time::current_time` rename

To avoid confusion with the `Rtc::current_time` wall clock time APIs, we've renamed `esp_hal::time::current_time` to `esp_hal::time::now()`.

### RTC Wall Clock APIs

Instead of the `get_time_ms`, `get_time_us`, and `get_time_raw` functions, the `Rtc` struct now provides the `current_time` function, using `chrono`'s `NaiveDateTime` struct.

```diff
let rtc = Rtc::new(peripherals.LPWR);
- let current_time_ms = rtc.get_time_ms();
+ let current_time_ms = rtc.current_time().and_utc().timestamp_millis(); // assuming UTC
```

## PCNT input config

The `PcntSource` and `PcntInputConfig` have been removed. You can use `Input` or `Flex` instead to
configure an input pin, and pass it to `set_edge_signal` or `set_ctrl_signal`.

```diff
-   let mut pin_a = io.pins.gpio4;
-   ch0.set_ctrl_signal(PcntSource::from_pin(
-       &mut pin_a,
-       PcntInputConfig { pull: Pull::Up },
-   ));
+   ch0.set_ctrl_signal(Input::new(io.pins.gpio4, Pull::Up));

-   let mut pin_b = io.pins.gpio5;
-   ch0.set_edge_signal(PcntSource::from_pin(
-       &mut pin_b,
-       PcntInputConfig { pull: Pull::Down },
-   ));
+   ch0.set_edge_signal(Input::new(io.pins.gpio5, Pull::Down));
```

## SPI pins and `NO_PIN`

Use `NoPin` in place of the now-removed `NO_PIN` constant.

SPI pins, when using the `with_pin` function, are no longer optional.
You can pass `NoPin` or `Level` if you don't need a particular pin.

```diff
 let spi = Spi::new(peripherals.SPI2, 100.kHz(), SpiMode::Mode0)
-    .with_pins(Some(sclk), Some(mosi), NO_PIN, NO_PIN);
+    .with_pins(sclk, mosi, Level::Low, NoPin);
```

## I8080 type definition

The I8080 driver no longer holds on to pins in its type definition.

```diff
- let _: I8080<'a, DmaChannel3, TxEightBits<AnyPin, AnyPin, AnyPin, ....>, Blocking>;
+ let _: I8080<'a, DmaChannel3, Blocking>;
```

## I8080 transfer API changes

- The I8080 driver now decides bus width at transfer time, which means you don't get type inference anymore.
- Starting a transfer moves the driver into the transfer object, allowing you to store it in a `static` or struct.
- The transfer API no longer works with plain slices, it now works with `DmaTxBuffer`s which allow to bring your own DMA data structure and implement efficient queueing of transfers.
- The three transfer methods (`send`, `send_dma`, `send_dma_async`) have been merged into one `send` method.

```diff
let (_, _, tx_buffer, tx_descriptors) = dma_buffers!(0, 32678);
+ let mut dma_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

let mut i8080 = I8080::new(
    lcd_cam.lcd,
    channel.tx,
-   tx_descriptors,
    tx_pins,
    20.MHz(),
    Config::default(),
)

- i8080.send(0x12, 0, &[0, 1, 2, 3, 4]);
+ dma_buf.fill(&[0, 1, 2, 3, 4]);
+ let transfer = i8080.send(0x12u8, 0, dma_buf).unwrap();
+ // transfer.wait_for_done().await;
+ (_, i8080, dma_buf) = transfer.wait();
```

### Placing drivers in RAM is now done via esp-config

We've replaced some usage of features with [esp-config](https://docs.rs/esp-config). Please remove any reference to `place-spi-driver-in-ram` in your `Cargo.toml` and migrate to the `[env]` section of `.cargo/config.toml`.

```diff
# feature in Cargo.toml
- esp-hal = { version = "0.20", features = ["place-spi-driver-in-ram"] }
# key in .cargo/config.toml [env] section
+ ESP_HAL_PLACE_SPI_DRIVER_IN_RAM=true
```

## `Camera` driver now uses `DmaRxBuffer` and moves the driver into the transfer object.

For one shot transfers.
```diff
let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(32678, 0);
+ let dma_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();

let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
let mut camera = Camera::new(
     lcd_cam.cam,
     channel.rx,
-    rx_descriptors,
     data_pins,
     20u32.MHz(),
);

- let transfer = camera.read_dma(rx_buffer).unwrap();
- transfer.wait();
+ let transfer = camera.receive(dma_buf).unwrap();
+ let (_, camera, buf) = transfer.wait();
```

For circular transfers.
```diff
- let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(32678, 0);
+ let dma_buf = dma_rx_stream_buffer!(32678);

let lcd_cam = LcdCam::new(peripherals.LCD_CAM);
let mut camera = Camera::new(
     lcd_cam.cam,
     channel.rx,
-    rx_descriptors,
     data_pins,
     20u32.MHz(),
);

- let mut transfer = camera.read_dma_circular(rx_buffer).unwrap();
+ let mut transfer = camera.receive(dma_buf).unwrap();
transfer.pop(&mut [.....]);
transfer.pop(&mut [.....]);
transfer.pop(&mut [.....]);
```

## PS-RAM

Initializing PS-RAM now takes a chip specific config and returns start of the mapped memory and the size.

Example
```rust
let (start, size) = psram::init_psram(peripherals.PSRAM, psram::PsramConfig::default());
```

If you don't specify the size of PS-RAM via `PsramConfig::size` the size of PS-RAM is derived from the RAM-chip id (or via probing in case of ESP32).

`psram::psram_vaddr_start()` and `psram::PSRAM_BYTES` are removed.

The features `psram-Xm` and `opsram-Xm` are removed and replaced by `quad-psram`/`octal-psram`.
The feature `psram-80mhz` is removed and replaced by `PsramConfig`

Diff of the `psram_quad.rs` example
```diff
-//% FEATURES: psram-2m
+//% FEATURES: esp-hal/quad-psram

...

-fn init_psram_heap() {
+fn init_psram_heap(start: *mut u8, size: usize) {
     unsafe {
         esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
-            psram::psram_vaddr_start() as *mut u8,
-            psram::PSRAM_BYTES,
+            start,
+            size,
             esp_alloc::MemoryCapability::External.into(),
         ));
     }

...

-    psram::init_psram(peripherals.PSRAM);
-    init_psram_heap();
+    let (start, size) = psram::init_psram(peripherals.PSRAM, psram::PsramConfig::default());
+    init_psram_heap(start, size);

...

```

## eFuse

Calling `Efuse::read_field_le::<bool>()` no longer compiles. Use `Efuse::read_bit()` instead.

## DMA

The DMA channel types have been removed from peripherals.

A non-exhausitve list demonstrating this change:

```diff
-I2sTx<'static, I2S0, DmaChannel0, Async>
+I2sTx<'static, I2S0, Async>

-SpiDma<'static, esp_hal::peripherals::SPI2, DmaChannel0, HalfDuplexMode, Blocking>
+SpiDma<'static, esp_hal::peripherals::SPI2, HalfDuplexMode, Blocking>
```

The type parameters of `Channel` have been reordered. The channel type
has been moved to the last position. For example:

```diff
-Channel<'d, DmaChannel0, Async>
+Channel<'d, Async, DmaChannel0>
```

You can now call `dma_channel.degrade()` to obtain a type-erased version of `Channel`. You
don't have to spell out `Channel<'d, Mode, AnyDmaChannel>`, you can use `Channel<'d, Mode>` instead.
Note that on ESP32 and ESP32-S2, passing this channel to an incompatible peripheral (for example an
I2S-specific DMA channel to SPI) will result in a panic.
