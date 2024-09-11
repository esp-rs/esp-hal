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

You no longer have to spell out the GPIO pin type for `Input`, `Output`, `OutputOpenDrain` or `Flex`.
However, if you want to, you can keep using their typed form!

```rust
let pin = Input::new(io.gpio0); // pin will have the type `Input<'some>` (or `Input<'some, ErasedPin>` if you want to be explicit about it)
let pin = Input::new_typed(io.gpio0); // pin will have the type `Input<'some, GpioPin<0>>`
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

## `esp_hal::time::current_time` rename

To avoid confusion with the `Rtc::current_time` wall clock time APIs, we've renamed `esp_hal::time::current_time` to `esp_hal::time::now()`.

```diff
- use esp_hal::time::current_time;
+ use esp_hal::time::now;
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

## Removed UART constructors

The `Uart::new_with_default_pins` and `Uart::new_async_with_default_pins` constructors
have been removed. Use `new` or `new_async` instead.

## RTC Wall Clock APIs

Instead of the `get_time_ms`, `get_time_us`, and `get_time_raw` functions, the `Rtc` struct now provides the `current_time` function, using `chrono`'s `NaiveDateTime` struct.

```diff
let rtc = Rtc::new(peripherals.LPWR);
- let current_time_ms = rtc.get_time_ms();
+ let current_time_ms = rtc.current_time().and_utc().timestamp_millis(); // assuming UTC
```
