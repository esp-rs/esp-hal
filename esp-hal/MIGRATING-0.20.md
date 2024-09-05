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
