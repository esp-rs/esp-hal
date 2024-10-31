# Migration Guide from 0.10.x to v0.11.x

## Initialization changes

`EspWifiInitFor` has been removed, individual drivers such as `Wifi` and `BleConnector` handle the initialization and de-initialization of the radio stack.

`EspWifiInit` has been removed in favour of `EspWifiController`, each radio driver takes reference to this object. If no driver is borrowing `EspWifiController`,
you can safely call `EspWifiController::deinit()` to completely deinit the stack and return the peripherals used in `esp_wifi::init`.

`esp-wifi::init` now takes all peripherals using the `PeripheralRef` pattern, with the exception of the rng source.

`esp_wifi::init` now accepts `esp_hal::rng::Rng` or `esp_hal::rng::Trng`.

The following error enum variants have been removed from `InitializationError`:

- `Timer(hal::timer::Error)`
- `TimerUnavailable`
- `RadioClockUnavailable`

## No need to include `rom_functions.x` manually

Don't include `rom_functions.x` from esp-wifi

```diff
rustflags = [
    "-C", "link-arg=-Tlinkall.x",
-    "-C", "link-arg=-Trom_functions.x",
]
```

## ESP-NOW: Use `data` to access the received payload

Previously `data` and `len` were public - use the previously already existing `data()` function.

Accessing `data` or `len` was never encouraged.
