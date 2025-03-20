# Migration Guide from v1.0.0-beta.0 to v1.0.0-beta.1

## GPIO pins now play nice with `assign-resources`

For compatibility with third party libraries, as well as for consistency with other peripherals,
the GPIO pin types (`GpioPin<N>`) have been replaced by separate `esp_hal::peripherals::GPION`
types.

```diff
-use esp_hal::gpio::GpioPin;
+use esp_hal::peripherals::{GPIO2, GPIO3};

-fn my_function(gpio2: GpioPin<2>, gpio3: GpioPin<3>) {...}
+fn my_function(gpio2: GPIO2, gpio3: GPIO3) {...}
```
