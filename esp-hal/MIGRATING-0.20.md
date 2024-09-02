Migration Guide from 0.20.x to vNext
====================================

HAL initialsation
-----------------

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
+    let (peripherals, clocks) = esp_hal::init(esp_hal::Config::default());

     // ...
 }
```
