# Migration Guide from 0.3.x to v0.4.x

## Initialization

You no longer have to set up clocks and pass them to `esp_hal_embassy::init`.

```diff
 use esp_hal::{
-    clock::ClockControl,
-    peripherals::Peripherals,
     prelude::*,
-    system::SystemControl,
 };

 #[esp_hal_embassy::main]
 async fn main(_spawner: Spawner) -> ! {
-    let peripherals = Peripherals::take();
-    let system = SystemControl::new(peripherals.SYSTEM);
-    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
+    let peripherals = esp_hal::init(esp_hal::Config::default());

     let timg0 = TimerGroup::new(peripherals.TIMG0);
-    esp_hal_embassy::init(&clocks, timg0);
+    esp_hal_embassy::init(timg0);

     // ...
 }
```
