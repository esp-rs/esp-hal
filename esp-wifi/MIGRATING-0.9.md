Migration Guide from 0.9.x to vNext
====================================

Initialsation
-------------

You no longer have to set up clocks and pass them to `esp_wifi::initialize`.

```diff
 use esp_hal::{
-    clock::ClockControl,
-    peripherals::Peripherals,
     prelude::*,
-    system::SystemControl,
 };
 use esp_wifi::{
     initialize,
     // ...
 };
 
 #[entry]
 fn main() -> ! {
-    let peripherals = Peripherals::take();
-    let system = SystemControl::new(peripherals.SYSTEM);
-    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
+    let peripherals = esp_hal::init(esp_hal::Config::default());

     let timg0 = TimerGroup::new(peripherals.TIMG0);

     let init = initialize(
         EspWifiInitFor::Wifi,
         timg0.timer0,
         Rng::new(peripherals.RNG),
         peripherals.RADIO_CLK,
-        &clocks,
     )
     .unwrap();

     // ...
 }
```
