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
+    esp_hal_embassy::init(timg0.timer0);

     // ...
 }
```

You have to specify a timer instance (that may be a `TimerGroup` timer unit
or a `SystemTimer` alarm) or an array of `AnyTimer`s when calling `init`.
An example of how you can set multiple timers (for example when using
multiple executors):

```rust
use esp_hal::{
    prelude::*,
    timer::{
        AnyTimer,
        systimer::SystemTimer
    }
};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    let timer1: AnyTimer = timg0.timer1.into();

    // You can use either a TimerGroup timer, a SystemTimer alarm,
    // or you can mix and match them as well.
    let systimer = SystemTimer::new(peripherals.SYSTIMER).split::<Target>();
    let timer2: AnyTimer = systimer.alarm0;

    esp_hal_embassy::init([timer0, timer1, timer2]);

    // ...
}
```

Note that you only have to convert into `AnyTimer` if you want to use multiple timers.
