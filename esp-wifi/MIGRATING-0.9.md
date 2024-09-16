# Migration Guide from 0.9.x to vNext

## Initialization

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

## Memory allocation

By default the `esp-alloc` feature is enabled which means you need to have a global allocator provided by `esp-alloc` allowing allocations from internal memory

```diff
 #![no_std]
 #![no_main]

+extern crate alloc;
+
 use embedded_io::*;
+use esp_alloc as _;
 use esp_backtrace as _;
 // ...

 #[entry]
 fn main() -> ! {
+    esp_alloc::heap_allocator!(72 * 1024);
+

    // ...
```

The size of the heap depends on what you are going to use esp-wifi for and if you are using the heap for your own allocations or not.

E.g. when using `coex` you need around 92k. If not using `coex`, going lower than 72k you will observe some failed allocations but it might still work. Going even lower will make things fail.

If you see linker errors regarding undefined symbols for `esp_wifi_free_internal_heap` and `esp_wifi_allocate_from_internal_ram` you either want to opt-in to use the `esp-alloc` feature 
or provide your own allocator (see below)

### Using your own allocator

You can also use your own allocator instead of using `esp-alloc`. To do that you need to opt-out of the default feature `esp-alloc` and provide two functions:

```rust
#[no_mangle]
pub extern "C" fn esp_wifi_free_internal_heap() -> usize {
    // return size of free allocatable RAM
}

#[no_mangle]
pub extern "C" fn esp_wifi_allocate_from_internal_ram(size: usize) -> *mut u8 {
    // allocate memory of size `size` from internal memory
    unsafe {
        core::alloc::alloc(
            esp_alloc::MemoryCapability::Internal.into(),
            core::alloc::Layout::from_size_align_unchecked(size, 4),
        )
    }
}
```

It's important to allocate from internal memory (i.e. not PSRAM)
