#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_hal::{
    clock::CpuClock,
    main,
    time::{Duration, Instant},
};
//%if !option("probe-rs") && (group_selected("log-frontend") || option("esp-backtrace"))
//%if chip.soc_has_usb_device
use esp_hal::{Blocking, usb::usb_serial_jtag::UsbSerialJtag};
//%else
//+use esp_hal::{
//+    Blocking,
//+    uart::{self, UartTx},
//+};
//%endif
use static_cell::StaticCell;
//%endif
//%if option("defmt")
//+use defmt::info;
//%if !group_selected("panic-handler")
//+use defmt::error;
//%endif !group_selected("panic-handler")
//%else if option("log")
use log::info;
//%if !group_selected("panic-handler")
use log::error;
//%endif !group_selected("panic-handler")
//%else if option("probe-rs")
//+use rtt_target::rprintln;
//%endif !defmt

//%if !group_selected("panic-handler")
//%if option("defmt") || option("log")
//+#[panic_handler]
//+fn panic(panic_info: &core::panic::PanicInfo) -> ! {
//+    error!("{}", panic_info);
//+    loop {}
//+}
//%else if option("probe-rs")
//+#[panic_handler]
//+fn panic(panic_info: &core::panic::PanicInfo) -> ! {
//+    rprintln!("{}", panic_info);
//+    loop {}
//+}
//%else
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
//%endif
//%else if option("esp-backtrace")
//+use esp_backtrace as _;
//%else if option("panic-rtt-target")
//+use panic_rtt_target as _;
//%endif

//%if option("alloc")
extern crate alloc;
//%endif

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: {{ generate_version }}
    // generator parameters: {{ generate_parameters }}

    //%if option("probe-rs")
    //%if chip.name == "esp32p4"
    // The debugger reads RTT from physical memory, past the CPU cache, so the control block and
    // the buffers must be in uncached memory.
    //+    let channels = rtt_target::rtt_init! {
    //+        up: {
    //+            0: {
    //+                size: 1024,
    //%if option("defmt")
    //+                name: "defmt",
    //%else
    //+                name: "Terminal",
    //%endif
    //+                section: ".uncached.data",
    //+            }
    //+        }
    //+        section_cb: ".uncached.data"
    //+    };
    //%if option("defmt")
    //+    rtt_target::set_defmt_channel(channels.up.0);
    //%else
    //+    rtt_target::set_print_channel(channels.up.0);
    //%endif
    //%else if option("defmt")
    rtt_target::rtt_init_defmt!();
    //%else
    rtt_target::rtt_init_print!();
    //%endif
    //%endif

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    //%if has_reserved_pins || (!option("probe-rs") && (group_selected("log-frontend") || option("esp-backtrace")))
    let peripherals = esp_hal::init(config);
    //%else
    //+    let _peripherals = esp_hal::init(config);
    //%endif

    //%if !option("probe-rs") && (group_selected("log-frontend") || option("esp-backtrace"))
    //%if chip.soc_has_usb_device
    // Log on USB-Serial-JTAG. To log on UART0 instead, construct an `esp_hal::uart::UartTx` on
    // the chip's default TX pin.
    //%if chip.name == "esp32c3"
    //+    let (usb_dp, usb_dm) = (peripherals.GPIO19, peripherals.GPIO18);
    //%else if chip.name == "esp32c5" || chip.name == "esp32h4"
    //+    let (usb_dp, usb_dm) = (peripherals.GPIO14, peripherals.GPIO13);
    //%else if chip.name == "esp32h2"
    //+    let (usb_dp, usb_dm) = (peripherals.GPIO27, peripherals.GPIO26);
    //%else if chip.name == "esp32p4"
    //+    let (usb_dp, usb_dm) = (peripherals.GPIO25, peripherals.GPIO24);
    //%else if chip.name == "esp32s3"
    //+    let (usb_dp, usb_dm) = (peripherals.GPIO20, peripherals.GPIO19);
    //%else if chip.name == "esp32s31"
    //+    let (usb_dp, usb_dm) = (peripherals.GPIO34, peripherals.GPIO33);
    //%else
    let (usb_dp, usb_dm) = (peripherals.GPIO13, peripherals.GPIO12);
    //%endif
    let log_writer =
        UsbSerialJtag::new(peripherals.USB_DEVICE, usb_dp, usb_dm, Default::default()).unwrap();
    static LOG_WRITER: StaticCell<UsbSerialJtag<'static, Blocking>> = StaticCell::new();
    //%else
    //+    // Log on UART0's default TX pin.
    //+    let log_writer = UartTx::new(peripherals.UART0, uart::Config::default())
    //+        .unwrap()
    //%if chip.name == "esp32"
    //+        .with_tx(peripherals.GPIO1);
    //%else if chip.name == "esp32c2"
    //+        .with_tx(peripherals.GPIO20);
    //%else
    //+        .with_tx(peripherals.GPIO43);
    //%endif
    //+    static LOG_WRITER: StaticCell<UartTx<'static, Blocking>> = StaticCell::new();
    //%endif
    embedded_log::init(LOG_WRITER.init(log_writer));
    //%endif

    {{ reserved_gpio_code }}

    //%if option("alloc")
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: {{ str(chip.dram2_uninit_size) }});
    //%endif alloc

    loop {
        //%if option("defmt") || option("log")
        info!("Hello world!");
        //%else if option("probe-rs")
        rprintln!("Hello world!");
        //%endif
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/main/examples
    //%if template_origin == "clone"
    // (this project uses esp-hal commit {{ template_commit }}; the examples at that commit match it exactly)
    //%endif
}
