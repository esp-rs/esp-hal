#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;

//%if option("ble-trouble")
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;
//%endif

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

//%if option("defmt")
//%if !option("probe-rs")
//+use esp_println as _;
//%endif
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

//%if option("ble-trouble")
const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + ATT
//%endif

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: {{ generate_version }}
    // generator parameters: {{ generate_parameters }}

    //%if option("probe-rs")
    //%if option("defmt")
    rtt_target::rtt_init_defmt!();
    //%else
    rtt_target::rtt_init_print!();
    //%endif
    //%else if option("log")
    esp_println::logger::init_logger_from_env();
    //%endif

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    {{ reserved_gpio_code }}

    //%if option("alloc")
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: {{ str(chip.dram2_uninit_size) }});
    //%if option("wifi") && option("ble-trouble")
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);
    //%endif
    //%endif alloc

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    //%if option("defmt") || option("log")
    info!("Embassy initialized!");
    //%else if option("probe-rs")
    rprintln!("Embassy initialized!");
    //%endif

    //%if option("wifi")
    let _wifi_controller =
        esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    let _wifi_interface = esp_radio::wifi::Interface::station();
    //%endif
    //%if option("ble-trouble")
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 1>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let _stack = trouble_host::new(ble_controller, &mut resources).build();
    //%endif

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        //%if option("defmt") || option("log")
        info!("Hello world!");
        //%else if option("probe-rs")
        rprintln!("Hello world!");
        //%endif
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/main/examples
    //%if template_origin == "clone"
    // (this project uses esp-hal commit {{ template_commit }}; the examples at that commit match it exactly)
    //%endif
}
