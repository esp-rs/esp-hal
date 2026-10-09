#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use esp_hal::clock::CpuClock;
//%if option("embassy")
//+use esp_hal::timer::timg::TimerGroup;
//+use embassy_executor::Spawner;
//+use embassy_time::{Duration, Timer};
//%else
use esp_hal::{
    main,
    time::{Duration, Instant},
};
//%endif
//%if option("ble-trouble")
//+use bt_hci::controller::ExternalController;
//+use esp_radio::ble::controller::BleConnector;
//+use trouble_host::prelude::*;
//%endif

use esp_backtrace as _;
use log::info;

//%if option("alloc")
extern crate alloc;
//%endif

//%if option("ble-trouble")
//+const CONNECTIONS_MAX: usize = 1;
//+const L2CAP_CHANNELS_MAX: usize = 1;
//%endif

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

//%if option("embassy")
//+#[esp_rtos::main]
//+async fn main(spawner: Spawner) -> ! {
//%else
#[main]
fn main() -> ! {
    //%endif
    // generator version: {{ generate_version }}
    // generator parameters: {{ generate_parameters }}

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    //%if !option("embassy")
    let _ = peripherals;
    //%endif

    //%if option("alloc")
    //+esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: {{ str(chip.dram2_uninit_size) }});
    //%if option("wifi") && option("ble-trouble")
    //+// COEX needs more RAM - so we've added some more
    //+esp_alloc::heap_allocator!(size: 64 * 1024);
    //%endif
    //%endif alloc

    //%if option("embassy")
    //+let timg0 = TimerGroup::new(peripherals.TIMG0);
    //+esp_rtos::start(timg0.timer0);
    //%endif

    //%if option("wifi")
    //+let _wifi_controller =
    //+    esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())
    //+        .expect("Failed to initialize Wi-Fi controller");
    //+let _wifi_interface = esp_radio::wifi::Interface::station();
    //%endif
    //%if option("ble-trouble")
    //+// find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    //+let transport = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    //+let ble_controller = ExternalController::<_, 1>::new(transport);
    //+let mut resources: HostResources<_, DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
    //+    HostResources::new();
    //+let _stack = trouble_host::new(ble_controller, &mut resources).build();
    //%endif

    //%if option("embassy")
    //+// TODO: Spawn some tasks
    //+let _ = spawner;
    //%endif

    loop {
        info!("Hello world!");
        //%if option("embassy")
        //+Timer::after(Duration::from_secs(1)).await;
        //%else
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
        //%endif
    }

    // for inspiration have a look at the examples in this repository
}
