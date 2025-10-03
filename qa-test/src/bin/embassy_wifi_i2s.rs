//% FEATURES: esp-radio esp-radio/wifi esp-radio/smoltcp esp-radio/unstable esp-hal/unstable
//% CHIPS: esp32 esp32s2 esp32s3

#![no_std]
#![no_main]

extern crate alloc;

use embassy_executor::Spawner;
use embassy_net::{Runner, StackResources};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_alloc;
use esp_backtrace as _;
#[cfg(target_arch = "riscv32")]
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::{
    dma_buffers,
    i2s::master::{Channels, Config as I2sConfig, DataFormat, I2s},
    ram,
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::{
    Controller,
    wifi::{
        ClientConfig,
        Config as WifiConfig,
        WifiController,
        WifiDevice,
        WifiEvent,
        WifiStaState,
        sta_state,
    },
};
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

const SAMPLE_RATE: u32 = 16_000;
const I2S_BUFFER_SIZE: usize = 4092 * 8; // DMA ring size

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

/// Network stack task
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

/// Simple WiFi connection manager: start, connect, and reconnect on disconnect.
#[embassy_executor::task]
async fn connection_manager(
    mut controller: WifiController<'static>,
    connected_signal: &'static Signal<NoopRawMutex, bool>,
) {
    println!("📡 Starting WiFi connection manager");

    if !matches!(controller.is_started(), Ok(true)) {
        let client_config = WifiConfig::Client(
            ClientConfig::default()
                .with_ssid(SSID.into())
                .with_password(PASSWORD.into()),
        );
        controller.set_config(&client_config).unwrap();
        println!("🔄 Starting WiFi...");
        controller.start_async().await.unwrap();
        println!("✅ WiFi started");
    }

    println!("🔗 Connecting to WiFi network");
    match controller.connect_async().await {
        Ok(_) => {
            println!("✅ WiFi connected!");
            connected_signal.signal(true);
        }
        Err(e) => println!("❌ Initial WiFi connection failed: {:?}", e),
    }

    loop {
        match sta_state() {
            WifiStaState::Connected => {
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                println!("📶 WiFi connection lost - attempting reconnection");
                Timer::after(Duration::from_millis(2000)).await;
                match controller.connect_async().await {
                    Ok(_) => {
                        println!("✅ WiFi reconnected!");
                        connected_signal.signal(true);
                    }
                    Err(e) => {
                        println!("❌ WiFi reconnection failed: {:?}", e);
                        Timer::after(Duration::from_millis(5000)).await;
                    }
                }
            }
            _ => {
                println!("🔗 Reconnecting to WiFi network: {}", SSID);
                match controller.connect_async().await {
                    Ok(_) => {
                        println!("✅ WiFi connected!");
                        connected_signal.signal(true);
                    }
                    Err(e) => {
                        println!("❌ WiFi connection failed: {:?}", e);
                        Timer::after(Duration::from_millis(5000)).await;
                    }
                }
            }
        }
    }
}

/// I2S DMA drain task: waits for WiFi, then continuously drains and logs Late errors.
#[embassy_executor::task]
async fn i2s_dma_drain(
    i2s_rx: esp_hal::i2s::master::I2sRx<'static, esp_hal::Async>,
    buffer: &'static mut [u8],
    connected_signal: &'static Signal<NoopRawMutex, bool>,
) {
    // Temporary buffer for DMA pops
    static I2S_DATA_BUFFER: StaticCell<[u8; 8192]> = StaticCell::new();
    let i2s_data = I2S_DATA_BUFFER.init([0u8; 8192]);

    // Create circular DMA transaction
    println!(
        "🎛️  Creating I2S DMA circular transaction with ring size: {} bytes",
        buffer.len()
    );
    let mut transaction = match i2s_rx.read_dma_circular_async(buffer) {
        Ok(t) => t,
        Err(e) => {
            println!("❌ Failed to start I2S DMA: {:?}", e);
            return;
        }
    };

    let mut total_drained: usize = 0;
    let mut late_errors: u32 = 0;
    let start = Instant::now();

    // Start continuous draining to prevent DmaError(Late)
    println!("🧹 Starting continuous buffer drain to keep DMA synchronized...");
    while !connected_signal.signaled() {
        // Check for available data and drain it
        match transaction.pop(i2s_data).await {
            Ok(bytes) => {
                total_drained += bytes;
            }
            Err(e) => {
                if matches!(
                    e,
                    esp_hal::i2s::master::Error::DmaError(esp_hal::dma::DmaError::Late)
                ) {
                    println!(
                        "🧹 Late error during drain - (waiting connection signal) {:?}",
                        esp_hal::system::Cpu::current()
                    );
                    late_errors += 1;
                } else {
                    println!(
                        "⚠️ Error during continuous drain: {:?}  {:?}",
                        e,
                        esp_hal::system::Cpu::current()
                    );
                }
            }
        }

        // Small delay to prevent busy waiting
        Timer::after(Duration::from_millis(1)).await;
    }
    println!(
        "📊 Drained: {} bytes | Late errors: {} | uptime: {} ms",
        total_drained,
        late_errors,
        start.elapsed().as_millis()
    );
    match transaction.pop(i2s_data).await {
        Ok(bytes) => {
            println!("🧹 Final drained {} bytes total", bytes);
        }
        Err(e) => {
            if matches!(
                e,
                esp_hal::i2s::master::Error::DmaError(esp_hal::dma::DmaError::Late)
            ) {
                println!("🧹 Late error during drain -  (final drain)");
            } else {
                println!("⚠️ Error during final drain: {:?}", e);
            }
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // Enable logging from the ESP_LOG environment variable (set at build time)
    // Example: ESP_LOG=warn,esp_rtos=trace,esp_radio=info
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Provide a heap for components that allocate (esp-rtos/esp-radio, etc.)
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);

    // Preempt scheduler (WiFi)
    #[cfg(target_arch = "riscv32")]
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(
        timg0.timer0,
        #[cfg(target_arch = "riscv32")]
        sw_int.software_interrupt0,
    );

    esp_rtos::CurrentThreadHandle::get().set_priority(30);

    // I2S config (TDM Philips, 32-bit data per channel, stereo)
    #[cfg(any(feature = "esp32", feature = "esp32s2"))]
    let dma_channel = peripherals.DMA_I2S0;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
    let dma_channel = peripherals.DMA_CH0;
    let (rx_buffer, rx_descriptors, _, _) = dma_buffers!(I2S_BUFFER_SIZE, 0);

    let i2s_cfg = I2sConfig::new_tdm_philips()
        .with_sample_rate(Rate::from_hz(SAMPLE_RATE))
        .with_data_format(DataFormat::Data32Channel32)
        .with_channels(Channels::STEREO);

    let i2s = I2s::new(peripherals.I2S0, dma_channel, i2s_cfg)
        .expect("I2S init failed")
        .into_async();

    let i2s_rx = i2s
        .i2s_rx
        .with_bclk(peripherals.GPIO2) // SCK
        .with_ws(peripherals.GPIO4) // WS
        .with_din(peripherals.GPIO5) // SD
        .build(rx_descriptors);

    // WiFi + network stack
    let esp_radio_ctrl = &*mk_static!(Controller<'static>, esp_radio::init().unwrap());
    let (controller, interfaces) =
        esp_radio::wifi::new(&esp_radio_ctrl, peripherals.WIFI, Default::default()).unwrap();
    let wifi_interface = interfaces.sta;

    let config = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (_stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    static CONNECTED_SIGNAL: StaticCell<Signal<NoopRawMutex, bool>> = StaticCell::new();

    let connected_signal = &*CONNECTED_SIGNAL.init(Signal::new());

    // Tasks
    spawner.spawn(net_task(runner)).ok();
    spawner
        .spawn(connection_manager(controller, connected_signal))
        .ok();
    spawner
        .spawn(i2s_dma_drain(i2s_rx, rx_buffer, connected_signal))
        .ok();
}
