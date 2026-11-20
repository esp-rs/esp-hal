//! Receives and echoes one packet using the C6 SLC0 interrupt and async API.
//! See the shared SDIO README for the reference board, wiring and companion firmware.

//% CHIP_FILTER: sd_slave_driver_supported

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    Async,
    sdio_slave::{Descriptor, FunctionState, SdioSlave},
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart},
};
use esp_println::println;
use static_cell::StaticCell;

const PACKET_LENGTH: usize = 64;

esp_bootloader_esp_idf::esp_app_desc!();

static DESCRIPTOR: StaticCell<Descriptor> = StaticCell::new();
static BUFFER: StaticCell<[u32; 128]> = StaticCell::new();

async fn exchange(slave: &mut SdioSlave<'_, Async>) -> Result<(), &'static str> {
    slave.enable_bus().map_err(|_| "bus enable failed")?;
    slave
        .arm_receive(DESCRIPTOR.init(Descriptor::new()), BUFFER.init([0; 128]))
        .map_err(|error| {
            println!("Receive arm failed: {}", error);
            "receive arm failed"
        })?;
    with_timeout(Duration::from_secs(10), async {
        while slave.function1() != FunctionState::EnabledByHost {
            Timer::after_millis(10).await;
        }
    })
    .await
    .map_err(|_| "host enable timed out")?;
    slave
        .set_function1_ready()
        .map_err(|_| "receive readiness failed")?;
    let length = with_timeout(Duration::from_secs(10), slave.wait_received())
        .await
        .map_err(|_| "receive timed out")?
        .map_err(|error| {
            println!("Receive failed: {}", error);
            "receive failed"
        })?;
    let expected = core::array::from_fn::<_, PACKET_LENGTH, _>(|i| i as u8);
    if length != PACKET_LENGTH || slave.received() != Some(expected.as_slice()) {
        return Err("unexpected request length or payload");
    }
    println!("C6 received and checked {} bytes", length);
    // Return CPU ownership, then hand the same allocations to the outbound DMA.
    let (descriptor, buffer) = slave.finish_receive().map_err(|_| "receive stop failed")?;
    slave
        .arm_transmit(descriptor, buffer, length)
        .map_err(|error| {
            println!("Transmit arm failed: {}", error);
            "transmit arm failed"
        })?;
    slave
        .set_function1_ready()
        .map_err(|_| "transmit readiness failed")?;
    let sent = with_timeout(Duration::from_secs(10), slave.wait_transmitted())
        .await
        .map_err(|_| "transmit timed out")?
        .map_err(|error| {
            println!("Transmit failed: {}", error);
            "transmit failed"
        })?;
    slave
        .finish_transmit()
        .map_err(|_| "transmit stop failed")?;
    println!(
        "C6 completed {} outbound DMA bytes; check the P4 payload result",
        sent
    );
    Ok(())
}

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timers = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timers.timer0);
    let _console = Uart::new(
        peripherals.UART0,
        UartConfig::default().with_baudrate(115_200),
    )
    .unwrap()
    .with_tx(peripherals.GPIO16);
    let mut slave = SdioSlave::new(peripherals.HINF, peripherals.SLC, peripherals.SLCHOST)
        .into_async()
        .with_cmd(peripherals.GPIO18)
        .with_clk(peripherals.GPIO19)
        .with_data0(peripherals.GPIO20)
        .with_data1(peripherals.GPIO21)
        .with_data2(peripherals.GPIO22)
        .with_data3(peripherals.GPIO23);
    println!("C6 SDIO echo ready to arm");
    if let Err(error) = exchange(&mut slave).await {
        println!("SDIO exchange failed: {}", error);
        // Dropping a timed-out wait only masks notifications. Stop DMA explicitly.
        if let Err(error) = slave.disable_bus() {
            println!("DMA shutdown unconfirmed: {}; storage retained", error);
        }
    }
    // Keep pads routed after normal DMA completion so the card can finish bus signalling.
    loop {
        Timer::after_secs(1).await;
    }
}
