//! This example utilises the UHCI peripheral to use UART with DMA, but with async using embassy
//! uart_uhci

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf}, dma_buffers, i2s::master::{DataFormat, I2s, Standard}, time::Rate, timer::{timg::TimerGroup}, uart::{uhci::UhciPer, Config, RxConfig, Uart}, Async
};
use esp_println::println;
use esp_hal::peripherals::Peripherals;
use embassy_time::{Duration, Timer};

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    println!("Init!");
    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(unsafe { esp_hal::peripherals::TIMG0::steal() });
    esp_hal_embassy::init(timg0.timer0);

    _spawner.spawn(run_uart(peripherals)).unwrap();
    _spawner.spawn(run_logs()).unwrap();
}

#[embassy_executor::task()]
async fn run_uart(peripherals: Peripherals) {
    let config = Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(64))
        .with_baudrate(115200);

    let uart = Uart::new(peripherals.UART1, config)
        .unwrap()
        .with_tx(peripherals.GPIO2)
        .with_rx(peripherals.GPIO3).into_async();

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(25);
    let mut dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();


    let mut uhci = UhciPer::<'_, Async>::new(uart, peripherals.UHCI0, peripherals.DMA_CH0);
    uhci.configure();

    loop {
        println!("Waiting for message");
        uhci.read(&mut dma_rx).await;

        let received = dma_rx.number_of_received_bytes();
        println!("Received dma bytes: {}", dma_rx.number_of_received_bytes());

        let rec_slice = &dma_rx.as_slice()[0..received];
        if received > 0 {
            match core::str::from_utf8(&rec_slice) {
                Ok(x) => {
                    println!("Received DMA message: \"{}\"", x);
                    dma_tx.as_mut_slice()[0..received].copy_from_slice(&rec_slice);
                    uhci.write(&mut dma_tx, received).await;
                }
                Err(x) => println!("Error string: {}", x),
            }
        }

        // let delay_start = Instant::now();
        // while delay_start.elapsed() < Duration::from_secs(3) {}
    }
}

#[embassy_executor::task()]
async fn run_logs() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        println!("Loop is looping!");
    }
}

