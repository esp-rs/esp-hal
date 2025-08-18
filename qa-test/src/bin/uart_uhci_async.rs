//! This example utilises the UHCI peripheral to use UART with DMA
//! using the Uhci ("normal", transfer like) implementation with async using embassy

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    dma::{DmaRxBuf, DmaRxBuffer, DmaTxBuf, DmaTxBuffer},
    dma_buffers,
    peripherals::Peripherals,
    timer::timg::TimerGroup,
    uart::{self, RxConfig, Uart, uhci::Uhci},
};
use esp_println::println;

use crate::uart::uhci::Config;

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
    let config = uart::Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(64))
        .with_baudrate(115200);

    let uart = Uart::new(peripherals.UART1, config)
        .unwrap()
        .with_tx(peripherals.GPIO2)
        .with_rx(peripherals.GPIO3);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(4092);
    let dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0).into_async();
    uhci.apply_config(&Config::default().with_chunk_limit(dma_rx.len() as u16))
        .unwrap();

    // Change uart config after uhci consumed it
    let config = uart::Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(64))
        .with_baudrate(9600);
    uhci.set_uart_config(&config).unwrap();

    println!("Waiting for message");
    let transfer = uhci.read(dma_rx).unwrap();
    let (uhci, dma_rx) = transfer.wait().await.unwrap();
    let dma_rx: DmaRxBuf = <DmaRxBuf as DmaRxBuffer>::from_view(dma_rx);

    let received = dma_rx.number_of_received_bytes();
    println!("Received dma bytes: {}", received);

    let rec_slice = &dma_rx.as_slice()[0..received];
    if received > 0 {
        match core::str::from_utf8(&rec_slice) {
            Ok(x) => {
                println!("Received DMA message: \"{}\"", x);
                dma_tx.as_mut_slice()[0..received].copy_from_slice(&rec_slice);
                dma_tx.set_length(received);
                let transfer = uhci.write(dma_tx).unwrap();
                let (_uhci, dma_tx) = transfer.wait().await.unwrap();
                let _dma_tx: DmaTxBuf = <DmaTxBuf as DmaTxBuffer>::from_view(dma_tx);
                // Do what you want...
            }
            Err(x) => println!("Error string: {}", x),
        }
    }

    println!("Waiting");
    Timer::after(Duration::from_secs(10)).await;
    println!("bye");
}

#[embassy_executor::task()]
async fn run_logs() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        println!("Loop is looping!");
    }
}
