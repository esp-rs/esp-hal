//! This example utilises the UHCI peripheral to use UART with DMA, using the UhciSimple
//! implementation

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock, dma::{DmaRxBuf, DmaTxBuf}, dma_buffers, main, rom::software_reset, uart::{RxConfig, Uart}
};
use esp_println::println;
use esp_hal::dma::DmaRxBuffer;
use esp_hal::dma::DmaTxBuffer;
use esp_hal::uart;
use esp_hal::uart::uhci;
use esp_hal::uart::uhci::Uhci;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

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

    let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0);
    uhci.apply_config(&uhci::Config::default().with_chunk_limit(dma_rx.len() as u16))
        .unwrap();

    // Change uart config after uhci consumed it
    let config = uart::Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(64))
        .with_baudrate(9600);
    uhci.set_uart_config(&config).unwrap();

    println!("Waiting for message");
    let transfer = uhci.read(dma_rx).unwrap();
    let (uhci, dma_rx) = transfer.wait();
    let dma_rx: DmaRxBuf = DmaRxBuffer::from_view(dma_rx);

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
                let (_uhci, dma_tx) = transfer.wait().unwrap();
                let _dma_tx: DmaTxBuf = DmaTxBuffer::from_view(dma_tx);
                // Do what you want...
            }
            Err(x) => println!("Error string: {}", x),
        }
    }
    software_reset()
}
