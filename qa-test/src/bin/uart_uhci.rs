//! This example utilises the UHCI peripheral to use UART with DMA
//! temperature

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{Channel, DmaRxBuf, DmaTxBuf},
    dma_buffers,
    main,
    time::{Duration, Instant},
    uart::{AtCmdConfig, Config, RxConfig, Uart, UartRx, UartTx, uhci::UhciPer},
};
use esp_println::println;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let config = Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(64))
        .with_baudrate(115200);

    let uart = Uart::new(peripherals.UART1, config)
        .unwrap()
        .with_tx(peripherals.GPIO2)
        .with_rx(peripherals.GPIO3);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(100);
    let mut dma_rx = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let mut dma_tx = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();


    let mut uhci = UhciPer::new(uart, peripherals.UHCI0, peripherals.DMA_CH0);
    println!("Before configuring buffers");
    uhci.configure();
    println!("After configuring buffers");

    loop {
        println!("After");
        uhci.read(&mut dma_rx);
        println!("Received dma bytes: {}", dma_rx.number_of_received_bytes());

        let mut buf: [u8; 100] = [0; 100];
        let received = dma_rx.read_received_data(&mut buf);
        if received > 0 {
            let vec = buf.to_vec();
            match core::str::from_utf8(&vec) {
                Ok(x) => {
                    println!("Received DMA message: {}", x);
                    
                }
                Err(x) => println!("Error string: {}", x),
            }
        }

        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_secs(3) {}
    }
}
