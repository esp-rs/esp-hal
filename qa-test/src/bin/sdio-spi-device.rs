#![no_std]
#![no_main]

use embedded_hal_sdmmc::Common;
use esp_backtrace as _;
use esp_hal::{
    dma::{DescriptorFlagFields, Owner},
    sdio::{
        Config,
        Mode,
        Pins,
        Sdio,
        SpiMode,
        dma::{
            AtomicBuffer,
            AtomicDmaDescriptor,
            AtomicDmaDescriptors,
            DmaDescriptor,
            DmaDescriptorFlags,
        },
    },
};

esp_bootloader_esp_idf::esp_app_desc!();

// Use 10KB of data to fit inside SRAM
// One RX + one TX buffer
const DATA_SIZE: usize = 1024 * 5;
// Use 6KB of descriptors to fit inside SRAM
const DESC_SIZE: usize = 1024 * 6;
// Represents the default byte length of a SDIO block.
const BLOCK_LEN: usize = 512;
// Total number of DMA descriptors.
const DESC_NUM: usize = DESC_SIZE / core::mem::size_of::<AtomicDmaDescriptor>() / 2;
// Represents the set of Receive DMA descriptors.
static RX_DESCRIPTORS: AtomicDmaDescriptors<DESC_NUM> = AtomicDmaDescriptors::new();
// Represents the set of Transmit DMA descriptors.
static TX_DESCRIPTORS: AtomicDmaDescriptors<DESC_NUM> = AtomicDmaDescriptors::new();
// Represents the Receive DMA buffer.
static RX_BUFFER: AtomicBuffer<DATA_SIZE> = AtomicBuffer::new();
// Represents the Transmit DMA buffer.
static TX_BUFFER: AtomicBuffer<DATA_SIZE> = AtomicBuffer::new();

struct Context {
    sdio: Sdio<'static>,
}

impl Context {
    /// Creates a new context for the SDIO SPI slave controller.
    pub fn new() -> Self {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                // GPIO Slot 1 config
                let pins = Pins::new(
                    Mode::Spi,
                    peripherals.GPIO6,  // CLK/SCK
                    peripherals.GPIO11, // CMD/MOSI
                    peripherals.GPIO7,  // DAT0/MISO
                    peripherals.GPIO8,  // DAT1/IRQ
                    peripherals.GPIO9,  // DAT2
                    peripherals.GPIO10, // DAT3/#CS
                );

                // GPIO Slot 2 config
                //let pins = Pins::new(
                //    Mode::Spi,
                //    peripherals.GPIO14, // CLK/SCK
                //    peripherals.GPIO15, // CMD/MOSI
                //    peripherals.GPIO2,  // DAT0/MISO
                //    peripherals.GPIO4,  // DAT1/IRQ
                //    peripherals.GPIO12, // DAT2
                //    peripherals.GPIO13, // DAT3/#CS
                //);
            } else if #[cfg(feature = "esp32c6")] {
                let pins = Pins::new(
                    Mode::Spi,
                    peripherals.GPIO19, // CLK/SCLK
                    peripherals.GPIO18, // CMD/MOSI
                    peripherals.GPIO20, // DAT0/MISO
                    peripherals.GPIO21, // DAT1/IRQ
                    peripherals.GPIO22, // DAT2
                    peripherals.GPIO23, // DAT3/#CS
                );
            } else {
                panic!("unsupported platform");
            }
        }

        let config = Config::new().with_spi_mode(SpiMode::_2);

        let mut sdio = Sdio::new(
            peripherals.SLC,
            peripherals.SLCHOST,
            peripherals.HINF,
            pins,
            config,
        );

        sdio.init().ok();

        Self { sdio }
    }
}

#[esp_hal::main]
fn main() -> ! {
    let ctx = Context::new();

    // TODO: perform data transfer
    // indicate a transfer of a single SDIO block
    let mut flags = DmaDescriptorFlags::new();
    flags.set_owner(Owner::Dma);
    flags.set_suc_eof(true);
    flags.set_size(BLOCK_LEN);
    flags.set_len(BLOCK_LEN);

    let tx_descriptor = &TX_DESCRIPTORS[0];

    let packet = b"test RX packet";
    RX_BUFFER.write(packet.as_slice());

    let buffer = unsafe { RX_BUFFER.as_ptr_mut() };
    tx_descriptor.update(DmaDescriptor {
        flags,
        buffer,
        next: core::ptr::null_mut(),
    });

    let slc = unsafe { &*ctx.sdio.slc().register_block };

    // configure SLC RX descriptors
    slc.slc1rx_link_addr()
        .write(|w| unsafe { w.bits(RX_DESCRIPTORS.address()) });
    slc.slc1rx_link()
        .modify(|_, w| w.sdio_slc1_rxlink_start().set_bit());

    loop {
        core::hint::spin_loop()
    }
}
