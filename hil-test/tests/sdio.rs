//! SDIO Tests
//!
//! # Pin Connections
//!
//! On the ESP32-C6, there is no SDIO host implemented in hardware, so one needs to be mocked using
//! a SPI peripheral.
//!
//! The following physical pin connections are needed on a ESP32C6-DevKitC-1:
//!
//! - SPI CLK (GPIO6, pin 5) => SDIO CLK (GPIO19, pin 19)
//! - SPI CS0 (GPIO16, pin 2) => SDIO CS (GPIO23, pin 23)
//! - SPI MOSI (GPIO7, pin 6) => SDIO MOSI (GPIO18, pin 18)
//! - SPI MISO (GPIO2, pin 12) => SDIO MISO (GPIO20, pin 20)

//% CHIPS: esp32 esp32c6
//% FEATURES: unstable

#![no_std]
#![no_main]

#[cfg(feature = "esp32c6")]
use esp_hal::interrupt::{self, IsrCallback, Priority};
#[cfg(feature = "esp32c6")]
use esp_hal::peripherals::Interrupt;
use esp_hal::{
    Blocking,
    dma::{DescriptorFlagFields, Owner},
    sdio::{
        Config,
        Mode,
        Pins,
        Sdio,
        command::Cmd52,
        dma::{
            AtomicBuffer,
            AtomicDmaDescriptor,
            AtomicDmaDescriptors,
            DmaDescriptor,
            DmaDescriptorFlags,
        },
        response::spi::R5,
    },
    spi::master as spi,
    time::Rate,
};
use hil_test as _;

esp_bootloader_esp_idf::esp_app_desc!();

// Use 10KB of data to fit inside SRAM
// One RX + one TX buffer
const DATA_SIZE: usize = 1024 * 5;
// Use 6KB of descriptors to fit inside SRAM
const DESC_SIZE: usize = 1024 * 6;
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
    spi: spi::Spi<'static, Blocking>,
}

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    // Represents one block length.
    const TEST_BLOCK_LEN: usize = 512;
    // Represents one packet length with multiple blocks.
    // const TEST_PACKET_LEN: usize = 2048;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
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

                // Create SPI master for mock SDIO host
                // HSPI config
                let spi = spi::Spi::new(
                    peripherals.SPI2,
                    spi::Config::default().with_frequency(Rate::from_mhz(10)),
                )
                .unwrap()
                .with_sck(peripherals.GPIO14)
                .with_mosi(peripherals.GPIO13)
                .with_miso(peripherals.GPIO12)
                .with_cs(peripherals.GPIO15);

                // Create SPI master for mock SDIO host
                // VSPI config
                //let spi = spi::Spi::new(
                //    peripherals.SPI3,
                //    spi::Config::default().with_frequency(Rate::from_mhz(10)),
                //)
                //.unwrap()
                //.with_sck(peripherals.GPIO18)
                //.with_mosi(peripherals.GPIO23)
                //.with_miso(peripherals.GPIO19)
                //.with_cs(peripherals.GPIO5);
            } else if #[cfg(esp32c6)] {
                let pins = Pins::new(
                    Mode::Spi,
                    peripherals.GPIO19, // CLK/SCLK
                    peripherals.GPIO18, // CMD/MOSI
                    peripherals.GPIO20, // DAT0/MISO
                    peripherals.GPIO21, // DAT1/IRQ
                    peripherals.GPIO22, // DAT2
                    peripherals.GPIO23, // DAT3/#CS
                );

                // Create SPI master for mock SDIO host
                let spi = spi::Spi::new(
                    peripherals.SPI2,
                    spi::Config::default().with_frequency(Rate::from_mhz(10)),
                )
                .unwrap()
                .with_sck(peripherals.GPIO6)
                .with_mosi(peripherals.GPIO7)
                .with_miso(peripherals.GPIO2)
                .with_cs(peripherals.GPIO16);
            } else {
                panic!("unsupported platform");
            }
        }

        let config = Config::new();

        let sdio = Sdio::new(
            peripherals.SLC,
            peripherals.SLCHOST,
            peripherals.HINF,
            pins,
            config,
        );

        Context { sdio, spi }
    }

    #[test]
    fn test_dma_rx_descriptors(mut ctx: Context) {
        // indicate a transfer of a single SDIO block
        let mut flags = DmaDescriptorFlags::new();
        flags.set_owner(Owner::Dma);
        flags.set_suc_eof(true);
        flags.set_size(TEST_BLOCK_LEN);
        flags.set_len(TEST_BLOCK_LEN);

        let rx_descriptor = &RX_DESCRIPTORS[0];

        let test_buffer: [u8; TEST_BLOCK_LEN] = core::array::from_fn(|i| i as u8);
        RX_BUFFER.write(&test_buffer);

        let buffer = unsafe { RX_BUFFER.as_ptr_mut() };
        rx_descriptor.update(DmaDescriptor {
            flags,
            buffer,
            next: core::ptr::null_mut(),
        });

        // TODO: mock transfers to the SDIO host
        let mut test_rx = [0u8; TEST_BLOCK_LEN];

        ctx.spi
            .read(&mut test_rx)
            .expect("error reading from SDIO host");
        // assert_eq!(test_buffer, test_rx);
    }

    #[test]
    fn test_dma_tx_descriptors(mut ctx: Context) {
        // indicate a transfer of a single SDIO block
        let mut flags = DmaDescriptorFlags::new();
        flags.set_owner(Owner::Dma);
        flags.set_suc_eof(true);
        flags.set_size(TEST_BLOCK_LEN);
        flags.set_len(TEST_BLOCK_LEN);

        let tx_descriptor = &TX_DESCRIPTORS[0];

        let test_packet = Cmd52::new();

        let mut test_buffer: [u8; TEST_BLOCK_LEN] = [0u8; TEST_BLOCK_LEN];
        test_buffer[..Cmd52::LEN].copy_from_slice(test_packet.into_bytes().as_ref());

        let buffer = unsafe { TX_BUFFER.as_ptr_mut() };
        tx_descriptor.update(DmaDescriptor {
            flags,
            buffer,
            next: core::ptr::null_mut(),
        });

        let _sdio = &ctx.sdio;

        enable_slc_interrupt();

        // TODO: mock transfers from the SDIO host
        ctx.spi
            .write(&test_buffer)
            .expect("error writing SDIO from host");

        let mut res_buffer = [0u8; TEST_BLOCK_LEN];
        // expect R5 response from SDIO client
        ctx.spi
            .read(&mut res_buffer)
            .expect("error reading SDIO from host");

        // TODO: implement response types, and check for expected response
        assert_ne!(res_buffer, [0u8; TEST_BLOCK_LEN]);

        let exp_res = R5::new().into_bytes();
        assert_eq!(&res_buffer[..R5::LEN], exp_res.as_ref());
    }
}

#[cfg(feature = "esp32")]
fn enable_slc_interrupt() {}

#[cfg(feature = "esp32")]
#[allow(unused)]
extern "C" fn slc_handler() {
    // TODO: implement the handler
}

#[cfg(feature = "esp32c6")]
fn enable_slc_interrupt() {
    // enable the SLC0 interrupt, and set the handler
    interrupt::enable(Interrupt::SLC0, Priority::Priority1).expect("error enabling SLC0 interrupt");
    unsafe { interrupt::bind_interrupt(Interrupt::SLC0, IsrCallback::new(slc0_handler)) };
}

#[cfg(feature = "esp32c6")]
extern "C" fn slc0_handler() {
    // write data to the host
    // configuring DMA info
    // TODO: move this into the Sdio type
    // let slc = unsafe { &*ctx.sdio.slc().register_block };
    //
    // let buf_start = TX_BUFFER.address();
    // let buf_end = buf_start + (test_buffer.len() as u32);
    //
    // slc.slc1_tx_sharemem_start()
    // .write(|w| unsafe { w.bits(buf_start) });
    // slc.slc1_tx_sharemem_end()
    // .write(|w| unsafe { w.bits(buf_end) });
    //
    // configure SLC interrupts
    // slc.slc1int_ena1().modify(|_, w| {
    // w.sdio_slc1_tx_dscr_err_int_ena1().set_bit();
    // w.sdio_slc1_tx_ovf_int_ena1().set_bit()
    // });
    // TODO: match interrupt source to properly handle interrupts, e.g. send, receive, ...
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let slc = &*peripherals.SLC.register_block();

    // configure SLC TX descriptors
    slc.slc1tx_link_addr()
        .write(|w| unsafe { w.bits(TX_DESCRIPTORS.address()) });
    slc.slc1tx_link()
        .modify(|_, w| w.sdio_slc1_txlink_start().set_bit());

    let mut dma_buf = [0u8; 512];
    TX_BUFFER.read(&mut dma_buf);

    let cmd = Cmd52::try_from_bytes(&dma_buf);

    assert!(cmd.is_ok());
}
