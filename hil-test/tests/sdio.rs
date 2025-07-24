//! SDIO Tests

//% CHIPS: esp32 esp32c6
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    dma::{DescriptorFlagFields, Owner},
    sdio::{
        Config,
        Mode,
        Pins,
        Sdio,
        dma::{
            AtomicBuffer,
            AtomicDmaDescriptor,
            AtomicDmaDescriptors,
            DmaDescriptor,
            DmaDescriptorFlags,
        },
    },
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

#[cfg(test)]
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use super::*;

    #[init]
    fn init() -> Sdio<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                // GPIO Slot 1 config
                let pins = Pins::new(
                    Mode::Sd4bit,
                    peripherals.GPIO6,  // CLK/SCK
                    peripherals.GPIO11, // CMD/MOSI
                    peripherals.GPIO7,  // DAT0/MISO
                    peripherals.GPIO8,  // DAT1/IRQ
                    peripherals.GPIO9,  // DAT2
                    peripherals.GPIO10, // DAT3/#CS
                );
                // GPIO Slot 2 config
                //let pins = Pins::new(
                //    Mode::Sd4bit,
                //    peripherals.GPIO14, // CLK/SCK
                //    peripherals.GPIO15, // CMD/MOSI
                //    peripherals.GPIO2,  // DAT0/MISO
                //    peripherals.GPIO4,  // DAT1/IRQ
                //    peripherals.GPIO12, // DAT2
                //    peripherals.GPIO13, // DAT3/#CS
                //);
            } else if #[cfg(esp32c6)] {
                let pins = Pins::new(
                    Mode::Sd4bit,
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

        let config = Config::new();

        Sdio::new(
            peripherals.SLC,
            peripherals.SLCHOST,
            peripherals.HINF,
            pins,
            config,
        )
    }

    #[test]
    fn test_dma_rx_descriptors(sdio: Sdio<'static>) {
        // indicate a transfer of a single SDIO block
        let mut flags = DmaDescriptorFlags::new();
        flags.set_owner(Owner::Dma);
        flags.set_suc_eof(true);
        flags.set_size(512);
        flags.set_len(512);

        let rx_descriptor = &RX_DESCRIPTORS[0];

        let test_buffer: [u8; 512] = core::array::from_fn(|i| i as u8);
        RX_BUFFER.write(&test_buffer);

        // configuring DMA info
        // TODO: move this into the Sdio type
        let slc = unsafe { &*sdio.slc().register_block };

        let buf_start = RX_BUFFER.address();
        let buf_end = buf_start + (test_buffer.len() as u32);

        slc.slc1_rx_sharemem_start()
            .write(|w| unsafe { w.bits(buf_start) });
        slc.slc1_rx_sharemem_end()
            .write(|w| unsafe { w.bits(buf_end) });

        let buffer = unsafe { RX_BUFFER.as_ptr_mut() };
        rx_descriptor.update(DmaDescriptor {
            flags,
            buffer,
            next: core::ptr::null_mut(),
        });

        // increment the token value to indicate an added buffer
        slc.slc1token1()
            .write(|w| w.sdio_slc0_token1_inc().set_bit());

        // configure SLC interrupts
        slc.slc1int_ena1().modify(|_, w| {
            w.sdio_slc1_rx_dscr_err_int_ena1().set_bit();
            w.sdio_slc1_rx_udf_int_ena1().set_bit()
        });

        // configure SLC RX descriptors
        // TODO: move into Sdio type
        slc.slc1rx_link_addr()
            .write(|w| unsafe { w.bits(RX_DESCRIPTORS.address()) });
        slc.slc1rx_link()
            .modify(|_, w| w.sdio_slc1_rxlink_start().set_bit());

        let slc_int = slc.slc1int_st1().read();

        assert!(slc_int.sdio_slc1_rx_dscr_err_int_st1().bit_is_clear());
        assert!(slc_int.sdio_slc1_rx_udf_int_st1().bit_is_clear());

        // TODO: mock transfers to the SDIO host
    }

    #[test]
    fn test_dma_tx_descriptors(sdio: Sdio<'static>) {
        // indicate a transfer of a single SDIO block
        let mut flags = DmaDescriptorFlags::new();
        flags.set_owner(Owner::Dma);
        flags.set_suc_eof(true);
        flags.set_size(512);
        flags.set_len(512);

        let tx_descriptor = &TX_DESCRIPTORS[0];

        let test_buffer: [u8; 512] = core::array::from_fn(|i| i as u8);
        RX_BUFFER.write(&test_buffer);

        let buffer = unsafe { TX_BUFFER.as_ptr_mut() };
        tx_descriptor.update(DmaDescriptor {
            flags,
            buffer,
            next: core::ptr::null_mut(),
        });

        // configuring DMA info
        // TODO: move this into the Sdio type
        let slc = unsafe { &*sdio.slc().register_block };

        let buf_start = TX_BUFFER.address();
        let buf_end = buf_start + (test_buffer.len() as u32);

        slc.slc1_tx_sharemem_start()
            .write(|w| unsafe { w.bits(buf_start) });
        slc.slc1_tx_sharemem_end()
            .write(|w| unsafe { w.bits(buf_end) });

        // configure SLC interrupts
        slc.slc1int_ena1().modify(|_, w| {
            w.sdio_slc1_tx_dscr_err_int_ena1().set_bit();
            w.sdio_slc1_tx_ovf_int_ena1().set_bit()
        });

        // configure SLC RX descriptors
        // TODO: move into Sdio type
        slc.slc1tx_link_addr()
            .write(|w| unsafe { w.bits(TX_DESCRIPTORS.address()) });
        slc.slc1tx_link()
            .modify(|_, w| w.sdio_slc1_txlink_start().set_bit());

        let slc_int = slc.slc1int_st1().read();

        assert!(slc_int.sdio_slc1_tx_dscr_err_int_st1().bit_is_clear());
        assert!(slc_int.sdio_slc1_tx_ovf_int_st1().bit_is_clear());

        // TODO: mock transfers from the SDIO host
    }
}
