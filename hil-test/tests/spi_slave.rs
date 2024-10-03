//! SPI slave mode test suite.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{Dma, DmaPriority},
    dma_buffers,
    gpio::{interconnect::InputSignal, Io, Level, Output, PeripheralInput},
    peripherals::SPI2,
    spi::{slave::Spi, FullDuplexMode, SpiMode},
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2))] {
        type DmaChannelCreator = esp_hal::dma::Spi2DmaChannelCreator;
    } else {
        type DmaChannelCreator = esp_hal::dma::ChannelCreator<0>;
    }
}

struct Context {
    spi: Spi<'static, SPI2, FullDuplexMode>,
    dma_channel: DmaChannelCreator,
    bitbang_spi: BitbangSpi,
}

struct BitbangSpi {
    sclk: Output<'static>,
    mosi: Output<'static>,
    miso: InputSignal,
    cs: Output<'static>,
}

impl BitbangSpi {
    fn new(
        sclk: Output<'static>,
        mosi: Output<'static>,
        miso: InputSignal,
        cs: Output<'static>,
    ) -> Self {
        Self {
            sclk,
            mosi,
            miso,
            cs,
        }
    }

    fn assert_cs(&mut self) {
        self.cs.set_level(Level::Low);
    }

    fn deassert_cs(&mut self) {
        self.cs.set_level(Level::High);
    }

    // Mode 0, so sampled on the rising edge and set on the falling edge.
    fn shift_bit(&mut self, bit: bool) -> bool {
        self.mosi.set_level(Level::from(bit));
        self.sclk.set_level(Level::Low);

        let miso = self.miso.get_level().into();
        self.sclk.set_level(Level::High);

        miso
    }

    // Shift a byte out and in, MSB first.
    fn shift_byte(&mut self, byte: u8) -> u8 {
        let mut out = 0;
        for i in 0..8 {
            let shift = 7 - i;
            out |= (self.shift_bit((byte >> shift) & 1 != 0) as u8) << shift;
        }
        out
    }

    fn transfer_buf(&mut self, rx: &mut [u8], tx: &[u8]) {
        self.assert_cs();
        for (tx, rx) in tx.iter().zip(rx.iter_mut()) {
            *rx = self.shift_byte(*tx);
        }
        self.deassert_cs();
    }
}

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
        let (mosi_pin, miso_pin) = hil_test::i2c_pins!(io);
        let (sclk_pin, sclk_gpio) = hil_test::common_test_pins!(io);
        let cs_pin = hil_test::unconnected_pin!(io);

        let dma = Dma::new(peripherals.DMA);

        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32s2))] {
                let dma_channel = dma.spi2channel;
            } else {
                let dma_channel = dma.channel0;
            }
        }

        let cs = cs_pin.peripheral_input();
        let mosi = mosi_pin.peripheral_input();
        let mut miso = miso_pin.peripheral_input();

        let mosi_gpio = Output::new(mosi_pin, Level::Low);
        let cs_gpio = Output::new(cs_pin, Level::High);
        let sclk_gpio = Output::new(sclk_gpio, Level::Low);

        let spi = Spi::new(
            peripherals.SPI2,
            sclk_pin,
            mosi,
            miso_pin,
            cs,
            SpiMode::Mode0,
        );

        miso.enable_input(true, unsafe { esp_hal::Internal::conjure() });

        Context {
            spi,
            dma_channel,
            bitbang_spi: BitbangSpi::new(sclk_gpio, mosi_gpio, miso, cs_gpio),
        }
    }

    #[test]
    #[timeout(10)]
    fn test_basic(mut ctx: Context) {
        const DMA_SIZE: usize = 32;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_SIZE);
        let mut spi = ctx.spi.with_dma(
            ctx.dma_channel.configure(false, DmaPriority::Priority0),
            tx_descriptors,
            rx_descriptors,
        );
        let slave_send = tx_buffer;
        let slave_receive = rx_buffer;

        // The transfer stops if the buffers are full, not when the master
        // deasserts CS. Therefore, these need to be the same size as the DMA buffers.
        let master_send = &mut [0u8; DMA_SIZE];
        let master_receive = &mut [0xFFu8; DMA_SIZE];

        for (i, v) in master_send.iter_mut().enumerate() {
            *v = (i % 255) as u8;
        }
        for (i, v) in slave_send.iter_mut().enumerate() {
            *v = (254 - (i % 255)) as u8;
        }
        slave_receive.fill(0xFF);

        let transfer = spi.dma_transfer(slave_receive, &slave_send).unwrap();

        ctx.bitbang_spi.transfer_buf(master_receive, master_send);

        transfer.wait().unwrap();

        assert_eq!(slave_receive, master_send);
        assert_eq!(master_receive, slave_send);
    }
}
