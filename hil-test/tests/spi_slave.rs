//! SPI slave mode test suite.
//!
//! ESP32 does not support Modes 0 and 2 (properly, at least), so here we're
//! testing Mode 1.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_hal::{
    Blocking,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    gpio::{Flex, Input, InputConfig, Level, OutputConfig, Pull},
    spi::{Mode, slave::Spi},
};
use hil_test as _;

cfg_if::cfg_if! {
    if #[cfg(any(esp32, esp32s2))] {
        type DmaChannel<'d> = esp_hal::peripherals::DMA_SPI2<'d>;
    } else {
        type DmaChannel<'d> = esp_hal::peripherals::DMA_CH0<'d>;
    }
}

struct Context {
    spi: Spi<'static, Blocking>,
    dma_channel: DmaChannel<'static>,
    bitbang_spi: BitbangSpi,
}

struct BitbangSpi {
    sclk: Flex<'static>,
    mosi: Flex<'static>,
    miso: Input<'static>,
    cs: Flex<'static>,
}

impl BitbangSpi {
    fn new(
        sclk: Flex<'static>,
        mosi: Flex<'static>,
        miso: Input<'static>,
        cs: Flex<'static>,
    ) -> Self {
        Self {
            sclk,
            mosi,
            miso,
            cs,
        }
    }

    fn assert_cs(&mut self) {
        self.sclk.set_level(Level::Low);
        self.cs.set_level(Level::Low);
    }

    fn deassert_cs(&mut self) {
        self.sclk.set_level(Level::Low);
        self.cs.set_level(Level::High);
    }

    // Mode 1, so sampled on the rising edge and set on the falling edge.
    fn shift_bit(&mut self, bit: bool) -> bool {
        self.mosi.set_level(Level::from(bit));
        self.sclk.set_level(Level::High);

        let miso = self.miso.level().into();
        self.sclk.set_level(Level::Low);

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
#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (mosi_pin, miso_pin) = hil_test::i2c_pins!(peripherals);
        let (sclk_pin, _) = hil_test::common_test_pins!(peripherals);
        let cs_pin = hil_test::unconnected_pin!(peripherals);

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let mut mosi_gpio = Flex::new(mosi_pin);
        mosi_gpio.apply_output_config(&OutputConfig::default());
        mosi_gpio.set_level(Level::Low);
        mosi_gpio.set_output_enable(true);

        let mut cs_gpio = Flex::new(cs_pin);
        cs_gpio.apply_output_config(&OutputConfig::default());
        cs_gpio.set_level(Level::High);
        cs_gpio.set_output_enable(true);

        let mut sclk_gpio = Flex::new(sclk_pin);
        sclk_gpio.apply_output_config(&OutputConfig::default());
        sclk_gpio.set_level(Level::Low);
        sclk_gpio.set_output_enable(true);

        let mut miso_gpio = Flex::new(miso_pin);
        miso_gpio.set_input_enable(true);
        miso_gpio.apply_input_config(&InputConfig::default().with_pull(Pull::None));
        miso_gpio.apply_output_config(&OutputConfig::default());

        let cs = cs_gpio.peripheral_input();
        let sclk = sclk_gpio.peripheral_input();
        let mosi = mosi_gpio.peripheral_input();
        let (miso_in, miso_out) = unsafe { miso_gpio.split_into_drivers() };

        Context {
            spi: Spi::new(peripherals.SPI2, Mode::_1)
                .with_sck(sclk)
                .with_mosi(mosi)
                .with_miso(miso_out)
                .with_cs(cs),
            bitbang_spi: BitbangSpi::new(sclk_gpio, mosi_gpio, miso_in, cs_gpio),
            dma_channel,
        }
    }

    #[test]
    fn test_basic(mut ctx: Context) {
        const DMA_SIZE: usize = 32;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_SIZE);
        let mut slave_receive = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut slave_send = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let spi = ctx.spi.with_dma(ctx.dma_channel);

        // The transfer stops if the buffers are full, not when the master
        // deasserts CS. Therefore, these need to be the same size as the DMA buffers.
        let master_send = &mut [0u8; DMA_SIZE];
        let master_receive = &mut [0xFFu8; DMA_SIZE];

        for (i, v) in master_send.iter_mut().enumerate() {
            *v = (i % 255) as u8;
        }
        for (i, v) in slave_send.as_mut_slice().iter_mut().enumerate() {
            *v = (254 - (i % 255)) as u8;
        }
        slave_receive.as_mut_slice().fill(0xFF);

        let transfer = spi
            .transfer(DMA_SIZE, slave_receive, DMA_SIZE, slave_send)
            .unwrap();

        ctx.bitbang_spi.transfer_buf(master_receive, master_send);

        (_, (slave_receive, slave_send)) = transfer.wait();

        assert_eq!(slave_receive.as_slice(), master_send);
        assert_eq!(master_receive, slave_send.as_slice());
    }
}
