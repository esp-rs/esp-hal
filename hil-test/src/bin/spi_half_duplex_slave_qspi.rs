//! SPI Half Duplex Read/Write, SPI Slave and QSPI Tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[embedded_test::tests(default_timeout = 3)]
mod read {
    use esp_hal::{
        Blocking,
        gpio::{Level, Output, OutputConfig},
        spi::{
            Mode,
            master::{Address, Command, Config, DataMode, Spi},
        },
        time::Rate,
    };
    #[cfg(spi_master_supports_dma)]
    use esp_hal::{
        dma::{DmaRxBuf, DmaTxBuf},
        dma_buffers,
    };

    #[cfg(spi_master_supports_dma)]
    cfg_if::cfg_if! {
        if #[cfg(any(pdma))] {
            type DmaChannel<'d> = esp_hal::peripherals::DMA_SPI2<'d>;
        } else {
            type DmaChannel<'d> = esp_hal::peripherals::DMA_CH0<'d>;
        }
    }

    struct Context {
        spi: Spi<'static, Blocking>,
        miso_mirror: Output<'static>,
        #[cfg(spi_master_supports_dma)]
        dma_channel: DmaChannel<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sclk = peripherals.GPIO0;
        let (miso, miso_mirror) = hil_test::common_test_pins!(peripherals);

        let miso_mirror = Output::new(miso_mirror, Level::High, OutputConfig::default());

        #[cfg(spi_master_supports_dma)]
        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sclk)
        .with_miso(miso);

        Context {
            spi,
            miso_mirror,
            #[cfg(spi_master_supports_dma)]
            dma_channel,
        }
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const BUFFER_SIZE: usize = 4;

        // SPI should read '0's from the MISO pin
        ctx.miso_mirror.set_low();

        let mut rx_buf = [0u8; BUFFER_SIZE];

        let mut spi = ctx.spi;

        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut rx_buf,
        )
        .unwrap();

        assert_eq!(rx_buf.as_slice(), &[0x00; BUFFER_SIZE]);

        // SPI should read '1's from the MISO pin
        ctx.miso_mirror.set_high();

        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut rx_buf,
        )
        .unwrap();

        assert_eq!(rx_buf.as_slice(), &[0xFF; BUFFER_SIZE]);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (buffer, descriptors, _, _) = dma_buffers!(DMA_BUFFER_SIZE, 0);
        let mut dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();

        // SPI should read '0's from the MISO pin
        ctx.miso_mirror.set_low();

        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        let transfer = spi
            .half_duplex_read(
                DataMode::SingleTwoDataLines,
                Command::None,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_rx_buf) = transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), &[0x00; DMA_BUFFER_SIZE]);

        // SPI should read '1's from the MISO pin
        ctx.miso_mirror.set_high();

        let transfer = spi
            .half_duplex_read(
                DataMode::SingleTwoDataLines,
                Command::None,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();

        (_, dma_rx_buf) = transfer.wait();

        assert_eq!(dma_rx_buf.as_slice(), &[0xFF; DMA_BUFFER_SIZE]);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_reads_correctly_from_gpio_pin(mut ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        // WAS THIS AN ERROR?
        let (buffer, descriptors, tx, txd) = dma_buffers!(DMA_BUFFER_SIZE, 1);
        let dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(txd, tx).unwrap();

        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        // SPI should read '0's from the MISO pin
        ctx.miso_mirror.set_low();

        let mut buffer = [0xAA; DMA_BUFFER_SIZE];
        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut buffer,
        )
        .unwrap();

        assert_eq!(buffer.as_slice(), &[0x00; DMA_BUFFER_SIZE]);

        // SPI should read '1's from the MISO pin
        ctx.miso_mirror.set_high();

        spi.half_duplex_read(
            DataMode::SingleTwoDataLines,
            Command::None,
            Address::None,
            0,
            &mut buffer,
        )
        .unwrap();

        assert_eq!(buffer.as_slice(), &[0xFF; DMA_BUFFER_SIZE]);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn data_mode_combinations_are_not_rejected(ctx: Context) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (buffer, descriptors, tx, txd) = dma_buffers!(DMA_BUFFER_SIZE, DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(txd, tx).unwrap();

        let mut buffer = [0xAA; DMA_BUFFER_SIZE];
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        let modes = [
            // 4-wire half-duplex mode
            (Command::None, Address::None, DataMode::SingleTwoDataLines),
            // Simple 3-wire half-duplex mode
            (Command::None, Address::None, DataMode::Single),
            // Simple DSPI/QSPI modes
            (Command::None, Address::None, DataMode::Dual),
            (Command::None, Address::None, DataMode::Quad),
            // Half-duplex modes with command and/or address phases
            (
                Command::_8Bit(0x32, DataMode::Single),
                Address::_24Bit(0x2C << 8, DataMode::Single),
                DataMode::Single,
            ),
            (
                Command::_8Bit(0x32, DataMode::Single),
                Address::_24Bit(0x2C << 8, DataMode::Single),
                DataMode::Dual,
            ),
            (
                Command::_8Bit(0x32, DataMode::Single),
                Address::_24Bit(0x2C << 8, DataMode::Single),
                DataMode::Quad,
            ),
            // SingleTwoDataLines is not meaningful for command/address phases but supporting it
            // shouldn't be an issue.
            (
                Command::_8Bit(0x32, DataMode::SingleTwoDataLines),
                Address::_24Bit(0x2C << 8, DataMode::SingleTwoDataLines),
                DataMode::Quad,
            ),
        ];

        for (command, address, data) in modes {
            if let Err(e) = spi.half_duplex_read(data, command, address, 0, &mut buffer) {
                panic!(
                    "Failed to read with command {:?}, address {:?}, data mode {:?}: {:?}",
                    command, address, data, e
                );
            }
        }
    }
}

#[cfg(pcnt_driver_supported)]
#[embedded_test::tests(default_timeout = 3)]
mod write {
    use esp_hal::{
        Blocking,
        gpio::interconnect::InputSignal,
        pcnt::{Pcnt, channel::EdgeMode, unit::Unit},
        spi::{
            Mode,
            master::{Address, Command, Config, DataMode, Spi},
        },
        time::Rate,
    };
    #[cfg(spi_master_supports_dma)]
    use esp_hal::{
        dma::{DmaRxBuf, DmaTxBuf},
        dma_buffers,
    };

    #[cfg(spi_master_supports_dma)]
    cfg_if::cfg_if! {
        if #[cfg(pdma)] {
            type DmaChannel<'d> = esp_hal::peripherals::DMA_SPI2<'d>;
        } else {
            type DmaChannel<'d> = esp_hal::peripherals::DMA_CH0<'d>;
        }
    }

    struct Context {
        spi: Spi<'static, Blocking>,
        pcnt_unit: Unit<'static, 0>,
        pcnt_source: InputSignal<'static>,

        #[cfg(spi_master_supports_dma)]
        dma_channel: DmaChannel<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let sclk = peripherals.GPIO0;
        let (mosi, _) = hil_test::common_test_pins!(peripherals);

        let pcnt = Pcnt::new(peripherals.PCNT);

        #[cfg(spi_master_supports_dma)]
        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let (mosi_loopback, mosi) = unsafe { mosi.split() };

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap()
        .with_sck(sclk)
        .with_sio0(mosi);

        Context {
            spi,
            pcnt_unit: pcnt.unit0,
            pcnt_source: mosi_loopback,
            #[cfg(spi_master_supports_dma)]
            dma_channel,
        }
    }

    fn perform_spi_writes_are_correctly_by_pcnt(ctx: Context, mode: DataMode) {
        const BUFFER_SIZE: usize = 4;

        let tx_buf = &mut [0; BUFFER_SIZE];

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi;

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        tx_buf.fill(0b0110_1010);

        spi.half_duplex_write(mode, Command::None, Address::None, 0, tx_buf)
            .unwrap();

        assert_eq!(unit.value(), (3 * BUFFER_SIZE) as _);

        spi.half_duplex_write(mode, Command::None, Address::None, 0, tx_buf)
            .unwrap();

        assert_eq!(unit.value(), (6 * BUFFER_SIZE) as _);
    }

    #[cfg(spi_master_supports_dma)]
    fn perform_spidma_writes_are_correctly_by_pcnt(ctx: Context, mode: DataMode) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx.spi.with_dma(ctx.dma_channel);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        // Fill the buffer where each byte has 3 pos edges.
        dma_tx_buf.fill(&[0b0110_1010; DMA_BUFFER_SIZE]);

        let transfer = spi
            .half_duplex_write(
                mode,
                Command::None,
                Address::None,
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        (spi, dma_tx_buf) = transfer.wait();

        assert_eq!(unit.value(), (3 * DMA_BUFFER_SIZE) as _);

        let transfer = spi
            .half_duplex_write(
                mode,
                Command::None,
                Address::None,
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        // dropping SPI would make us see an additional edge - so let's keep SPI alive
        let (_spi, _) = transfer.wait();

        assert_eq!(unit.value(), (6 * DMA_BUFFER_SIZE) as _);
    }

    #[cfg(spi_master_supports_dma)]
    fn perform_spidmabus_writes_are_correctly_by_pcnt(ctx: Context, mode: DataMode) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (rx, rxd, buffer, descriptors) = dma_buffers!(4, DMA_BUFFER_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rxd, rx).unwrap();
        let dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        let unit = ctx.pcnt_unit;
        let mut spi = ctx
            .spi
            .with_dma(ctx.dma_channel)
            .with_buffers(dma_rx_buf, dma_tx_buf);

        unit.channel0.set_edge_signal(ctx.pcnt_source);
        unit.channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let buffer = [0b0110_1010; DMA_BUFFER_SIZE];
        // Write the buffer where each byte has 3 pos edges.
        spi.half_duplex_write(mode, Command::None, Address::None, 0, &buffer)
            .unwrap();

        assert_eq!(unit.value(), (3 * DMA_BUFFER_SIZE) as _);

        spi.half_duplex_write(mode, Command::None, Address::None, 0, &buffer)
            .unwrap();

        assert_eq!(unit.value(), (6 * DMA_BUFFER_SIZE) as _);
    }

    #[test]
    fn test_spi_writes_are_correctly_by_pcnt(ctx: Context) {
        perform_spi_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    fn test_spi_writes_are_correctly_by_pcnt_tree_wire(ctx: Context) {
        perform_spi_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    fn test_spi_writes_are_correctly_by_pcnt_four_wire(ctx: Context) {
        perform_spi_writes_are_correctly_by_pcnt(ctx, DataMode::Single);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_writes_are_correctly_by_pcnt(ctx: Context) {
        perform_spidma_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_writes_are_correctly_by_pcnt(ctx: Context) {
        perform_spidmabus_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_writes_are_correctly_by_pcnt_tree_wire(ctx: Context) {
        perform_spidma_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_writes_are_correctly_by_pcnt_tree_wire(ctx: Context) {
        perform_spidmabus_writes_are_correctly_by_pcnt(ctx, DataMode::SingleTwoDataLines);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidma_writes_are_correctly_by_pcnt_four_wire(ctx: Context) {
        perform_spidma_writes_are_correctly_by_pcnt(ctx, DataMode::Single);
    }

    #[test]
    #[cfg(spi_master_supports_dma)]
    fn test_spidmabus_writes_are_correctly_by_pcnt_four_wire(ctx: Context) {
        perform_spidmabus_writes_are_correctly_by_pcnt(ctx, DataMode::Single);
    }
}

#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
#[cfg(any(feature = "unstable", spi_slave_driver_supported))]
#[cfg(spi_slave_supports_dma)]
mod spi_slave {
    use esp_hal::{
        Blocking,
        gpio::{Flex, Input, InputConfig, Level, OutputConfig, Pull},
        spi::{Mode, slave::Spi},
    };
    #[cfg(spi_slave_supports_dma)]
    use esp_hal::{
        dma::{DmaRxBuf, DmaTxBuf},
        dma_buffers,
    };

    #[cfg(spi_slave_supports_dma)]
    cfg_if::cfg_if! {
        if #[cfg(any(esp32, esp32s2))] {
            type DmaChannel<'d> = esp_hal::peripherals::DMA_SPI2<'d>;
        } else {
            type DmaChannel<'d> = esp_hal::peripherals::DMA_CH0<'d>;
        }
    }

    struct Context {
        spi: Spi<'static, Blocking>,
        #[cfg(spi_slave_supports_dma)]
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

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (mosi_pin, miso_pin) = hil_test::i2c_pins!(peripherals);
        let (sclk_pin, _) = hil_test::common_test_pins!(peripherals);
        let cs_pin = hil_test::unconnected_pin!(peripherals);

        #[cfg(spi_slave_supports_dma)]
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
            #[cfg(spi_slave_supports_dma)]
            dma_channel,
        }
    }

    #[test]
    fn test_basic_dma(mut ctx: Context) {
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

#[embedded_test::tests(default_timeout = 3)]
#[cfg(spi_master_supports_dma)]
mod qspi {
    #[cfg(pcnt_driver_supported)]
    use esp_hal::pcnt::{Pcnt, channel::EdgeMode, unit::Unit};
    use esp_hal::{
        Blocking,
        dma::{DmaRxBuf, DmaTxBuf},
        dma_buffers,
        gpio::{AnyPin, Input, InputConfig, Level, Output, OutputConfig, Pull},
        spi::{
            Mode,
            master::{Address, Command, Config, DataMode, Spi, SpiDma},
        },
        time::Rate,
    };

    cfg_if::cfg_if! {
        if #[cfg(pdma)] {
            type DmaChannel0<'d> = esp_hal::peripherals::DMA_SPI2<'d>;
        } else {
            type DmaChannel0<'d> = esp_hal::peripherals::DMA_CH0<'d>;
        }
    }

    cfg_if::cfg_if! {
        if #[cfg(esp32)] {
            const COMMAND_DATA_MODES: [DataMode; 1] = [DataMode::SingleTwoDataLines];
        } else {
            const COMMAND_DATA_MODES: [DataMode; 2] = [DataMode::SingleTwoDataLines, DataMode::Quad];
        }
    }

    type SpiUnderTest = SpiDma<'static, Blocking>;

    struct Context {
        spi: Spi<'static, Blocking>,
        #[cfg(pcnt_driver_supported)]
        pcnt: esp_hal::peripherals::PCNT<'static>,
        dma_channel: DmaChannel0<'static>,
        gpios: [AnyPin<'static>; 3],
    }

    fn transfer_read(
        spi: SpiUnderTest,
        dma_rx_buf: DmaRxBuf,
        command: Command,
    ) -> (SpiUnderTest, DmaRxBuf) {
        let transfer = spi
            .half_duplex_read(
                DataMode::Quad,
                command,
                Address::None,
                0,
                dma_rx_buf.len(),
                dma_rx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        transfer.wait()
    }

    fn transfer_write(
        spi: SpiUnderTest,
        dma_tx_buf: DmaTxBuf,
        write: u8,
        command_data_mode: DataMode,
    ) -> (SpiUnderTest, DmaTxBuf) {
        let transfer = spi
            .half_duplex_write(
                DataMode::Quad,
                Command::_8Bit(write as u16, command_data_mode),
                Address::_24Bit(
                    write as u32 | (write as u32) << 8 | (write as u32) << 16,
                    DataMode::Quad,
                ),
                0,
                dma_tx_buf.len(),
                dma_tx_buf,
            )
            .map_err(|e| e.0)
            .unwrap();
        transfer.wait()
    }

    fn execute_read(mut spi: SpiUnderTest, mut miso_mirror: Output<'static>, expected: u8) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(descriptors, buffer).unwrap();

        miso_mirror.set_low();
        (spi, dma_rx_buf) = transfer_read(spi, dma_rx_buf, Command::None);
        assert_eq!(dma_rx_buf.as_slice(), &[0; DMA_BUFFER_SIZE]);

        // Set two bits in the written bytes to 1
        miso_mirror.set_high();
        (_, dma_rx_buf) = transfer_read(spi, dma_rx_buf, Command::None);
        assert_eq!(dma_rx_buf.as_slice(), &[expected; DMA_BUFFER_SIZE]);
    }

    // Regression test for https://github.com/esp-rs/esp-hal/issues/1860
    fn execute_write_read(mut spi: SpiUnderTest, mut mosi_mirror: Output<'static>, expected: u8) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (rx_buffer, rx_descriptors, buffer, descriptors) =
            dma_buffers!(DMA_BUFFER_SIZE, DMA_BUFFER_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        dma_tx_buf.fill(&[0x00; DMA_BUFFER_SIZE]);

        for command_data_mode in COMMAND_DATA_MODES {
            (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, expected, command_data_mode);

            mosi_mirror.set_high();

            (spi, dma_rx_buf) = transfer_read(
                spi,
                dma_rx_buf,
                Command::_8Bit(expected as u16, command_data_mode),
            );
            assert_eq!(dma_rx_buf.as_slice(), &[expected; DMA_BUFFER_SIZE]);
        }
    }

    #[cfg(pcnt_driver_supported)]
    fn execute_write(
        unit0: Unit<'static, 0>,
        unit1: Unit<'static, 1>,
        mut spi: SpiUnderTest,
        write: u8,
        data_on_multiple_pins: bool,
    ) {
        const DMA_BUFFER_SIZE: usize = 4;

        let (_, _, buffer, descriptors) = dma_buffers!(0, DMA_BUFFER_SIZE);
        let mut dma_tx_buf = DmaTxBuf::new(descriptors, buffer).unwrap();

        for command_data_mode in COMMAND_DATA_MODES {
            dma_tx_buf.fill(&[write; DMA_BUFFER_SIZE]);

            // Send command + address + data.
            // Should read 8 high bits: 1 command bit, 3 address bits, 4 data bits
            unit0.clear();
            unit1.clear();
            (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
            assert_eq!(unit0.value() + unit1.value(), 8);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), 7);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), 8);
                }
            }

            // Send command + address only
            // Should read 4 bits high: 1 command bit, 3 address bits
            dma_tx_buf.set_length(0);
            unit0.clear();
            unit1.clear();
            (spi, dma_tx_buf) = transfer_write(spi, dma_tx_buf, write, command_data_mode);
            assert_eq!(unit0.value() + unit1.value(), 4);

            if data_on_multiple_pins {
                if command_data_mode == DataMode::SingleTwoDataLines {
                    assert_eq!(unit0.value(), 1);
                    assert_eq!(unit1.value(), 3);
                } else {
                    assert_eq!(unit0.value(), 0);
                    assert_eq!(unit1.value(), 4);
                }
            }
        }
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let (mut pin, mut pin_mirror) = hil_test::common_test_pins!(peripherals);
        let mut unconnected_pin = hil_test::unconnected_pin!(peripherals);

        // Make sure pins have no pullups
        let config = InputConfig::default().with_pull(Pull::Down);
        let _ = Input::new(pin.reborrow(), config);
        let _ = Input::new(pin_mirror.reborrow(), config);
        let _ = Input::new(unconnected_pin.reborrow(), config);

        cfg_if::cfg_if! {
            if #[cfg(pdma)] {
                let dma_channel = peripherals.DMA_SPI2;
            } else {
                let dma_channel = peripherals.DMA_CH0;
            }
        }

        let spi = Spi::new(
            peripherals.SPI2,
            Config::default()
                .with_frequency(Rate::from_khz(100))
                .with_mode(Mode::_0),
        )
        .unwrap();

        Context {
            spi,
            #[cfg(pcnt_driver_supported)]
            pcnt: peripherals.PCNT,
            dma_channel,
            gpios: [pin.into(), pin_mirror.into(), unconnected_pin.into()],
        }
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_0(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio0(pin).with_dma(ctx.dma_channel);

        execute_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio1(pin).with_dma(ctx.dma_channel);

        execute_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio2(pin).with_dma(ctx.dma_channel);

        execute_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    fn test_spi_reads_correctly_from_gpio_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio3(pin).with_dma(ctx.dma_channel);

        execute_read(spi, pin_mirror, 0b1000_1000);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_0(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio0(pin).with_dma(ctx.dma_channel);

        execute_write_read(spi, pin_mirror, 0b0001_0001);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_1(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio1(pin).with_dma(ctx.dma_channel);

        execute_write_read(spi, pin_mirror, 0b0010_0010);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_2(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio2(pin).with_dma(ctx.dma_channel);

        execute_write_read(spi, pin_mirror, 0b0100_0100);
    }

    #[test]
    fn test_spi_writes_and_reads_correctly_pin_3(ctx: Context) {
        let [pin, pin_mirror, _] = ctx.gpios;
        let pin_mirror = Output::new(pin_mirror, Level::High, OutputConfig::default());

        let spi = ctx.spi.with_sio3(pin).with_dma(ctx.dma_channel);

        execute_write_read(spi, pin_mirror, 0b1000_1000);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_0(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [_, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = unsafe { mosi.split() };

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx.spi.with_sio0(mosi).with_dma(ctx.dma_channel);

        execute_write(unit0, unit1, spi, 0b0000_0001, false);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_1(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = unsafe { mosi.split() };
        let (gpio_loopback, gpio) = unsafe { gpio.split() };

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(gpio_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx
            .spi
            .with_sio0(mosi)
            .with_sio1(gpio)
            .with_dma(ctx.dma_channel);

        execute_write(unit0, unit1, spi, 0b0000_0010, true);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_2(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = unsafe { mosi.split() };
        let (gpio_loopback, gpio) = unsafe { gpio.split() };

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(gpio_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx
            .spi
            .with_sio0(mosi)
            .with_sio2(gpio)
            .with_dma(ctx.dma_channel);

        execute_write(unit0, unit1, spi, 0b0000_0100, true);
    }

    #[test]
    #[cfg(pcnt_driver_supported)]
    fn test_spi_writes_correctly_to_pin_3(ctx: Context) {
        // For PCNT-using tests we swap the pins around so that the PCNT is not pulled
        // up by a resistor if the command phase doesn't drive its line.
        let [gpio, _, mosi] = ctx.gpios;

        let pcnt = Pcnt::new(ctx.pcnt);
        let unit0 = pcnt.unit0;
        let unit1 = pcnt.unit1;

        let (mosi_loopback, mosi) = unsafe { mosi.split() };
        let (gpio_loopback, gpio) = unsafe { gpio.split() };

        unit0.channel0.set_edge_signal(mosi_loopback);
        unit0
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        unit1.channel0.set_edge_signal(gpio_loopback);
        unit1
            .channel0
            .set_input_mode(EdgeMode::Hold, EdgeMode::Increment);

        let spi = ctx
            .spi
            .with_sio0(mosi)
            .with_sio3(gpio)
            .with_dma(ctx.dma_channel);

        execute_write(unit0, unit1, spi, 0b0000_1000, true);
    }
}
