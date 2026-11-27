#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Blocking,
        dma::{
            BurstSize,
            CHUNK_SIZE,
            DmaAlignmentError,
            DmaError,
            Mem2Mem,
            max_compatible_chunk_size,
        },
        dma_descriptors,
    };

    use crate::dma_pool::dma_buffers;
    const DATA_SIZE: usize = 1024 * 10;

    /// The hardware channel index of the test channel.
    #[cfg(not(esp32s2))]
    const CHANNEL: usize = cfg_select! {
        any(esp32p4, esp32s31) => 1,
        _ => 0,
    };

    /// Returns the data burst fields of the test channel, RX and TX.
    #[cfg(not(esp32s2))]
    fn burst_registers() -> (u8, u8) {
        cfg_select! {
            esp32s31 => {
                let regs = esp_hal::peripherals::AXI_GDMA::regs();
                (
                    regs.in_ch(CHANNEL)
                        .in_conf0()
                        .read()
                        .in_burst_size_sel()
                        .bits(),
                    regs.out_ch(CHANNEL)
                        .out_conf0()
                        .read()
                        .out_burst_size_sel()
                        .bits(),
                )
            }
            dma_gdma_version = "1" => {
                let ch = esp_hal::peripherals::DMA::regs().ch(CHANNEL);
                (
                    ch.in_conf0().read().in_data_burst_en().bit() as u8,
                    ch.out_conf0().read().out_data_burst_en().bit() as u8,
                )
            }
            _ => {
                let ch = esp_hal::peripherals::DMA::regs().ch(CHANNEL);
                (
                    ch.in_conf0().read().in_data_burst_mode_sel().bits(),
                    ch.out_conf0().read().out_data_burst_mode_sel().bits(),
                )
            }
        }
    }

    cfg_select! {
        esp32s2 => {
            type Engine = esp_hal::dma::CopyDma;
        }
        esp32s31 => {
            type Engine = esp_hal::dma::AxiGdma;
        }
        _ => {
            type Engine = esp_hal::dma::AhbGdma;
        }
    }

    struct Context {
        mem2mem: Mem2Mem<'static, Blocking, Engine>,
        #[cfg(all(dma_can_access_psram, not(any(esp32s2, dma_gdma_version = "1"))))]
        psram: esp_hal::peripherals::PSRAM<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());
        hil_test::init_logger!(peripherals);

        let mem2mem = cfg_select! {
            esp32s2 => Mem2Mem::new(peripherals.DMA_COPY),
            any(esp32c3, esp32s3) => Mem2Mem::new(peripherals.DMA_CH0, peripherals.SPI2),
            esp32p4 => Mem2Mem::new(peripherals.DMA_CH1),
            esp32s31 => Mem2Mem::new(peripherals.DMA_AXI_CH1),
            _ => Mem2Mem::new(peripherals.DMA_CH0),
        };

        Context {
            mem2mem,
            #[cfg(all(dma_can_access_psram, not(any(esp32s2, dma_gdma_version = "1"))))]
            psram: peripherals.PSRAM,
        }
    }

    #[test]
    fn test_internal_mem2mem(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            dma_buffers(DATA_SIZE, DATA_SIZE, CHUNK_SIZE);

        let mut mem2mem = ctx
            .mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Default::default())
            .unwrap();

        for i in 0..tx_buffer.len() {
            tx_buffer[i] = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(rx_buffer, tx_buffer).unwrap();
        dma_wait.wait().unwrap();
        for i in 0..tx_buffer.len() {
            assert_eq!(rx_buffer[i], tx_buffer[i]);
        }
    }

    #[test]
    fn test_mem2mem_transfers_after_dma_config(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers(
            1024,
            1024,
            max_compatible_chunk_size(Some(BurstSize::Size32)),
        );

        // With the register value of a 32-byte burst, or of burst on for on/off engines.
        let (config, _expected_burst) = cfg_select! {
            esp32s2 => (esp_hal::dma::CopyDmaConfig::default(), ()),
            esp32s31 => (
                esp_hal::dma::AxiGdmaConfig::default()
                    .with_priority(esp_hal::dma::AxiPriority::Priority5)
                    .with_internal_burst(esp_hal::dma::AxiInternalBurst::Size32),
                2,
            ),
            dma_gdma_version = "1" => (
                esp_hal::dma::AhbGdmaConfig::default()
                    .with_priority(esp_hal::dma::AhbPriority::Priority5)
                    .with_internal_burst(esp_hal::dma::AhbInternalBurst::Enabled),
                1,
            ),
            _ => (
                esp_hal::dma::AhbGdmaConfig::default()
                    .with_priority(esp_hal::dma::AhbPriority::Priority5)
                    .with_internal_burst(esp_hal::dma::AhbInternalBurst::Size32),
                2,
            ),
        };

        let mut mem2mem = ctx.mem2mem;
        mem2mem.apply_dma_config(&config).unwrap();

        let mut mem2mem = mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Some(BurstSize::Size32))
            .unwrap();

        for (i, byte) in tx_buffer.iter_mut().enumerate() {
            *byte = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(rx_buffer, tx_buffer).unwrap();
        dma_wait.wait().unwrap();
        assert_eq!(rx_buffer, tx_buffer);

        #[cfg(not(esp32s2))]
        assert_eq!(burst_registers(), (_expected_burst, _expected_burst));
    }

    #[test]
    fn test_mem2mem_transfers_after_invalid_alignment(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers(
            DATA_SIZE,
            DATA_SIZE,
            max_compatible_chunk_size(Some(BurstSize::Size128)),
        );

        let mut mem2mem = ctx
            .mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Some(BurstSize::Size128))
            .unwrap();

        assert!(matches!(
            mem2mem.start_transfer(&mut rx_buffer[1..], &tx_buffer[1..]),
            Err(DmaError::InvalidAlignment(_))
        ));

        // Aligned for internal memory, but not to the minimum burst.
        const OFFSET: usize = cfg_select! {
            soc_internal_memory_cached => 64,
            _ => 4,
        };
        assert!(matches!(
            mem2mem.start_transfer(&mut rx_buffer[OFFSET..], &tx_buffer[OFFSET..]),
            Err(DmaError::InvalidAlignment(DmaAlignmentError::Address))
        ));

        for (i, byte) in tx_buffer.iter_mut().enumerate() {
            *byte = (i % 256) as u8;
        }
        let dma_wait = mem2mem.start_transfer(rx_buffer, tx_buffer).unwrap();
        dma_wait.wait().unwrap();
        assert_eq!(rx_buffer, tx_buffer);
    }

    #[test]
    fn test_mem2mem_errors_zero_tx(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(1024, 0);
        match ctx
            .mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Default::default())
        {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    fn test_mem2mem_errors_zero_rx(ctx: Context) {
        let (rx_descriptors, tx_descriptors) = dma_descriptors!(0, 1024);
        match ctx
            .mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, Default::default())
        {
            Err(DmaError::OutOfDescriptors) => (),
            _ => panic!("Expected OutOfDescriptors"),
        }
    }

    #[test]
    #[cfg(dma_gdma_version = "1")]
    fn test_mem2mem_rx_burst_follows_buffer_alignment(ctx: Context) {
        use esp_hal::dma::{
            AhbGdmaConfig,
            AhbInternalBurst,
            DmaRxBuf,
            DmaTxBuf,
            aligned::DmaAlignedMut,
        };

        let mut mem2mem = ctx.mem2mem;
        mem2mem
            .apply_dma_config(
                &AhbGdmaConfig::default().with_internal_burst(AhbInternalBurst::Enabled),
            )
            .unwrap();

        // A receive length that is not a multiple of 4 bytes cannot use burst. Transmit needs no
        // alignment.
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            dma_buffers(255, 255, max_compatible_chunk_size(None));
        // SAFETY: The length is not a multiple of the region alignment, so `DmaAlignedMut::new`
        // rejects the slice. The address is aligned to the pool alignment, and the bytes after
        // the slice stay unused.
        let (rx_buffer, tx_buffer) = unsafe {
            (
                DmaAlignedMut::new_unchecked(rx_buffer),
                DmaAlignedMut::new_unchecked(tx_buffer),
            )
        };
        let rx_buffer =
            DmaRxBuf::new(DmaAlignedMut::new(rx_descriptors).unwrap(), rx_buffer).unwrap();
        let tx_buffer =
            DmaTxBuf::new(DmaAlignedMut::new(tx_descriptors).unwrap(), tx_buffer).unwrap();

        let Mem2Mem { rx, tx } = mem2mem;
        let rx_transfer = rx.receive(rx_buffer).map_err(|e| e.0).unwrap();
        let tx_transfer = tx.send(tx_buffer).map_err(|e| e.0).unwrap();
        let registers = burst_registers();

        let (result, _, _) = tx_transfer.wait();
        result.unwrap();
        let _ = rx_transfer.stop();

        assert_eq!(registers, (0, 1));
    }

    #[test]
    #[cfg(all(dma_can_access_psram, not(any(esp32s2, dma_gdma_version = "1"))))]
    fn test_mem2mem_tx_from_psram_uses_external_burst(ctx: Context) {
        const LEN: usize = 1024;

        // One burst field selects the burst for both memories, so a PSRAM source gets the
        // external burst (16 B, code 1) and an internal destination the internal one (32 B,
        // code 2).
        let config = cfg_select! {
            esp32s31 => esp_hal::dma::AxiGdmaConfig::default()
                .with_internal_burst(esp_hal::dma::AxiInternalBurst::Size32)
                .with_external_burst(esp_hal::dma::AxiExternalBurst::Size16),
            _ => esp_hal::dma::AhbGdmaConfig::default()
                .with_internal_burst(esp_hal::dma::AhbInternalBurst::Size32)
                .with_external_burst(esp_hal::dma::AhbExternalBurst::Size16),
        };

        let psram = esp_hal::psram::Psram::new(ctx.psram, Default::default());
        let (start, size) = psram.raw_parts();
        assert!(size >= LEN);
        let tx_memory = unsafe { core::slice::from_raw_parts_mut(start, LEN) };
        for (i, byte) in tx_memory.iter_mut().enumerate() {
            *byte = (i % 251) as u8;
        }

        let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers(LEN, LEN, CHUNK_SIZE);

        let mut mem2mem = ctx.mem2mem;
        mem2mem.apply_dma_config(&config).unwrap();
        let mut mem2mem = mem2mem
            .with_descriptors(rx_descriptors, tx_descriptors, None)
            .unwrap();

        let dma_wait = mem2mem.start_transfer(rx_buffer, tx_memory).unwrap();
        let registers = burst_registers();
        dma_wait.wait().unwrap();

        assert_eq!(registers, (2, 1));
        for (i, byte) in rx_buffer.iter().enumerate() {
            assert_eq!(*byte, (i % 251) as u8, "Mismatch at index {i}");
        }
    }
}
