#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Blocking,
        dma::{DmaError, Mem2Mem},
        dma_buffers,
        dma_descriptors,
    };
    #[cfg(dma_has_axi_gdma)]
    use esp_hal::{
        dma::{AxiBurstConfig, AxiBurstSize, AxiGdmaChannel, DmaDescriptor},
        peripherals::{AXI_GDMA, DMA_AXI_CH0, DMA_AXI_CH2},
    };
    const DATA_SIZE: usize = 1024 * 10;

    struct Context {
        mem2mem: Mem2Mem<'static, Blocking>,
        #[cfg(dma_has_axi_gdma)]
        axi: DMA_AXI_CH0<'static>,
        #[cfg(dma_has_axi_gdma)]
        axi_keepalive: DMA_AXI_CH2<'static>,
        #[cfg(esp32s31)]
        psram: esp_hal::peripherals::PSRAM<'static>,
    }

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let mem2mem = cfg_select! {
            esp32s2 => Mem2Mem::new(peripherals.DMA_COPY),
            any(esp32c3, esp32s3) => Mem2Mem::new(peripherals.DMA_CH0, peripherals.SPI2),
            esp32p4 => Mem2Mem::new(peripherals.DMA_CH1),
            esp32s31 => Mem2Mem::new(peripherals.DMA_AXI_CH1),
            _ => Mem2Mem::new(peripherals.DMA_CH0),
        };

        Context {
            mem2mem,
            #[cfg(dma_has_axi_gdma)]
            axi: peripherals.DMA_AXI_CH0,
            #[cfg(dma_has_axi_gdma)]
            axi_keepalive: peripherals.DMA_AXI_CH2,
            #[cfg(esp32s31)]
            psram: peripherals.PSRAM,
        }
    }

    #[test]
    fn test_internal_mem2mem(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);

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

    #[cfg(dma_has_axi_gdma)]
    fn copy_and_check<'d>(
        dma: Mem2Mem<'d, Blocking>,
        rx: &mut [u8],
        tx: &mut [u8],
        descriptors: (&'d mut [DmaDescriptor], &'d mut [DmaDescriptor]),
        selectors: (u8, u8),
    ) {
        let mut dma = dma
            .with_descriptors(descriptors.0, descriptors.1, Default::default())
            .unwrap();
        for (i, byte) in tx.iter_mut().enumerate() {
            *byte = i as u8;
        }
        for (dst, src) in rx.iter_mut().zip(tx.iter()) {
            *dst = !src;
        }

        let transfer = dma.start_transfer(rx, tx).unwrap();
        let regs = AXI_GDMA::regs();
        assert_eq!(
            regs.in_ch(0).in_conf0().read().in_burst_size_sel().bits(),
            selectors.0
        );
        assert_eq!(
            regs.out_ch(0)
                .out_conf0()
                .read()
                .out_burst_size_sel()
                .bits(),
            selectors.1
        );
        transfer.wait().unwrap();
        assert_eq!(rx, tx);
    }

    #[test]
    #[cfg(dma_has_axi_gdma)]
    fn test_axi_mem2mem_burst(mut ctx: Context) {
        let (rx, rx_desc, tx, tx_desc) = dma_buffers!(DATA_SIZE);
        let (rx, tx) = if cfg!(esp32s31) {
            // S31 SRAM buffers need four-byte alignment in the HAL. Use offsets
            // and a length that is not a multiple of any AXI burst size.
            (&mut rx[4..], &mut tx[..DATA_SIZE - 4])
        } else {
            (rx, tx)
        };

        // Keep the engine clocked so a reset cannot hide stale selectors.
        let _keepalive = Mem2Mem::new(ctx.axi_keepalive);
        let config = AxiBurstConfig::default()
            .with_rx(AxiBurstSize::Bytes128)
            .with_tx(AxiBurstSize::Bytes16);
        let channel = AxiGdmaChannel::from(ctx.axi.reborrow()).with_burst_config(config);
        copy_and_check(Mem2Mem::new(channel), rx, tx, (rx_desc, tx_desc), (4, 1));
        copy_and_check(
            Mem2Mem::new(ctx.axi.reborrow()),
            rx,
            tx,
            (rx_desc, tx_desc),
            (0, 0),
        );
    }

    #[test]
    #[cfg(esp32s31)]
    fn test_axi_mem2mem_burst_psram(ctx: Context) {
        const LEN: usize = 4160; // Cache-line aligned on S31, with two descriptors.
        let psram = esp_hal::psram::Psram::new(ctx.psram, Default::default());
        let (ptr, len) = psram.raw_parts();
        assert!(len >= 2 * LEN);
        // This test owns the first two buffers in the PSRAM mapping.
        let (rx, tx) = unsafe { core::slice::from_raw_parts_mut(ptr, 2 * LEN) }.split_at_mut(LEN);
        let (rx_desc, tx_desc) = dma_descriptors!(LEN);

        let config = AxiBurstConfig::default()
            .with_rx(AxiBurstSize::Bytes16)
            .with_tx(AxiBurstSize::Bytes128);
        let channel = AxiGdmaChannel::from(ctx.axi).with_burst_config(config);
        copy_and_check(Mem2Mem::new(channel), rx, tx, (rx_desc, tx_desc), (1, 4));
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
}
