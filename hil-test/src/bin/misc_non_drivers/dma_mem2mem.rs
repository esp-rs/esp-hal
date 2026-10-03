#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Blocking,
        dma::{DmaError, Mem2Mem},
        dma_buffers,
        dma_descriptors,
    };
    const DATA_SIZE: usize = 1024 * 10;

    struct Context {
        mem2mem: Mem2Mem<'static, Blocking>,
        #[cfg(soc_has_axi_gdma)]
        axi_channel: esp_hal::peripherals::DMA_AXI_CH0<'static>,
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
            #[cfg(soc_has_axi_gdma)]
            axi_channel: peripherals.DMA_AXI_CH0,
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

    #[test]
    #[cfg(soc_has_axi_gdma)]
    fn test_axi_dma_burst_size(ctx: Context) {
        use esp_hal::{
            dma::{
                BurstConfig,
                ExternalBurstConfig::{Size16, Size32, Size64},
            },
            dma_rx_buffer,
            dma_tx_buffer,
        };

        let mem2mem = Mem2Mem::new(ctx.axi_channel);
        let mut rx = mem2mem.rx;
        let mut tx = mem2mem.tx;
        let mut received = dma_rx_buffer!(256).unwrap();
        let mut sent = dma_tx_buffer!(256).unwrap();
        for (i, byte) in sent.as_mut_slice().iter_mut().enumerate() {
            *byte = (i % 256) as u8;
        }

        let regs = esp_hal::peripherals::AXI_GDMA::regs();
        let rx_regs = regs.in_ch(0);
        let tx_regs = regs.out_ch(0);
        for (rx_config, tx_config, rx_expected, tx_expected) in [
            (Size64.into(), Size16.into(), 3, 1),
            (Size16.into(), Size32.into(), 1, 2),
            (Size32.into(), Size64.into(), 2, 3),
            (BurstConfig::default(), BurstConfig::default(), 1, 1),
        ] {
            received.set_burst_config(rx_config).unwrap();
            sent.set_burst_config(tx_config).unwrap();
            received.as_mut_slice().fill(0);
            let input = rx.receive(received).map_err(|e| e.0).unwrap();
            let output = tx.send(sent).map_err(|e| e.0).unwrap();

            assert_eq!(
                rx_regs.in_conf0().read().in_burst_size_sel().bits(),
                rx_expected
            );
            assert_eq!(
                tx_regs.out_conf0().read().out_burst_size_sel().bits(),
                tx_expected
            );

            while !output.is_done() || rx_regs.in_int().raw().read().in_suc_eof().bit_is_clear() {
                core::hint::spin_loop();
            }
            let (result, next_tx, next_sent) = output.wait();
            result.unwrap();
            (tx, sent) = (next_tx, next_sent);
            (rx, received) = input.stop();
            assert_eq!(received.number_of_received_bytes(), sent.len());
            assert_eq!(received.as_slice(), sent.as_slice());
        }
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
