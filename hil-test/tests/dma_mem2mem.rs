//! DMA Mem2Mem Tests

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use esp_hal::{
    dma::{AnyGdmaChannel, Channel, Dma, DmaPriority, DmaRxBuf, DmaTxBuf, Mem2Mem},
    dma_buffers,
    Blocking,
};
use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

cfg_if::cfg_if! {
    if #[cfg(any(esp32c2, esp32c6, esp32h2))] {
        type DmaPeripheralType = esp_hal::peripherals::MEM2MEM1;
    } else {
        type DmaPeripheralType = esp_hal::peripherals::SPI2;
    }
}

struct Context {
    channel: Channel<'static, AnyGdmaChannel, Blocking>,
    dma_peripheral: DmaPeripheralType,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        let dma = Dma::new(peripherals.DMA);
        let dma_channel = dma.channel0;

        cfg_if::cfg_if! {
            if #[cfg(any(esp32c2, esp32c6, esp32h2))] {
                let dma_peripheral = peripherals.MEM2MEM1;
            } else {
                let dma_peripheral = peripherals.SPI2;
            }
        }

        Context {
            channel: dma_channel
                .configure(false, DmaPriority::Priority0)
                .degrade(),
            dma_peripheral,
        }
    }

    #[test]
    fn test_mem2mem_rx_receives_tx(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mem2mem = Mem2Mem::new(ctx.channel, ctx.dma_peripheral);

        for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *b = (i % 256) as u8;
        }

        let rx_transfer = mem2mem
            .rx
            .receive(dma_rx_buf, true)
            .map_err(|e| e.0)
            .unwrap();
        let tx_transfer = mem2mem.tx.send(dma_tx_buf, true).map_err(|e| e.0).unwrap();

        let (_, _, dma_tx_buf) = tx_transfer.wait();
        let (_, dma_rx_buf) = rx_transfer.stop();

        assert_eq!(dma_rx_buf.as_slice(), dma_tx_buf.as_slice());
    }

    #[test]
    fn test_mem2mem_rx_receives_multiple_tx(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mem2mem = Mem2Mem::new(ctx.channel, ctx.dma_peripheral);

        let data_to_send: &[_; 100] = &core::array::from_fn(|i| (i % 256) as _);
        dma_tx_buf.fill(data_to_send);

        let rx_transfer = mem2mem
            .rx
            .receive(dma_rx_buf, true)
            .map_err(|e| e.0)
            .unwrap();

        let mut tx_transfer = mem2mem.tx.send(dma_tx_buf, true).map_err(|e| e.0).unwrap();
        for _ in 0..2 {
            let (_, tx, dma_tx_buf) = tx_transfer.wait();
            tx_transfer = tx.send(dma_tx_buf, true).map_err(|e| e.0).unwrap();
        }
        let (_, _, _) = tx_transfer.wait();

        let (_, dma_rx_buf) = rx_transfer.stop();

        let mut packets = dma_rx_buf.received_data();

        assert_eq!(data_to_send, packets.next().unwrap());
        assert_eq!(data_to_send, packets.next().unwrap());
        assert_eq!(data_to_send, packets.next().unwrap());
        assert!(packets.next().is_none());
    }

    #[test]
    fn test_mem2mem_multiple_rx_receives_tx(ctx: Context) {
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DATA_SIZE);
        let mut dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let mut dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let mem2mem = Mem2Mem::new(ctx.channel, ctx.dma_peripheral);

        for (i, b) in dma_tx_buf.as_mut_slice().iter_mut().enumerate() {
            *b = (i % 256) as u8;
        }

        dma_rx_buf.set_length(10);

        let tx_transfer = mem2mem.tx.send(dma_tx_buf, true).map_err(|e| e.0).unwrap();

        let mut bytes = (0..).into_iter();
        let mut rx = mem2mem.rx;
        for _ in 0..10 {
            let rx_transfer = rx.receive(dma_rx_buf, false).map_err(|e| e.0).unwrap();
            (_, rx, dma_rx_buf) = rx_transfer.wait();

            let received = dma_rx_buf.received_data().next().unwrap();
            assert_eq!(received.len(), 10);
            for b in received {
                assert_eq!(*b, bytes.next().unwrap());
            }
        }

        tx_transfer.stop();
    }
}
