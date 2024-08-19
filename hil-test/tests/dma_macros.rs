//! DMA Mem2Mem Tests

//% CHIPS: esp32s3 esp32c2 esp32c3 esp32c6 esp32h2

#![no_std]
#![no_main]

use hil_test as _;

const DATA_SIZE: usize = 1024 * 10;

pub(crate) const fn compute_size(size: usize, chunk_size: usize) -> usize {
    size.div_ceil(chunk_size)
}

pub(crate) const fn compute_circular_size(size: usize, chunk_size: usize) -> usize {
    if size > chunk_size * 2 {
        size.div_ceil(chunk_size)
    } else {
        3
    }
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() {}

    #[test]
    fn test_dma_descriptors_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (tx_descriptors, rx_descriptors) = esp_hal::dma_descriptors!(DATA_SIZE);
        assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_descriptors_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const TX_SIZE: usize = DATA_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        let (tx_descriptors, rx_descriptors) = esp_hal::dma_descriptors!(TX_SIZE, RX_SIZE);
        assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
        assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_descriptors_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (tx_descriptors, rx_descriptors) = esp_hal::dma_circular_descriptors!(DATA_SIZE);
        assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_descriptors_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const TX_SIZE: usize = CHUNK_SIZE * 2;
        const RX_SIZE: usize = DATA_SIZE / 2;
        let (tx_descriptors, rx_descriptors) = esp_hal::dma_circular_descriptors!(TX_SIZE, RX_SIZE);
        assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
        assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_buffers_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            esp_hal::dma_buffers!(DATA_SIZE);
        assert_eq!(tx_buffer.len(), DATA_SIZE);
        assert_eq!(rx_buffer.len(), DATA_SIZE);
        assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_buffers_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const TX_SIZE: usize = DATA_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;

        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            esp_hal::dma_buffers!(TX_SIZE, RX_SIZE);
        assert_eq!(tx_buffer.len(), TX_SIZE);
        assert_eq!(rx_buffer.len(), RX_SIZE);
        assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
        assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_buffers_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            esp_hal::dma_circular_buffers!(DATA_SIZE);
        assert_eq!(tx_buffer.len(), DATA_SIZE);
        assert_eq!(rx_buffer.len(), DATA_SIZE);
        assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_buffers_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const TX_SIZE: usize = CHUNK_SIZE * 4;
        const RX_SIZE: usize = CHUNK_SIZE * 2;
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            esp_hal::dma_circular_buffers!(TX_SIZE, RX_SIZE);
        assert_eq!(tx_buffer.len(), TX_SIZE);
        assert_eq!(rx_buffer.len(), RX_SIZE);
        assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
        assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_descriptors_chunk_size_same_size() {
        const CHUNK_SIZE: usize = 2048;
        let (tx_descriptors, rx_descriptors) =
            esp_hal::dma_descriptors_chunk_size!(DATA_SIZE, CHUNK_SIZE);
        assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_descriptors_chunk_size_different_size() {
        const CHUNK_SIZE: usize = 2048;
        const TX_SIZE: usize = DATA_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        let (tx_descriptors, rx_descriptors) =
            esp_hal::dma_descriptors_chunk_size!(TX_SIZE, RX_SIZE, CHUNK_SIZE);
        assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
        assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_buffers_chunk_size_same_size() {
        const CHUNK_SIZE: usize = 2048;
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            esp_hal::dma_circular_buffers_chunk_size!(DATA_SIZE, CHUNK_SIZE);
        assert_eq!(tx_buffer.len(), DATA_SIZE);
        assert_eq!(rx_buffer.len(), DATA_SIZE);
        assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_buffers_chunk_size_different_size() {
        const CHUNK_SIZE: usize = 2048;
        const TX_SIZE: usize = DATA_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        let (tx_buffer, tx_descriptors, rx_buffer, rx_descriptors) =
            esp_hal::dma_circular_buffers_chunk_size!(TX_SIZE, RX_SIZE, CHUNK_SIZE);
        assert_eq!(tx_buffer.len(), TX_SIZE);
        assert_eq!(rx_buffer.len(), RX_SIZE);
        assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
        assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
    }
}
