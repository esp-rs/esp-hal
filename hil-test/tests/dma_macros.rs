//! DMA macro tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

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
#[embedded_test::tests(default_timeout = 3)]
mod tests {
    // defmt::* is load-bearing, it ensures that the assert in dma_buffers! is not
    // using defmt's non-const assert. Doing so would result in a compile error.
    #[allow(unused_imports)]
    use defmt::*;

    use super::*;

    #[test]
    fn test_dma_descriptors_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_descriptors!(DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_descriptors_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_descriptors!(RX_SIZE, TX_SIZE);
        core::assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_descriptors_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_circular_descriptors!(DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_descriptors_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = CHUNK_SIZE * 2;
        let (rx_descriptors, tx_descriptors) = esp_hal::dma_circular_descriptors!(RX_SIZE, TX_SIZE);
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_buffers_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_buffers!(DATA_SIZE);
        core::assert_eq!(rx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_descriptors.len(), rx_descriptors.len());
        core::assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_buffers_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;

        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_buffers!(RX_SIZE, TX_SIZE);
        core::assert_eq!(rx_buffer.len(), RX_SIZE);
        core::assert_eq!(tx_buffer.len(), TX_SIZE);
        core::assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_buffers_same_size() {
        use esp_hal::dma::CHUNK_SIZE;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers!(DATA_SIZE);
        core::assert_eq!(rx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_buffer.len(), DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_buffers_different_size() {
        use esp_hal::dma::CHUNK_SIZE;
        const RX_SIZE: usize = CHUNK_SIZE * 2;
        const TX_SIZE: usize = CHUNK_SIZE * 4;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers!(RX_SIZE, TX_SIZE);
        core::assert_eq!(rx_buffer.len(), RX_SIZE);
        core::assert_eq!(tx_buffer.len(), TX_SIZE);
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_descriptors_chunk_size_same_size() {
        const CHUNK_SIZE: usize = 2048;
        let (rx_descriptors, tx_descriptors) =
            esp_hal::dma_descriptors_chunk_size!(DATA_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(rx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(DATA_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_descriptors_chunk_size_different_size() {
        const CHUNK_SIZE: usize = 2048;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;
        let (rx_descriptors, tx_descriptors) =
            esp_hal::dma_descriptors_chunk_size!(RX_SIZE, TX_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_descriptors.len(), compute_size(RX_SIZE, CHUNK_SIZE));
        core::assert_eq!(tx_descriptors.len(), compute_size(TX_SIZE, CHUNK_SIZE));
    }

    #[test]
    fn test_dma_circular_buffers_chunk_size_same_size() {
        const CHUNK_SIZE: usize = 2048;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers_chunk_size!(DATA_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_buffer.len(), DATA_SIZE);
        core::assert_eq!(tx_buffer.len(), DATA_SIZE);
        core::assert_eq!(rx_descriptors.len(), tx_descriptors.len());
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(DATA_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_circular_buffers_chunk_size_different_size() {
        const CHUNK_SIZE: usize = 2048;
        const RX_SIZE: usize = DATA_SIZE / 2;
        const TX_SIZE: usize = DATA_SIZE;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
            esp_hal::dma_circular_buffers_chunk_size!(RX_SIZE, TX_SIZE, CHUNK_SIZE);
        core::assert_eq!(rx_buffer.len(), RX_SIZE);
        core::assert_eq!(tx_buffer.len(), TX_SIZE);
        core::assert_eq!(
            rx_descriptors.len(),
            compute_circular_size(RX_SIZE, CHUNK_SIZE)
        );
        core::assert_eq!(
            tx_descriptors.len(),
            compute_circular_size(TX_SIZE, CHUNK_SIZE)
        );
    }

    #[test]
    fn test_dma_tx_buffer() {
        use esp_hal::dma::DmaTxBuf;
        const TX_SIZE: usize = DATA_SIZE;

        fn check(tx_buf: DmaTxBuf, size: usize) {
            core::assert_eq!(tx_buf.len(), size);
        }
        check(esp_hal::dma_tx_buffer!(TX_SIZE), TX_SIZE);
        check(esp_hal::dma_tx_buffer!(TX_SIZE + 1), TX_SIZE + 1);
        check(esp_hal::dma_tx_buffer!(TX_SIZE + 2), TX_SIZE + 2);
        check(esp_hal::dma_tx_buffer!(TX_SIZE + 3), TX_SIZE + 3);
    }
}
