#[embedded_test::tests(default_timeout = 3)]
mod tests {
    const BUFFER_SIZE: usize = 4096;
    const CHUNK_SIZE: usize = 1024;

    // defmt::* is load-bearing, it ensures that the assert in dma_buffers! is not
    // using defmt's non-const assert. Doing so would result in a compile error.
    #[allow(unused_imports)]
    use defmt::*;
    use esp_hal::{
        dma::{
            DmaBufError,
            DmaDescriptor,
            DmaRxBuffer,
            DmaRxStreamBuf,
            DmaRxStreamBufView,
            DmaTxBuffer,
            DmaTxStreamBuf,
            DmaTxStreamBufView,
            Owner,
        },
        dma_rx_stream_buffer,
        dma_tx_stream_buffer,
    };

    fn chunk_size(descriptors: &[DmaDescriptor]) -> usize {
        descriptors[0].size()
    }

    fn simulate_dma_rx_fill(
        descriptors: &mut [DmaDescriptor],
        buffer: &mut [u8],
        descriptor_index: usize,
        data: &[u8],
        eof: bool,
    ) {
        let chunk = chunk_size(descriptors);
        let offset = descriptor_index * chunk;
        buffer[offset..offset + data.len()].copy_from_slice(data);
        descriptors[descriptor_index].set_length(data.len());
        descriptors[descriptor_index].set_suc_eof(eof);
        descriptors[descriptor_index].set_owner(Owner::Cpu);
    }

    fn with_rx_view(initial_data: &[u8], eof: bool, f: impl FnOnce(DmaRxStreamBufView)) {
        let mut buf = dma_rx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        buf.prepare();
        let (mut descriptors, mut buffer) = buf.split();
        if !initial_data.is_empty() {
            simulate_dma_rx_fill(&mut descriptors, &mut buffer, 0, initial_data, eof);
        }
        let buf = DmaRxStreamBuf::new(descriptors, buffer).unwrap();
        f(buf.into_view());
    }

    fn with_rx_view_multi<F>(fills: &[(&[u8], bool)], f: F)
    where
        F: FnOnce(DmaRxStreamBufView),
    {
        let mut buf = dma_rx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        buf.prepare();
        let (mut descriptors, mut buffer) = buf.split();
        for (index, (data, eof)) in fills.iter().enumerate() {
            simulate_dma_rx_fill(&mut descriptors, &mut buffer, index, data, *eof);
        }
        let buf = DmaRxStreamBuf::new(descriptors, buffer).unwrap();
        f(buf.into_view());
    }

    fn with_tx_view(all_cpu_owned: bool, f: impl FnOnce(DmaTxStreamBufView)) {
        let mut buf = dma_tx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        buf.prepare();
        let (descriptors, buffer) = buf.split();
        if all_cpu_owned {
            for desc in descriptors.iter_mut() {
                desc.set_owner(Owner::Cpu);
                desc.set_length(0);
            }
        }
        let mut buf = DmaTxStreamBuf::new(descriptors, buffer).unwrap();
        buf.push(&[]);
        f(buf.into_view());
    }

    #[init]
    fn init() {
        // Ensures that watchdogs are disabled
        let _ = esp_hal::init(Default::default());
    }

    #[test]
    fn test_dma_rx_stream_buf_new() {
        let buf = dma_rx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        let (descriptors, buffer) = buf.split();
        core::assert_eq!(buffer.len(), BUFFER_SIZE);
        core::assert_eq!(descriptors.len(), BUFFER_SIZE / CHUNK_SIZE);
        core::assert!(descriptors.len() >= 4);
    }

    #[test]
    fn test_dma_tx_stream_buf_new() {
        let buf = dma_tx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        let (descriptors, buffer) = buf.split();
        core::assert_eq!(buffer.len(), BUFFER_SIZE);
        core::assert_eq!(descriptors.len(), BUFFER_SIZE / CHUNK_SIZE);
        core::assert!(descriptors.len() >= 4);
    }

    #[test]
    fn test_dma_rx_stream_buf_insufficient_descriptors() {
        let (buffer, descriptors) =
            esp_hal::dma_buffers_impl!(BUFFER_SIZE, CHUNK_SIZE * 2, is_circular = false);
        match DmaRxStreamBuf::new(descriptors, buffer) {
            Err(DmaBufError::InsufficientDescriptors) => (),
            _ => core::panic!("expected InsufficientDescriptors"),
        }
    }

    #[test]
    fn test_dma_tx_stream_buf_insufficient_descriptors() {
        let (buffer, descriptors) =
            esp_hal::dma_buffers_impl!(BUFFER_SIZE, CHUNK_SIZE * 2, is_circular = false);
        match DmaTxStreamBuf::new(descriptors, buffer) {
            Err(DmaBufError::InsufficientDescriptors) => (),
            _ => core::panic!("expected InsufficientDescriptors"),
        }
    }

    #[test]
    fn test_dma_rx_stream_buf_prepare_is_idempotent() {
        let mut buf = dma_rx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        buf.prepare();
        buf.prepare();
    }

    #[test]
    fn test_dma_tx_stream_buf_prepare_is_idempotent() {
        let mut buf = dma_tx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        buf.prepare();
        buf.prepare();
    }

    #[test]
    fn test_dma_tx_stream_buf_push_before_transfer() {
        let mut tx_buf = dma_tx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        let first = [1u8, 2, 3, 4, 5];
        let second = [6u8, 7, 8];

        core::assert_eq!(tx_buf.push(&first), first.len());
        core::assert_eq!(tx_buf.push(&second), second.len());

        let (_, buffer) = tx_buf.split();
        core::assert_eq!(&buffer[..first.len()], &first);
        core::assert_eq!(&buffer[first.len()..first.len() + second.len()], &second);
    }

    #[test]
    fn test_dma_tx_stream_buf_push_with_before_transfer() {
        let mut tx_buf = dma_tx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        let pushed = tx_buf.push_with(|buf| {
            for (index, byte) in buf.iter_mut().enumerate().take(10) {
                *byte = index as u8;
            }
            10
        });
        core::assert_eq!(pushed, 10);

        let (_, buffer) = tx_buf.split();
        for (index, byte) in buffer[..10].iter().enumerate() {
            core::assert_eq!(*byte, index as u8);
        }
    }

    #[test]
    fn test_dma_tx_stream_buf_view_unprefilled_has_no_available_space() {
        let mut buf = dma_tx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        buf.prepare();
        let view = buf.into_view();
        core::assert_eq!(view.available_bytes(), 0);
    }

    #[test]
    fn test_dma_tx_stream_buf_view_available_bytes() {
        with_tx_view(true, |view| {
            core::assert_eq!(view.available_bytes(), BUFFER_SIZE);
        });
    }

    #[test]
    fn test_dma_tx_stream_buf_view_push() {
        with_tx_view(true, |mut view| {
            let data = b"hello, dma tx stream";
            let pushed = view.push(data);
            core::assert_eq!(pushed, data.len());
            core::assert_eq!(view.available_bytes(), BUFFER_SIZE - data.len());

            let buf = <DmaTxStreamBuf as DmaTxBuffer>::from_view(view);
            let (_, buffer) = buf.split();
            core::assert_eq!(&buffer[..data.len()], data);
        });
    }

    #[test]
    fn test_dma_tx_stream_buf_view_push_with() {
        with_tx_view(true, |mut view| {
            let pushed = view.push_with(|buf| {
                buf[..3].copy_from_slice(&[10, 11, 12]);
                3
            });
            core::assert_eq!(pushed, 3);
            core::assert_eq!(view.available_bytes(), BUFFER_SIZE - 3);

            let buf = <DmaTxStreamBuf as DmaTxBuffer>::from_view(view);
            let (_, buffer) = buf.split();
            core::assert_eq!(&buffer[..3], &[10, 11, 12]);
        });
    }

    #[test]
    fn test_dma_tx_stream_buf_view_push_respects_available_space() {
        with_tx_view(true, |mut view| {
            let oversized = [0u8; BUFFER_SIZE + 1];
            let pushed = view.push(&oversized);
            core::assert_eq!(pushed, BUFFER_SIZE);
            core::assert_eq!(view.available_bytes(), 0);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_empty() {
        with_rx_view(&[], false, |view| {
            core::assert_eq!(view.available_bytes(), 0);
            core::assert_eq!(view.peek(), &[] as &[u8]);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_peek_and_available_bytes() {
        let data = b"peek me";
        with_rx_view(data, false, |view| {
            core::assert_eq!(view.available_bytes(), data.len());
            core::assert_eq!(view.peek(), data);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_peek_until_eof() {
        let data = b"eof test";
        with_rx_view(data, true, |view| {
            let (slice, eof) = view.peek_until_eof();
            core::assert_eq!(slice, data);
            core::assert!(eof);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_pop() {
        let data = b"pop this data";
        with_rx_view(data, false, |mut view| {
            let mut out = [0u8; 32];
            let popped = view.pop(&mut out);
            core::assert_eq!(popped, data.len());
            core::assert_eq!(&out[..data.len()], data);
            core::assert_eq!(view.available_bytes(), 0);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_consume_partial() {
        let data = b"0123456789";
        with_rx_view(data, false, |mut view| {
            core::assert_eq!(view.peek(), data);
            core::assert_eq!(view.consume(4), 4);
            core::assert_eq!(view.available_bytes(), data.len() - 4);
            core::assert_eq!(view.peek(), &data[4..]);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_consume_full_descriptor() {
        let data = [0xABu8; CHUNK_SIZE];
        with_rx_view(&data, false, |mut view| {
            core::assert_eq!(view.available_bytes(), CHUNK_SIZE);
            core::assert_eq!(view.consume(CHUNK_SIZE), CHUNK_SIZE);
            core::assert_eq!(view.available_bytes(), 0);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_peek_multiple_descriptors() {
        const SECOND_LEN: usize = 128;
        let first = [1u8; CHUNK_SIZE];
        let second = [2u8; SECOND_LEN];
        with_rx_view_multi(&[(&first, false), (&second, false)], |view| {
            core::assert_eq!(view.available_bytes(), CHUNK_SIZE + SECOND_LEN);
            let peeked = view.peek();
            core::assert_eq!(peeked.len(), CHUNK_SIZE + SECOND_LEN);
            core::assert_eq!(&peeked[..CHUNK_SIZE], &first);
            core::assert_eq!(&peeked[CHUNK_SIZE..], &second);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_view_pop_multiple_descriptors() {
        const SECOND_LEN: usize = 64;
        let first = [3u8; CHUNK_SIZE];
        let second = [4u8; SECOND_LEN];
        with_rx_view_multi(&[(&first, false), (&second, false)], |mut view| {
            let mut out = [0u8; CHUNK_SIZE + SECOND_LEN];
            let popped = view.pop(&mut out);
            core::assert_eq!(popped, CHUNK_SIZE + SECOND_LEN);
            core::assert_eq!(&out[..CHUNK_SIZE], &first);
            core::assert_eq!(&out[CHUNK_SIZE..], &second);
        });
    }

    #[test]
    fn test_dma_rx_stream_buf_from_view() {
        with_rx_view(b"round trip", false, |view| {
            let buf = <DmaRxStreamBuf as DmaRxBuffer>::from_view(view);
            let (descriptors, buffer) = buf.split();
            core::assert_eq!(buffer.len(), BUFFER_SIZE);
            core::assert_eq!(descriptors.len(), BUFFER_SIZE / CHUNK_SIZE);
        });
    }

    #[test]
    fn test_dma_tx_stream_buf_from_view() {
        with_tx_view(true, |view| {
            let buf = <DmaTxStreamBuf as DmaTxBuffer>::from_view(view);
            let (descriptors, buffer) = buf.split();
            core::assert_eq!(buffer.len(), BUFFER_SIZE);
            core::assert_eq!(descriptors.len(), BUFFER_SIZE / CHUNK_SIZE);
        });
    }

    #[test]
    fn test_dma_tx_stream_buf_initial() {
        with_tx_view(false, |view| {
            // initially all descriptors are owned by the DMA, so no space should be available
            core::assert_eq!(view.available_bytes(), 0);

            let buf = <DmaTxStreamBuf as DmaTxBuffer>::from_view(view);
            let (descriptors, buffer) = buf.split();

            // DMA processed first descriptor, now it should be owned by CPU
            descriptors[0].set_owner(Owner::Cpu);

            let buf = DmaTxStreamBuf::new(descriptors, buffer).unwrap();
            let mut view = buf.into_view();

            // the first descriptor is now owned by CPU, so its chunk should be available for
            // writing
            core::assert_eq!(view.available_bytes(), CHUNK_SIZE);

            // we can push more data
            core::assert_eq!(view.push(&[0u8; CHUNK_SIZE]), CHUNK_SIZE);

            // and after pushing, the available space should be reduced accordingly
            core::assert_eq!(view.available_bytes(), 0);
        });
    }

    #[test]
    fn test_dma_tx_stream_buf_prefill() {
        let mut buf = dma_tx_stream_buffer!(BUFFER_SIZE, CHUNK_SIZE);
        buf.push(&[0xffu8; CHUNK_SIZE * 2]);

        buf.prepare();

        // make sure initially all descriptors are owned by the DMA after prepare, even if we pushed
        // data before
        let (descriptors, _buffer) = buf.split();
        for desc in descriptors.iter() {
            core::assert!(matches!(desc.owner(), Owner::Dma));
        }
    }
}
