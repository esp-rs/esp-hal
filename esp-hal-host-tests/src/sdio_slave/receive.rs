include!("../../../esp-hal/src/sdio_slave/receive.rs");

#[cfg(test)]
mod tests {
    use super::*;

    fn receive() -> Receive {
        Receive::new(
            Box::leak(Box::new(Descriptor::new())),
            Box::leak(Box::new([0u32; 128])),
        )
        .unwrap()
    }

    fn free(rx: Receive) {
        // These tests never start DMA. Reclaim every simulated allocation so
        // Miri can check both aliasing and leaks.
        let (descriptor, buffer) = unsafe { rx.into_buffers() };
        unsafe {
            drop(Box::from_raw(descriptor));
            drop(Box::from_raw(buffer));
        }
    }

    // Emulates DMA write-back, without constructing references to owned RAM.
    fn write_back(rx: &Receive, word: u32) {
        unsafe {
            ptr::write_volatile(
                ptr::addr_of_mut!((*rx.descriptor.as_ptr()).dw0).cast::<u32>(),
                word,
            );
        }
    }

    #[test]
    fn buffer_is_unavailable_until_valid_completion() {
        let mut rx = receive();
        assert!(rx.received().is_none());
        for i in 0..64 {
            unsafe {
                rx.buffer
                    .as_ptr()
                    .cast::<u8>()
                    .add(i)
                    .write(0xa0u8.wrapping_add(i as u8));
            }
        }
        write_back(&rx, (1 << 30) | (64 << 14) | 512);
        assert_eq!(unsafe { rx.finish() }, Ok(64));
        let bytes = rx.received().unwrap();
        assert_eq!(bytes.len(), 64);
        assert_eq!(bytes[0], 0xa0);
        assert_eq!(bytes[63], 0xdf);
        free(rx);
    }

    #[test]
    fn malformed_completion_never_exposes_memory() {
        for word in [
            (1 << 31) | (1 << 30) | (64 << 14) | 512,
            (64 << 14) | 512,
            (1 << 30) | (516 << 14) | 512,
            (1 << 30) | (64 << 14) | 508,
        ] {
            let mut rx = receive();
            write_back(&rx, word);
            assert_eq!(unsafe { rx.finish() }, Err(Error::InvalidDescriptor));
            assert!(rx.received().is_none());
            free(rx);
        }
    }

    #[test]
    fn descriptor_layout_and_explicit_restitution() {
        assert_eq!(core::mem::size_of::<Descriptor>(), 12);
        assert_eq!(core::mem::align_of::<Descriptor>(), 4);
        let rx = receive();
        assert_eq!(rx.descriptor_word(), (1 << 31) | 512);
        let buffer_pointer = rx.buffer.as_ptr();
        // No real DMA is started by these host tests.
        let (descriptor, buffer) = unsafe { rx.into_buffers() };
        assert_eq!(buffer.as_mut_ptr(), buffer_pointer);
        assert_eq!(buffer.len(), 128);
        let rx = Receive::new(descriptor, buffer).unwrap();
        assert!(rx.received().is_none());
        free(rx);
    }

    #[test]
    fn invalid_buffer_sizes_return_original_storage_unchanged() {
        for words in [0, 4096] {
            let descriptor = Box::leak(Box::new(Descriptor::new()));
            let descriptor_pointer = descriptor as *mut Descriptor;
            let buffer = vec![0x11223344; words].leak();
            let buffer_pointer = buffer.as_mut_ptr();
            let error = Receive::new(descriptor, buffer).err().unwrap();
            assert_eq!(error.error(), Error::InvalidBuffer);
            assert!(!format!("{error:?}").contains("11223344"));
            let (code, descriptor, buffer) = error.into_parts();
            assert_eq!(code, Error::InvalidBuffer);
            assert_eq!(descriptor as *mut Descriptor, descriptor_pointer);
            assert_eq!(buffer.as_mut_ptr(), buffer_pointer);
            assert!(buffer.iter().all(|word| *word == 0x11223344));
            unsafe {
                assert_eq!(*descriptor.dw0.get(), 0);
                drop(Box::from_raw(descriptor));
                drop(Box::from_raw(buffer));
            }
        }
    }

    #[test]
    fn transmit_descriptor_sets_eof_length_and_preserves_word_aligned_capacity() {
        for length in [1usize, 63, 64, 65, 0x3ffc] {
            let descriptor = Box::leak(Box::new(Descriptor::new()));
            let buffer = vec![0x43424140u32; length.div_ceil(4)].leak();
            let capacity = buffer.len() * 4;
            let mut tx = Transmit::new(descriptor, buffer, length).unwrap();
            assert_eq!(
                tx.storage.descriptor_word(),
                (1 << 31) | (1 << 30) | ((length as u32) << 14) | capacity as u32
            );
            // Emulate DMA owner write-back after EOF and PARK.
            write_back(
                &tx.storage,
                (1 << 30) | ((length as u32) << 14) | capacity as u32,
            );
            assert_eq!(unsafe { tx.finish() }, Ok(length));
            let (descriptor, buffer) = unsafe { tx.into_buffers() };
            assert!(buffer.iter().all(|word| *word == 0x43424140));
            unsafe {
                drop(Box::from_raw(descriptor));
                drop(Box::from_raw(buffer));
            }
        }
    }

    #[test]
    fn transmit_rejects_changed_length_and_dma_owned_or_missing_eof() {
        for word in [
            (1 << 31) | (1 << 30) | (64 << 14) | 64,
            (64 << 14) | 64,
            (1 << 30) | (63 << 14) | 64,
            (1 << 30) | (64 << 14) | 60,
        ] {
            let mut tx = Transmit::new(
                Box::leak(Box::new(Descriptor::new())),
                Box::leak(Box::new([0u32; 16])),
                64,
            )
            .unwrap();
            write_back(&tx.storage, word);
            assert_eq!(unsafe { tx.finish() }, Err(Error::InvalidDescriptor));
            free(tx.storage);
        }
    }

    #[test]
    fn invalid_transmit_length_returns_storage_without_descriptor_or_payload_changes() {
        for length in [0, 65, usize::MAX] {
            let descriptor = Box::leak(Box::new(Descriptor::new()));
            let buffer = Box::leak(Box::new([0xdeadbeefu32; 16]));
            let error = Transmit::new(descriptor, buffer, length).err().unwrap();
            let (error, descriptor, buffer) = error.into_parts();
            assert_eq!(error, Error::InvalidBuffer);
            assert_eq!(unsafe { *descriptor.dw0.get() }, 0);
            assert_eq!(buffer, &[0xdeadbeef; 16]);
            unsafe {
                drop(Box::from_raw(descriptor));
                drop(Box::from_raw(buffer));
            }
        }
    }
}
