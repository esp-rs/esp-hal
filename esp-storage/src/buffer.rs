use core::{mem::MaybeUninit, slice};

#[cfg(feature = "nor-flash")]
pub fn uninit_slice(bytes: &[u8]) -> &[MaybeUninit<u8>] {
    unsafe { core::mem::transmute(bytes) }
}

#[cfg(feature = "nor-flash")]
pub fn uninit_slice_mut(bytes: &mut [u8]) -> &mut [MaybeUninit<u8>] {
    unsafe { core::mem::transmute(bytes) }
}

pub type FlashWordBuffer = FlashBuffer<4, 1>;
pub type FlashSectorBuffer = FlashBuffer<4096, 1024>;

#[repr(C)]
pub union FlashBuffer<const N: usize, const M: usize> {
    bytes: [MaybeUninit<u8>; N],
    words: [MaybeUninit<u32>; M],
}

impl<const N: usize, const M: usize> FlashBuffer<N, M> {
    pub const fn uninit() -> Self {
        assert!(N == M * 4);
        Self {
            words: [MaybeUninit::uninit(); M],
        }
    }

    pub fn as_bytes(&self) -> &[MaybeUninit<u8>] {
        unsafe { self.bytes.as_ref() }
    }

    pub fn as_bytes_mut(&mut self) -> &mut [MaybeUninit<u8>] {
        unsafe { self.bytes.as_mut() }
    }

    pub fn as_words(&self) -> &[MaybeUninit<u32>] {
        unsafe { self.words.as_ref() }
    }

    pub fn as_words_mut(&mut self) -> &mut [MaybeUninit<u32>] {
        unsafe { self.words.as_mut() }
    }

    pub unsafe fn assume_init_bytes(&self) -> &[u8] {
        slice::from_raw_parts(self.bytes.as_ptr() as *const u8, self.bytes.len())
    }

    pub unsafe fn assume_init_bytes_mut(&mut self) -> &mut [u8] {
        slice::from_raw_parts_mut(self.bytes.as_mut_ptr() as *mut u8, self.bytes.len())
    }

    pub unsafe fn assume_init_words(&self) -> &[u32] {
        slice::from_raw_parts(self.words.as_ptr() as *const u32, self.words.len())
    }

    pub unsafe fn assume_init_words_mut(&mut self) -> &mut [u32] {
        slice::from_raw_parts_mut(self.words.as_mut_ptr() as *mut u32, self.words.len())
    }
}
