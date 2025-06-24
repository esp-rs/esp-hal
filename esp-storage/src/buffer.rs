use core::{mem::MaybeUninit, slice};

pub fn uninit_slice(bytes: &[u8]) -> &[MaybeUninit<u8>] {
    unsafe { core::slice::from_raw_parts(bytes.as_ptr() as *const _, bytes.len()) }
}

pub fn uninit_slice_mut(bytes: &mut [u8]) -> &mut [MaybeUninit<u8>] {
    unsafe { core::slice::from_raw_parts_mut(bytes.as_mut_ptr() as *mut _, bytes.len()) }
}

pub type FlashWordBuffer = FlashBuffer<4>;

pub type FlashSectorBuffer = FlashBuffer<4096>;

#[repr(C, align(4))]
pub union FlashBuffer<const N: usize> {
    bytes: [MaybeUninit<u8>; N],
}

impl<const N: usize> FlashBuffer<N> {
    pub const fn uninit() -> Self {
        Self {
            bytes: [MaybeUninit::uninit(); N],
        }
    }

    pub fn as_bytes_mut(&mut self) -> &mut [MaybeUninit<u8>] {
        unsafe { self.bytes.as_mut() }
    }

    pub unsafe fn assume_init_bytes(&self) -> &[u8] {
        unsafe { slice::from_raw_parts(self.bytes.as_ptr() as *const _, self.bytes.len()) }
    }

    pub unsafe fn assume_init_bytes_mut(&mut self) -> &mut [u8] {
        unsafe { slice::from_raw_parts_mut(self.bytes.as_mut_ptr() as *mut _, self.bytes.len()) }
    }
}
