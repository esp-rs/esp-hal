use core::ptr;

const DESC_OWN: u32 = 1 << 31;
const DESC_FS: u32 = 1 << 29;
const DESC_LS: u32 = 1 << 28;

#[repr(C, align(16))]
pub struct DmaDesc {
    pub status: u32,
    pub length: u32,
    pub buffer: *mut u8,
    pub next: *mut DmaDesc,
}

#[repr(C, align(16))]
pub struct DmaRing<const N: usize> {
    desc: [DmaDesc; N],
    buffers: [[u8; 1536]; N],
    index: usize,
}

impl<const N: usize> DmaRing<N> {
    pub const fn new() -> Self {
        const EMPTY: DmaDesc = DmaDesc {
            status: 0,
            length: 0,
            buffer: ptr::null_mut(),
            next: ptr::null_mut(),
        };

        Self {
            desc: [EMPTY; N],
            buffers: [[0u8; 1536]; N],
            index: 0,
        }
    }

    pub fn init(&mut self, is_rx: bool) {
        for i in 0..N {
            self.desc[i].buffer = self.buffers[i].as_mut_ptr();
            self.desc[i].next = &mut self.desc[(i + 1) % N];
            self.desc[i].status = if is_rx { DESC_OWN } else { 0 };
            self.desc[i].length = 1536;
        }
    }

    #[inline(always)]
    pub fn desc_ptr(&mut self) -> *mut DmaDesc {
        self.desc.as_mut_ptr()
    }

    fn current(&mut self) -> &mut DmaDesc {
        &mut self.desc[self.index]
    }

    fn advance(&mut self) {
        self.index = (self.index + 1) % N;
    }

    pub fn prepare_tx(&mut self, data: &[u8]) -> bool {
        let d = self.current();

        if d.status & DESC_OWN != 0 {
            return false;
        }

        unsafe {
            core::ptr::copy_nonoverlapping(data.as_ptr(), d.buffer, data.len());
        }

        d.length = data.len() as u32;
        d.status = DESC_OWN | DESC_FS | DESC_LS;

        self.advance();
        true
    }

    pub fn try_recv(&mut self) -> Option<&[u8]> {
        let d = self.current();

        if d.status & DESC_OWN != 0 {
            return None;
        }

        let len = d.length as usize;

        let slice = unsafe { core::slice::from_raw_parts(d.buffer, len) };

        d.status = DESC_OWN;

        self.advance();
        Some(slice)
    }
}
