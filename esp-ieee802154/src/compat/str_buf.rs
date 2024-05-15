use core::fmt::Write;

pub struct StrBuf {
    buffer: [u8; 512],
    len: usize,
}

impl StrBuf {
    pub fn new() -> StrBuf {
        StrBuf {
            buffer: [0u8; 512],
            len: 0,
        }
    }

    pub unsafe fn from(c_str: *const u8) -> StrBuf {
        let mut res = StrBuf {
            buffer: [0u8; 512],
            len: 0,
        };

        let mut idx: usize = 0;
        while *(c_str.add(idx)) != 0 {
            res.buffer[idx] = *(c_str.add(idx));
            idx += 1;
        }

        res.len = idx;
        res
    }

    pub fn append(&mut self, s: &str) {
        let mut idx: usize = self.len;
        s.chars().for_each(|c| {
            self.buffer[idx] = c as u8;
            idx += 1;
        });
        self.len = idx;
    }

    pub fn append_char(&mut self, c: char) {
        let mut idx: usize = self.len;
        self.buffer[idx] = c as u8;
        idx += 1;
        self.len = idx;
    }

    pub unsafe fn as_str_ref(&self) -> &str {
        core::str::from_utf8_unchecked(&self.buffer[..self.len])
    }
}

impl Write for StrBuf {
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        self.append(s);
        Ok(())
    }
}
