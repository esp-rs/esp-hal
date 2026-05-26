// CSR_GPIO_OEN_USER   0x803
// CSR_GPIO_IN_USER    0x804
// CSR_GPIO_OUT_USER   0x805

#[inline(always)]
pub(super) fn set_output_enabled(mask: u32, en: bool) {
    riscv::read_csr!(0x803);
    riscv::write_csr!(0x803);

    unsafe {
        let bits = _read();
        if en {
            _write(bits | mask as usize)
        } else {
            _write(bits & !mask as usize)
        }
    }
}

#[inline(always)]
pub(super) fn read_in() -> u32 {
    riscv::read_csr!(0x804);
    unsafe { _read() as u32 }
}

#[inline(always)]
pub(super) fn read_out() -> u32 {
    riscv::read_csr!(0x805);
    unsafe { _read() as u32 }
}

#[inline(always)]
pub(super) fn write(mask: u32, value: u32) {
    riscv::set!(0x805);
    riscv::clear!(0x805);
    unsafe {
        _set((mask & value) as usize);
        _clear((mask & !value) as usize);
    }
}
