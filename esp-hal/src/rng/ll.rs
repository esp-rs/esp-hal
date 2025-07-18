use crate::{clock::Clocks, peripherals::RNG, sync::RawMutex};

static RNG_IN_USE: RawMutex = RawMutex::new();

// TODO: find a better place for this
#[inline]
fn current_cpu_cycles() -> usize {
    cfg_if::cfg_if! {
        if #[cfg(xtensa)] {
            xtensa_lx::timer::get_cycle_count() as usize
        } else if #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))] {
            riscv::register::mcycle::read()
        } else {
            const PRV_M: usize = 3;
            macro_rules! read_csr_fn {
                ($fnname:ident, $csr:literal) => {
                    #[inline]
                    fn $fnname() -> usize {
                        riscv::read_csr!($csr);

                        unsafe { _read() }
                    }
                }
            }

            read_csr_fn!(read_prv_mode, 0x810);
            read_csr_fn!(read_pccr_machine, 0x712);
            read_csr_fn!(read_pccr_user, 0x802);

            if read_prv_mode() == PRV_M {
                read_pccr_machine()
            } else {
                read_pccr_user()
            }
        }
    }
}

fn fill_ptr_range_inner(data: *mut u8, len: usize) {
    let clocks = Clocks::get();
    let cpu_to_apb_freq_ratio = clocks.cpu_clock / clocks.apb_clock;
    let wait_cycles = cpu_to_apb_freq_ratio as usize * property!("rng.apb_cycle_wait_num");

    let mut remaining = len;
    let mut dst = data;
    let mut last_wait_start = current_cpu_cycles();
    while remaining > 0 {
        // wait
        loop {
            let now = current_cpu_cycles();
            if now - last_wait_start >= wait_cycles {
                last_wait_start = now;
                break;
            }
        }

        // read
        let random_bytes = RNG::regs().data().read().bits().to_le_bytes();
        let bytes_to_copy = random_bytes.len().min(remaining);
        unsafe {
            dst.copy_from_nonoverlapping(random_bytes.as_ptr(), bytes_to_copy);
            dst = dst.add(bytes_to_copy);
        }
        remaining -= bytes_to_copy;
    }
}

// TODO: we should also provide a non-blocking function
pub(super) fn fill_ptr_range(data: *mut u8, len: usize) {
    RNG_IN_USE.lock(|| fill_ptr_range_inner(data, len))
}
