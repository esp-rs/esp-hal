use esp_sync::NonReentrantMutex;

use crate::{clock::Clocks, peripherals::RNG};

// TODO: find a better place for these
#[inline]
#[cfg(soc_cpu_has_prv_mode)]
fn tee_enabled() -> bool {
    false
}

#[inline]
fn current_cpu_cycles() -> usize {
    cfg_if::cfg_if! {
        if #[cfg(xtensa)] {
            xtensa_lx::timer::get_cycle_count() as usize
        } else if #[cfg(soc_cpu_has_csr_pc)] {
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

            read_csr_fn!(read_pccr_machine, 0x7e2);
            read_csr_fn!(read_pccr_user, 0x802);

            fn read_prv_mode() -> usize {
                #[cfg(soc_cpu_has_prv_mode)]
                if tee_enabled() {
                    riscv::read_csr!(0x810);
                    return unsafe { _read() };
                }

                PRV_M
            }

            if read_prv_mode() == PRV_M {
                read_pccr_machine()
            } else {
                read_pccr_user()
            }
        } else {
            riscv::register::mcycle::read()
        }
    }
}

static LAST_READ: NonReentrantMutex<usize> = NonReentrantMutex::new(0);

fn read_one(wait_cycles: usize) -> u32 {
    loop {
        let random = LAST_READ.with(|last_wait_start| {
            let now: usize = current_cpu_cycles();
            if now.wrapping_sub(*last_wait_start) >= wait_cycles {
                *last_wait_start = now;
                Some(RNG::regs().data().read().bits())
            } else {
                None
            }
        });
        if let Some(random) = random {
            return random;
        }
    }
}

pub(super) fn fill_ptr_range(data: *mut u8, len: usize) {
    let clocks = Clocks::get();
    let cpu_to_apb_freq_ratio = clocks.cpu_clock / clocks.apb_clock;
    let wait_cycles = cpu_to_apb_freq_ratio as usize * property!("rng.apb_cycle_wait_num");

    let mut remaining = len;
    let mut dst = data;
    while remaining > 0 {
        let random_bytes = read_one(wait_cycles).to_le_bytes();
        let bytes_to_copy = random_bytes.len().min(remaining);
        unsafe {
            dst.copy_from_nonoverlapping(random_bytes.as_ptr(), bytes_to_copy);
            dst = dst.add(bytes_to_copy);
        }
        remaining -= bytes_to_copy;
    }
}
