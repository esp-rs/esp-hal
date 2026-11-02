//! The ESP-DL functions that the speech features of ESP-SR import: the real FFT of `dl_fft` and a
//! dot product.

use core::{f32::consts::PI, ffi::c_int, ptr, slice};

use crate::memory::{allocate, boxed, deallocate};

const ESP_OK: c_int = 0;
const ESP_FAIL: c_int = -1;

/// `dl_fft_f32_t`. Only `dl_rfft_f32_*` read it, but the fields keep ESP-DL's layout.
#[repr(C)]
pub struct RealFft {
    fft_point: c_int,
    log2n: c_int,
    /// The twiddle factors of the complex FFT: `cos` and `-sin` of `2πk/(fft_point/2)`, for k in
    /// `0..fft_point/4`.
    fft_table: *mut f32,
    /// ESP-DL's table of the real FFT: `cos` and `sin` of `2πk/fft_point`, for k in
    /// `1..=fft_point/2`.
    rfft_table: *mut f32,
    bitrev_table: *mut u16,
    bitrev_size: c_int,
}

/// Creates a real FFT of `fft_point` samples, a power of two of at least 4, or returns null.
#[unsafe(no_mangle)]
extern "C" fn dl_rfft_f32_init(fft_point: c_int, _caps: u32) -> *mut RealFft {
    let Ok(n) = usize::try_from(fft_point) else {
        return ptr::null_mut();
    };
    if n < 4 || !n.is_power_of_two() {
        log::error!("FFT point must be a power of two of at least 4, not {fft_point}");
        return ptr::null_mut();
    }
    let fft_table = allocate(n / 2 * size_of::<f32>(), 16).cast::<f32>();
    let rfft_table = allocate(n * size_of::<f32>(), 16).cast::<f32>();
    let handle = boxed(RealFft {
        fft_point,
        log2n: n.trailing_zeros() as c_int,
        fft_table,
        rfft_table,
        bitrev_table: ptr::null_mut(),
        bitrev_size: 0,
    });
    if fft_table.is_null() || rfft_table.is_null() || handle.is_null() {
        // SAFETY: The pointers are null or new allocations.
        unsafe { dl_rfft_f32_deinit(handle) };
        if handle.is_null() {
            unsafe {
                deallocate(fft_table.cast());
                deallocate(rfft_table.cast());
            }
        }
        return ptr::null_mut();
    }
    // SAFETY: The tables have room for these entries.
    unsafe {
        for k in 0..n / 4 {
            let angle = 2.0 * PI * k as f32 / (n / 2) as f32;
            *fft_table.add(2 * k) = libm::cosf(angle);
            *fft_table.add(2 * k + 1) = -libm::sinf(angle);
        }
        // `dl_gen_rfft_table_f32`.
        for i in 1..=n / 2 {
            let angle = 2.0 * PI * i as f32 / n as f32;
            *rfft_table.add(2 * i - 2) = libm::cosf(angle);
            *rfft_table.add(2 * i - 1) = libm::sinf(angle);
        }
    }
    handle
}

#[unsafe(no_mangle)]
unsafe extern "C" fn dl_rfft_f32_deinit(handle: *mut RealFft) {
    if handle.is_null() {
        return;
    }
    // SAFETY: C callers pass a handle from `dl_rfft_f32_init`.
    unsafe {
        deallocate((*handle).fft_table.cast());
        deallocate((*handle).rfft_table.cast());
        deallocate(handle.cast());
    }
}

/// Transforms `fft_point` real samples in place into ESP-DL's spectrum layout: the real parts of
/// the DC and Nyquist bins, then the real and imaginary parts of bins 1 to `fft_point/2 - 1`.
#[unsafe(no_mangle)]
unsafe extern "C" fn dl_rfft_f32_run(handle: *const RealFft, data: *mut f32) -> c_int {
    if handle.is_null() || data.is_null() {
        return ESP_FAIL;
    }
    // SAFETY: C callers pass a handle from `dl_rfft_f32_init` and `fft_point` samples.
    unsafe {
        let handle = &*handle;
        let n = handle.fft_point as usize;
        let data = slice::from_raw_parts_mut(data, n);
        let fft_table = slice::from_raw_parts(handle.fft_table, n / 2);
        let rfft_table = slice::from_raw_parts(handle.rfft_table, n);
        complex_fft(data, fft_table);
        split(data, rfft_table);
    }
    ESP_OK
}

/// An in-place radix-2 FFT of `data.len() / 2` complex values (`e^-iωt`), in natural order.
fn complex_fft(data: &mut [f32], twiddles: &[f32]) {
    let n = data.len() / 2;
    // Bit reversal.
    let bits = n.trailing_zeros();
    for i in 0..n {
        let j = i.reverse_bits() >> (usize::BITS - bits);
        if i < j {
            data.swap(2 * i, 2 * j);
            data.swap(2 * i + 1, 2 * j + 1);
        }
    }
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let stride = n / size;
        for start in (0..n).step_by(size) {
            for k in 0..half {
                let (wr, wi) = (twiddles[2 * k * stride], twiddles[2 * k * stride + 1]);
                let (a, b) = (start + k, start + k + half);
                let (br, bi) = (data[2 * b], data[2 * b + 1]);
                let (tr, ti) = (wr * br - wi * bi, wr * bi + wi * br);
                let (ar, ai) = (data[2 * a], data[2 * a + 1]);
                data[2 * a] = ar + tr;
                data[2 * a + 1] = ai + ti;
                data[2 * b] = ar - tr;
                data[2 * b + 1] = ai - ti;
            }
        }
        size *= 2;
    }
}

/// Turns the FFT of the `n` complex values made of even and odd samples into the spectrum of the
/// `2n` real samples. A port of `dl_rfft_post_proc_fc32_ansi`.
fn split(data: &mut [f32], table: &[f32]) {
    let n = data.len() / 2;
    let (re0, im0) = (data[0], data[1]);
    data[0] = re0 + im0;
    data[1] = re0 - im0;
    for k in 1..=n / 2 {
        let (pk_re, pk_im) = (data[2 * k], data[2 * k + 1]);
        let (pnk_re, pnk_im) = (data[2 * (n - k)], data[2 * (n - k) + 1]);
        let (f1_re, f1_im) = (pk_re + pnk_re, pk_im - pnk_im);
        let (f2_re, f2_im) = (pk_re - pnk_re, pk_im + pnk_im);
        let c = -table[k * 2 - 1];
        let s = -table[k * 2 - 2];
        let tw_re = c * f2_re - s * f2_im;
        let tw_im = s * f2_re + c * f2_im;
        data[2 * k] = 0.5 * (f1_re + tw_re);
        data[2 * k + 1] = 0.5 * (f1_im + tw_im);
        data[2 * (n - k)] = 0.5 * (f1_re - tw_re);
        data[2 * (n - k) + 1] = 0.5 * (tw_im - f1_im);
    }
}

/// `dl::base::dotprod(float *, float *, float *, int, int)`. ESP-DL ignores `shift` for floats.
#[unsafe(no_mangle)]
unsafe extern "C" fn esp_sr_dotprod_f32(
    a: *const f32,
    b: *const f32,
    out: *mut f32,
    len: c_int,
    _shift: c_int,
) {
    // SAFETY: ESP-SR passes two vectors of `len` floats and one output float.
    unsafe {
        let a = slice::from_raw_parts(a, len as usize);
        let b = slice::from_raw_parts(b, len as usize);
        out.write(a.iter().zip(b).map(|(a, b)| a * b).sum());
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec::Vec;

    use super::*;

    /// The direct DFT, in ESP-DL's layout.
    fn reference(samples: &[f32]) -> Vec<f64> {
        let n = samples.len();
        let bin = |k: usize| {
            samples
                .iter()
                .enumerate()
                .fold((0.0f64, 0.0f64), |(re, im), (t, &x)| {
                    let angle = -2.0 * core::f64::consts::PI * (k * t) as f64 / n as f64;
                    (
                        re + f64::from(x) * angle.cos(),
                        im + f64::from(x) * angle.sin(),
                    )
                })
        };
        let mut out = std::vec![bin(0).0, bin(n / 2).0];
        for k in 1..n / 2 {
            let (re, im) = bin(k);
            out.extend([re, im]);
        }
        out
    }

    #[test]
    fn matches_the_dft() {
        for n in [4, 8, 64, 512] {
            let samples: Vec<f32> = (0..n)
                .map(|i| ((i * 7919 % 101) as f32 - 50.0) / 13.0)
                .collect();
            let handle = dl_rfft_f32_init(n as c_int, 0);
            let mut data = samples.clone();
            assert_eq!(
                unsafe { dl_rfft_f32_run(handle, data.as_mut_ptr()) },
                ESP_OK
            );
            unsafe { dl_rfft_f32_deinit(handle) };
            let scale = n as f64;
            for (i, (got, want)) in data.iter().zip(reference(&samples)).enumerate() {
                assert!(
                    (f64::from(*got) - want).abs() < 1e-4 * scale,
                    "n {n}, value {i}: {got} vs {want}"
                );
            }
        }
    }
}
