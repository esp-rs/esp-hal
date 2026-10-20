//! GCC/LLVM libatomic helpers for targets without hardware CAS.
//!
//! `wpa_supplicant` uses C11 atomics (`eloop.c`). rustc's `compiler_builtins`
//! does not export `__atomic_*` on `riscv32imc` or `xtensa-esp32s2`.
//! Implemented with [`portable_atomic`].

use portable_atomic::{AtomicU8, AtomicU16, AtomicU32, Ordering};

fn ordering(order: i32) -> Ordering {
    match order {
        0 => Ordering::Relaxed,
        1 | 2 => Ordering::Acquire,
        3 => Ordering::Release,
        4 => Ordering::AcqRel,
        _ => Ordering::SeqCst,
    }
}

fn failure_ordering(order: i32) -> Ordering {
    match order {
        0 => Ordering::Relaxed,
        1 | 2 => Ordering::Acquire,
        _ => Ordering::SeqCst,
    }
}

macro_rules! libatomic {
    ($ty:ty, $atomic:ty, $cas:ident, $xchg:ident, $add:ident, $sub:ident, $load:ident, $store:ident) => {
        /// GCC 6-operand CAS; `weak` selects `compare_exchange_weak`.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $cas(
            dst: *mut $ty,
            expected: *mut $ty,
            desired: $ty,
            weak: bool,
            success: i32,
            failure: i32,
        ) -> bool {
            let atomic = unsafe { <$atomic>::from_ptr(dst) };
            let exp = unsafe { core::ptr::read(expected) };
            let success = ordering(success);
            let failure = failure_ordering(failure);
            let result = if weak {
                atomic.compare_exchange_weak(exp, desired, success, failure)
            } else {
                atomic.compare_exchange(exp, desired, success, failure)
            };
            match result {
                Ok(_) => true,
                Err(current) => {
                    unsafe { core::ptr::write(expected, current) };
                    false
                }
            }
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $xchg(dst: *mut $ty, val: $ty, order: i32) -> $ty {
            unsafe { <$atomic>::from_ptr(dst) }.swap(val, ordering(order))
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $add(dst: *mut $ty, val: $ty, order: i32) -> $ty {
            unsafe { <$atomic>::from_ptr(dst) }.fetch_add(val, ordering(order))
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $sub(dst: *mut $ty, val: $ty, order: i32) -> $ty {
            unsafe { <$atomic>::from_ptr(dst) }.fetch_sub(val, ordering(order))
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $load(src: *const $ty, order: i32) -> $ty {
            unsafe { <$atomic>::from_ptr(src.cast_mut()) }.load(ordering(order))
        }

        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $store(dst: *mut $ty, val: $ty, order: i32) {
            unsafe { <$atomic>::from_ptr(dst) }.store(val, ordering(order))
        }
    };
}

libatomic!(
    u8,
    AtomicU8,
    __atomic_compare_exchange_1,
    __atomic_exchange_1,
    __atomic_fetch_add_1,
    __atomic_fetch_sub_1,
    __atomic_load_1,
    __atomic_store_1
);
libatomic!(
    u16,
    AtomicU16,
    __atomic_compare_exchange_2,
    __atomic_exchange_2,
    __atomic_fetch_add_2,
    __atomic_fetch_sub_2,
    __atomic_load_2,
    __atomic_store_2
);
libatomic!(
    u32,
    AtomicU32,
    __atomic_compare_exchange_4,
    __atomic_exchange_4,
    __atomic_fetch_add_4,
    __atomic_fetch_sub_4,
    __atomic_load_4,
    __atomic_store_4
);
