#[embedded_test::tests(default_timeout = 2)]
mod tests {
    use hil_test as _;

    #[init]
    fn init() {
        let _ = esp_hal::init(esp_hal::Config::default());
    }

    /// # Safety
    /// Caller must pass exactly `count` variadic arguments, each of type `i32`.
    unsafe extern "C" fn sum_of_n(count: i32, mut args: ...) -> i32 {
        // SAFETY: Caller guarantees exactly `count` i32 variadic args were
        // passed. Each `arg::<i32>()` reads the next i32 from the va_list.
        // i32 implements VaArgSafe (no C default argument promotion issues).
        unsafe {
            let mut sum = 0;
            for _ in 0..count {
                sum += args.arg::<i32>();
            }
            sum
        }
    }

    /// # Safety
    /// Caller must pass at least one variadic argument of type `i32`.
    unsafe extern "C" fn first_arg(mut args: ...) -> i32 {
        // SAFETY: Caller guarantees at least one i32 variadic arg was passed.
        unsafe { args.arg::<i32>() }
    }

    #[test]
    fn test_vaargs_sum() {
        // SAFETY: We pass exactly 3 i32 variadic args, matching count=3.
        let result = unsafe { sum_of_n(3, 10i32, 20i32, 30i32) };
        assert_eq!(result, 60);
    }

    #[test]
    fn test_vaargs_single() {
        // SAFETY: We pass one i32 variadic arg, satisfying first_arg's contract.
        let result = unsafe { first_arg(42i32) };
        assert_eq!(result, 42);
    }

    /// # Safety
    /// Caller must pass exactly `count` variadic arguments, each of type `i64`.
    unsafe extern "C" fn sum_i64(count: i32, mut args: ...) -> i64 {
        // SAFETY: Caller guarantees exactly `count` i64 variadic args were
        // passed. i64 implements VaArgSafe.
        unsafe {
            let mut sum: i64 = 0;
            for _ in 0..count {
                sum += args.arg::<i64>();
            }
            sum
        }
    }

    /// # Safety
    /// Caller must pass exactly `count` variadic arguments, each of type `f64`.
    unsafe extern "C" fn sum_f64(count: i32, mut args: ...) -> f64 {
        // SAFETY: Caller guarantees exactly `count` f64 variadic args were
        // passed. f64 implements VaArgSafe (C promotes float to double in
        // variadic calls, and f64 is double-width, so there is no mismatch).
        unsafe {
            let mut sum: f64 = 0.0;
            for _ in 0..count {
                sum += args.arg::<f64>();
            }
            sum
        }
    }

    /// # Safety
    /// Caller must pass one variadic argument of type `*const i32` that is
    /// non-null, properly aligned, and points to an initialized `i32`.
    unsafe extern "C" fn read_ptr(mut args: ...) -> i32 {
        unsafe {
            // SAFETY: Caller guarantees a *const i32 was passed as the
            // variadic arg. Raw pointers implement VaArgSafe.
            let p = args.arg::<*const i32>();
            // SAFETY: Caller guarantees `p` is non-null, properly aligned,
            // and points to an initialized i32 that is valid for reads.
            *p
        }
    }

    /// # Safety
    /// Caller must pass exactly three variadic arguments in order:
    /// `i32`, `i64`, `i32`.
    unsafe extern "C" fn mixed_args(mut args: ...) -> i64 {
        // SAFETY: Caller guarantees args are (i32, i64, i32) in that order.
        // Each type implements VaArgSafe. We read them in the exact order
        // and with the exact types they were passed.
        unsafe {
            let a = args.arg::<i32>() as i64;
            let b = args.arg::<i64>();
            let c = args.arg::<i32>() as i64;
            a + b + c
        }
    }

    #[test]
    fn test_vaargs_i64() {
        // SAFETY: We pass exactly 3 i64 variadic args, matching count=3.
        let result = unsafe { sum_i64(3, 0x1_0000_0000i64, 0x2_0000_0000i64, 0x3_0000_0000i64) };
        assert_eq!(result, 0x6_0000_0000i64);
    }

    #[test]
    fn test_vaargs_f64() {
        // SAFETY: We pass exactly 3 f64 variadic args, matching count=3.
        let result = unsafe { sum_f64(3, 1.5f64, 2.5f64, 3.0f64) };
        assert!((result - 7.0f64).abs() < f64::EPSILON);
    }

    #[test]
    fn test_vaargs_ptr() {
        let val: i32 = 99;
        // SAFETY: We pass a *const i32 derived from a reference to a local.
        // `val` is initialized, properly aligned, and lives for the duration
        // of the call.
        let result = unsafe { read_ptr(&val as *const i32) };
        assert_eq!(result, 99);
    }

    #[test]
    fn test_vaargs_mixed_types() {
        // SAFETY: We pass (i32, i64, i32) matching mixed_args' expected
        // type sequence exactly.
        let result = unsafe { mixed_args(1i32, 0x1_0000_0000i64, 2i32) };
        assert_eq!(result, 0x1_0000_0003i64);
    }

    #[derive(Clone, Copy, PartialEq, Debug)]
    #[repr(C)]
    struct BigStruct {
        tag: u32,
        data: [u8; 128],
    }

    /// # Safety
    /// Caller must pass one variadic argument of type `*const BigStruct` that
    /// is non-null, properly aligned, and points to an initialized `BigStruct`.
    unsafe extern "C" fn read_big_struct(mut args: ...) -> BigStruct {
        unsafe {
            // SAFETY: Caller guarantees a *const BigStruct was passed.
            let p = args.arg::<*const BigStruct>();
            // SAFETY: Caller guarantees `p` is non-null, aligned, and points
            // to an initialized BigStruct. BigStruct is Copy, so this
            // bitwise copy through a raw pointer is sound.
            *p
        }
    }

    #[test]
    fn test_vaargs_big_struct() {
        let mut input = BigStruct {
            tag: 0xDEAD_BEEF,
            data: [0; 128],
        };
        for (i, byte) in input.data.iter_mut().enumerate() {
            *byte = i as u8;
        }
        // SAFETY: We pass a *const BigStruct derived from a reference to a
        // local. `input` is initialized, #[repr(C)], properly aligned, and
        // lives for the duration of the call.
        let result = unsafe { read_big_struct(&input as *const BigStruct) };
        assert_eq!(result, input);
    }

    #[repr(C)]
    struct KitchenSinkResult {
        a: i32,
        b: f64,
        s: BigStruct,
        c: i64,
        d: i32,
        e: i32,
        f: f64,
    }

    /// # Safety
    /// - `out` must be non-null, properly aligned for `KitchenSinkResult`, and valid for writes
    ///   (need not be initialized).
    /// - Variadic args must be passed in this exact order: `i32`, `f64`, `*const BigStruct`
    ///   (valid), `i64`, `*const i32` (valid), `i32`, `f64`.
    unsafe extern "C" fn kitchen_sink(out: *mut KitchenSinkResult, mut args: ...) {
        unsafe {
            // SAFETY: Each arg::<T>() reads the next variadic argument as
            // type T. Caller guarantees the types and order match exactly.
            // All scalar types (i32, i64, f64) and pointer types implement
            // VaArgSafe. Writing through `out` is safe because caller
            // guarantees it is valid for writes and properly aligned.
            (*out).a = args.arg::<i32>();
            (*out).b = args.arg::<f64>();
            // SAFETY: Caller guarantees this pointer is non-null, aligned,
            // and points to an initialized BigStruct. BigStruct is Copy.
            (*out).s = *args.arg::<*const BigStruct>();
            (*out).c = args.arg::<i64>();
            // SAFETY: Caller guarantees this pointer is non-null, aligned,
            // and points to an initialized i32.
            (*out).d = *args.arg::<*const i32>();
            (*out).e = args.arg::<i32>();
            (*out).f = args.arg::<f64>();
        }
    }

    #[test]
    fn test_vaargs_kitchen_sink() {
        let big = BigStruct {
            tag: 0xDEAD_BEEF,
            data: {
                let mut d = [0u8; 128];
                d[0] = 10;
                d[127] = 20;
                d
            },
        };
        let pointee: i32 = 500;

        let mut result = core::mem::MaybeUninit::<KitchenSinkResult>::uninit();
        // SAFETY:
        // - `result.as_mut_ptr()` yields a non-null, properly aligned pointer (guaranteed by
        //   MaybeUninit's layout), valid for writes.
        // - Variadic args match kitchen_sink's expected types in order: 1i32, 2.5f64, *const
        //   BigStruct, 0x1_0000_0000i64, *const i32, 3i32, 4.25f64.
        // - Both pointers (&big, &pointee) are derived from references to initialized locals that
        //   outlive this call.
        // - After return, all 7 fields of KitchenSinkResult have been written.
        unsafe {
            kitchen_sink(
                result.as_mut_ptr(),
                1i32,
                2.5f64,
                &big as *const BigStruct,
                0x1_0000_0000i64,
                &pointee as *const i32,
                3i32,
                4.25f64,
            );
        }
        // SAFETY: kitchen_sink writes to every field of KitchenSinkResult
        // (a, b, s, c, d, e, f) through the out pointer before returning,
        // so the entire value is now initialized.
        let result = unsafe { result.assume_init() };

        assert_eq!(result.a, 1);
        assert!((result.b - 2.5f64).abs() < f64::EPSILON);
        assert_eq!(result.s.tag, 0xDEAD_BEEF);
        assert_eq!(result.s.data[0], 10);
        assert_eq!(result.s.data[127], 20);
        assert_eq!(result.c, 0x1_0000_0000i64);
        assert_eq!(result.d, 500);
        assert_eq!(result.e, 3);
        assert!((result.f - 4.25f64).abs() < f64::EPSILON);
    }

    #[test]
    fn test_va_copy() {
        /// # Safety
        /// - `out` must be non-null, properly aligned, and valid for writes.
        /// - Caller must pass exactly `count` variadic arguments, each `i32`.
        unsafe extern "C" fn sum_and_copy_sum(out: *mut i32, count: i32, mut args: ...) -> i32 {
            unsafe {
                // SAFETY: Clone on VaListImpl performs va_copy, producing an
                // independent copy of the va_list state. Both the original
                // and clone track their read position independently, so
                // iterating one does not affect the other. The clone is
                // automatically va_end'd when dropped.
                let mut copy = args.clone();

                // SAFETY: Caller guarantees exactly `count` i32 variadic args
                // were passed. We iterate exactly `count` times on both the
                // original and the cloned va_list.
                let mut sum1 = 0;
                for _ in 0..count {
                    sum1 += args.arg::<i32>();
                }

                let mut sum2 = 0;
                for _ in 0..count {
                    sum2 += copy.arg::<i32>();
                }

                // SAFETY: Caller guarantees `out` is valid for writes.
                *out = sum2;
                sum1
            }
        }

        let mut copy_sum: i32 = 0;
        // SAFETY:
        // - `&mut copy_sum` is non-null, aligned, and valid for writes.
        // - We pass count=3 and exactly 3 i32 variadic args.
        let sum = unsafe { sum_and_copy_sum(&mut copy_sum, 3, 1i32, 2i32, 3i32) };
        assert_eq!(sum, 6);
        assert_eq!(copy_sum, 6);
    }
}
