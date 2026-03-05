#[embedded_test::tests(default_timeout = 2)]
mod tests {
    use hil_test as _;

    #[init]
    fn init() {
        let _ = esp_hal::init(esp_hal::Config::default());
    }

    unsafe extern "C" fn sum_of_n(count: i32, mut args: ...) -> i32 {
        unsafe {
            let mut sum = 0;
            for _ in 0..count {
                sum += args.arg::<i32>();
            }
            sum
        }
    }

    unsafe extern "C" fn first_arg(mut args: ...) -> i32 {
        unsafe { args.arg::<i32>() }
    }

    #[test]
    fn test_vaargs_sum() {
        let result = unsafe { sum_of_n(3, 10i32, 20i32, 30i32) };
        assert_eq!(result, 60);
    }

    #[test]
    fn test_vaargs_single() {
        let result = unsafe { first_arg(42i32) };
        assert_eq!(result, 42);
    }

    unsafe extern "C" fn sum_i64(count: i32, mut args: ...) -> i64 {
        unsafe {
            let mut sum: i64 = 0;
            for _ in 0..count {
                sum += args.arg::<i64>();
            }
            sum
        }
    }

    unsafe extern "C" fn sum_f64(count: i32, mut args: ...) -> f64 {
        unsafe {
            let mut sum: f64 = 0.0;
            for _ in 0..count {
                sum += args.arg::<f64>();
            }
            sum
        }
    }

    unsafe extern "C" fn read_ptr(mut args: ...) -> i32 {
        unsafe {
            let p = args.arg::<*const i32>();
            *p
        }
    }

    unsafe extern "C" fn mixed_args(mut args: ...) -> i64 {
        unsafe {
            let a = args.arg::<i32>() as i64;
            let b = args.arg::<i64>();
            let c = args.arg::<i32>() as i64;
            a + b + c
        }
    }

    #[test]
    fn test_vaargs_i64() {
        let result = unsafe { sum_i64(3, 0x1_0000_0000i64, 0x2_0000_0000i64, 0x3_0000_0000i64) };
        assert_eq!(result, 0x6_0000_0000i64);
    }

    #[test]
    fn test_vaargs_f64() {
        let result = unsafe { sum_f64(3, 1.5f64, 2.5f64, 3.0f64) };
        assert!((result - 7.0f64).abs() < f64::EPSILON);
    }

    #[test]
    fn test_vaargs_ptr() {
        let val: i32 = 99;
        let result = unsafe { read_ptr(&val as *const i32) };
        assert_eq!(result, 99);
    }

    #[test]
    fn test_vaargs_mixed_types() {
        let result = unsafe { mixed_args(1i32, 0x1_0000_0000i64, 2i32) };
        assert_eq!(result, 0x1_0000_0003i64);
    }

    #[derive(Clone, Copy, PartialEq, Debug)]
    #[repr(C)]
    struct BigStruct {
        tag: u32,
        data: [u8; 128],
    }

    unsafe extern "C" fn read_big_struct(mut args: ...) -> BigStruct {
        unsafe {
            let p = args.arg::<*const BigStruct>();
            *p
        }
    }

    #[test]
    fn test_vaargs_big_struct() {
        let mut input = BigStruct {
            tag: 0xDEAD_BEEF,
            data: [0; 128],
        };
        // Fill with a recognizable pattern
        for (i, byte) in input.data.iter_mut().enumerate() {
            *byte = i as u8;
        }
        let result = unsafe { read_big_struct(&input as *const BigStruct) };
        assert_eq!(result, input);
    }

    #[test]
    fn test_va_copy() {
        unsafe extern "C" fn sum_and_copy_sum(out: *mut i32, count: i32, mut args: ...) -> i32 {
            unsafe {
                let mut copy = args.clone();

                let mut sum1 = 0;
                for _ in 0..count {
                    sum1 += args.arg::<i32>();
                }

                let mut sum2 = 0;
                for _ in 0..count {
                    sum2 += copy.arg::<i32>();
                }

                *out = sum2;
                sum1
            }
        }

        let mut copy_sum: i32 = 0;
        let sum = unsafe { sum_and_copy_sum(&mut copy_sum, 3, 1i32, 2i32, 3i32) };
        assert_eq!(sum, 6);
        assert_eq!(copy_sum, 6);
    }
}
