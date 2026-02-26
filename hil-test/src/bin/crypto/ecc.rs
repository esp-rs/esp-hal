#[embedded_test::tests(default_timeout = 10)]
mod tests {
    use core::ops::Mul;

    use crypto_bigint::{
        Encoding,
        modular::runtime_mod::{DynResidue, DynResidueParams},
    };
    use elliptic_curve::{
        Curve,
        sec1::{ModulusSize, ToEncodedPoint},
    };
    #[cfg(rng_trng_supported)]
    use esp_hal::rng::TrngSource;
    use esp_hal::{
        Blocking,
        clock::CpuClock,
        ecc::{Ecc, EllipticCurve, OperationError},
        rng::Rng,
    };
    use hex_literal::hex;
    use primeorder::{AffinePoint, PrimeCurveParams};

    struct Context<'a> {
        ecc: Ecc<'a, Blocking>,
        #[cfg(rng_trng_supported)]
        _rng_source: TrngSource<'a>,
    }

    #[init]
    fn init() -> Context<'static> {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);

        Context {
            ecc: Ecc::new(p.ECC),
            #[cfg(rng_trng_supported)]
            _rng_source: TrngSource::new(p.RNG, p.ADC1),
        }
    }

    fn for_each_test_case(mut test: impl FnMut(&[u8], EllipticCurve)) {
        let prime_fields: &[&[u8]] = &[
            &hex!("fffffffffffffffffffffffffffffffeffffffffffffffff"),
            &hex!("ffffffff00000001000000000000000000000000ffffffffffffffffffffffff"),
            #[cfg(ecc_has_curve_p384)]
            &hex!(
                "fffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffeffffffff0000000000000000ffffffff"
            ),
        ];

        for &prime_field in prime_fields {
            let curve;
            for_each_ecc_curve! {
                (all $(($_id:tt, $name:tt, $bits:literal)),*) => {
                    curve = match prime_field.len() {
                        $(
                            v if v == $bits / 8 => EllipticCurve::$name,
                        )*
                        _ => unimplemented!(),
                    }
                };
            };

            test(prime_field, curve)
        }
    }

    fn fill_random(prime_field: &[u8], buf: &mut [u8]) {
        let rng = Rng::new();
        loop {
            rng.read(buf);
            let is_zero = buf.iter().all(|&elt| elt == 0);
            let is_modulus = buf == prime_field;
            if !is_zero && !is_modulus {
                break;
            }
        }
    }

    fn encode_affine_point<T: Curve<FieldBytesSize: ModulusSize> + PrimeCurveParams>(
        p: AffinePoint<T>,
        x: &mut [u8],
        y: &mut [u8],
    ) where
        AffinePoint<T>: ToEncodedPoint<T>,
    {
        assert_eq!(x.len(), y.len());

        let p = p.to_encoded_point(false);
        x.copy_from_slice(p.x().unwrap());
        y.copy_from_slice(p.y().unwrap());
    }

    macro_rules! curve_crate {
        (P192 $($rest:tt)*) => {
            p192 $($rest)*
        };
        (P256 $($rest:tt)*) => {
            p256 $($rest)*
        };
        (P384 $($rest:tt)*) => {
            p384 $($rest)*
        };
    }

    macro_rules! uint {
        (P192) => {
            crypto_bigint::U192
        };
        (P256) => {
            crypto_bigint::U256
        };
        (P384) => {
            crypto_bigint::U384
        };
    }

    fn affine_point_coords(curve: EllipticCurve, x: &mut [u8], y: &mut [u8]) {
        for_each_ecc_curve! {
            (all $(($_id:tt, $name:tt, $bits:literal)),*) => {
                match curve {
                    $(
                        EllipticCurve::$name => encode_affine_point(curve_crate!($name::AffinePoint::GENERATOR), x, y),
                    )*
                }
            };
        };
    }

    fn affine_point_multiplication(
        curve: EllipticCurve,
        k: &[u8],
        sw_x: &mut [u8],
        sw_y: &mut [u8],
    ) {
        for_each_ecc_curve! {
            (all $(($_id:tt, $name:tt, $bits:literal)),*) => {
                match curve {
                    $(
                        EllipticCurve::$name => {
                            let p = curve_crate!($name::AffinePoint::GENERATOR);
                            let sw_k = curve_crate!($name::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap()));
                            encode_affine_point(p.mul(sw_k).to_affine(), sw_x, sw_y);
                        }
                    )*
                }
            };
        };
    }

    fn affine_to_jacobian(
        curve: EllipticCurve,
        prime_field: &[u8],
        k: &[u8],
        sw_x: &mut [u8],
        sw_y: &mut [u8],
    ) {
        for_each_ecc_curve! {
            (all $(($_id:tt, $name:tt, $bits:literal)),*) => {
                match curve {
                    $(
                        EllipticCurve::$name => {
                            let modulus = DynResidueParams::new(&< uint!($name) >::from_be_slice(prime_field));
                            let z = DynResidue::new(&< uint!($name) >::from_be_slice(k), modulus);

                            let x_affine = DynResidue::new(&< uint!($name) >::from_be_slice(sw_x), modulus);
                            let x_jacobian = x_affine * z * z;
                            sw_x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_ref());

                            let y_affine = DynResidue::new(&< uint!($name) >::from_be_slice(sw_y), modulus);
                            let y_jacobian = y_affine * z * z * z;
                            sw_y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_ref());
                        },
                    )*
                }
            };
        };
    }

    #[cfg(ecc_has_finite_field_division)]
    fn finite_field_division(
        curve: EllipticCurve,
        prime_field: &[u8],
        sw_k: &mut [u8],
        sw_y: &mut [u8],
    ) {
        for_each_ecc_curve! {
            (all $(($_id:tt, $name:tt, $bits:literal)),*) => {
                match curve {
                    $(
                        EllipticCurve::$name => {
                            let modulus = DynResidueParams::new(&< uint!($name) >::from_be_slice(prime_field));
                            let y_res = DynResidue::new(&< uint!($name) >::from_be_slice(sw_y), modulus);
                            let k_res = DynResidue::new(&< uint!($name) >::from_be_slice(sw_k), modulus);
                            sw_y.copy_from_slice(
                                y_res
                                    .mul(&(k_res.invert().0))
                                    .retrieve()
                                    .to_be_bytes()
                                    .as_ref(),
                            );
                        },
                    )*
                }
            };
        };
    }

    const MAX_MEM_BLOCK_SIZE: usize = property!("ecc.mem_block_size");

    // ECC hardware works on little endian data, but the standard encoding (and thus, the bytes we
    // get out of the elliptic_curve crates) is big endian.
    macro_rules! reverse {
        ($($buff:ident),*) => {
            $(
            $buff.reverse();
            )*
        }
    }

    /// Creates the requested buffers that are large enough to hold the parameters of the given
    /// curve.
    macro_rules! buffers {
        ($curve:ident => { $($buff:ident),* }) => {
            let len = $curve.size();

            let mut mem = &mut [0u8; (0 $(+ { stringify!($buff);1 })* ) * MAX_MEM_BLOCK_SIZE][..];
            $(
            let $buff = mem.split_off_mut(..len).unwrap();
            )*
        }
    }

    #[test]
    fn test_ecc_affine_point_multiplication(mut ctx: Context<'static>) {
        for_each_test_case(|prime_field, curve| {
            buffers!(curve => {
                k, x, y,
                sw_x, sw_y
            });

            // Generate test data
            fill_random(prime_field, k);
            affine_point_coords(curve, x, y);

            // Run software algorithm
            affine_point_multiplication(curve, k, sw_x, sw_y);

            // Run hardware algorithm
            reverse!(k, x, y);
            let result = ctx
                .ecc
                .affine_point_multiplication(curve, k, x, y)
                .expect("Input data doesn't match the key length");
            result
                .read_affine_point_result(x, y)
                .expect("Output data doesn't match the key length selected.");
            reverse!(x, y);

            assert_eq!(x, sw_x);
            assert_eq!(y, sw_y);
        })
    }

    #[test]
    fn test_ecc_affine_point_verification(mut ctx: Context<'static>) {
        for_each_test_case(|prime_field, curve| {
            buffers!(curve => { k, x, y });

            // Generate test data
            fill_random(prime_field, k);
            affine_point_multiplication(curve, k, x, y);

            // Run hardware algorithm
            reverse!(x, y);
            let result = ctx
                .ecc
                .affine_point_verification(curve, x, y)
                .expect("Input data doesn't match the key length");

            assert!(
                result.success(),
                "x = {:02X?} y = {:02X?} is not on the curve",
                x,
                y
            );
        })
    }

    #[test]
    fn test_ecc_affine_point_verification_multiplication(mut ctx: Context<'static>) {
        for_each_test_case(|prime_field, curve| {
            buffers!(curve => {
                k, px, py,
                sw_x, sw_y
            });

            #[cfg(ecc_separate_jacobian_point_memory)]
            buffers!(curve => { qx, qy, qz });

            // Generate test data
            fill_random(prime_field, k);
            affine_point_coords(curve, px, py);

            // Run software algorithm
            affine_point_multiplication(curve, k, sw_x, sw_y);

            // Run hardware algorithm
            reverse!(k, px, py);

            let result = ctx
                .ecc
                .affine_point_verification_multiplication(curve, k, px, py)
                .expect("Input data doesn't match the key length");

            match result.read_affine_point_result(px, py) {
                Err(OperationError::ParameterLengthMismatch) => {
                    panic!("Output data doesn't match the key length")
                }
                Err(OperationError::PointNotOnCurve) => {
                    panic!("x = {:02X?} y = {:02X?} is not on the curve", px, py)
                }
                _ => {}
            }
            reverse!(px, py);

            #[cfg(ecc_separate_jacobian_point_memory)]
            result
                .read_jacobian_point_result(qx, qy, qz)
                .expect("Output data doesn't match the key length");

            #[cfg(ecc_separate_jacobian_point_memory)]
            reverse!(qx, qy, qz);

            assert_eq!(px, sw_x);
            assert_eq!(py, sw_y);
        })
    }

    #[test]
    fn test_ecc_jacobian_point_multiplication(mut ctx: Context<'static>) {
        for_each_test_case(|prime_field, curve| {
            buffers!(curve => {
                k, x, y,
                sw_k, sw_x, sw_y
            });

            fill_random(prime_field, k);
            sw_k.copy_from_slice(k);
            affine_point_coords(curve, x, y);

            // FIXME: Overwrites k, its value should also be checked
            reverse!(k, x, y);
            let result = ctx
                .ecc
                .jacobian_point_multiplication(curve, k, x, y)
                .expect("Input data doesn't match the key length");
            result
                .read_jacobian_point_result(x, y, k)
                .expect("Output data doesn't match the key length");
            reverse!(k, x, y);

            affine_point_multiplication(curve, sw_k, sw_x, sw_y);
            // FIXME: k comes from the hardware result, it's surprising. We should compute it
            // in software to verify the result.
            affine_to_jacobian(curve, prime_field, k, sw_x, sw_y);

            assert_eq!(x, sw_x);
            assert_eq!(y, sw_y);
        })
    }

    #[test]
    fn test_jacobian_point_verification(mut ctx: Context<'static>) {
        for_each_test_case(|prime_field, curve| {
            // TODO: we should have a failure case here
            buffers!(curve => { k, x, y, z });

            fill_random(prime_field, k);
            fill_random(prime_field, z);

            affine_point_multiplication(curve, k, x, y);
            affine_to_jacobian(curve, prime_field, z, x, y);

            reverse!(x, y, z);
            let result = ctx
                .ecc
                .jacobian_point_verification(curve, x, y, z)
                .expect("Input data doesn't match the key length");

            assert!(
                result.success(),
                "x = {:02X?}, y = {:02X?}, z = {:02X?} is not on the curve",
                x,
                y,
                z,
            )
        })
    }

    #[test]
    fn test_ecc_affine_point_verification_jacobian_multiplication(mut ctx: Context<'static>) {
        for_each_test_case(|prime_field, curve| {
            buffers!(curve => {
                k, x, y,
                sw_k, sw_x, sw_y
            });

            fill_random(prime_field, k);
            sw_k.copy_from_slice(k);
            affine_point_coords(curve, x, y);

            // FIXME: overwrites k, its value should also be checked
            reverse!(k, x, y);
            match ctx
                .ecc
                .affine_point_verification_jacobian_multiplication(curve, k, x, y)
            {
                Err(_) => panic!("Input data doesn't match the key length"),
                Ok(result) => {
                    assert!(
                        result.success(),
                        "x = {:02X?} y = {:02X?} is not on the curve",
                        x,
                        y
                    );
                    result
                        .read_jacobian_point_result(x, y, k)
                        .expect("Output data doesn't match the key length")
                }
            }
            reverse!(k, x, y);

            affine_point_multiplication(curve, sw_k, sw_x, sw_y);
            // FIXME: k comes from the hardware result, it's surprising. We should compute it
            // in software to verify the result.
            affine_to_jacobian(curve, prime_field, k, sw_x, sw_y);

            assert_eq!(x, sw_x);
            assert_eq!(y, sw_y);
        })
    }

    #[test]
    #[cfg(ecc_has_finite_field_division)]
    fn test_ecc_finite_field_division(mut ctx: Context<'static>) {
        for_each_test_case(|prime_field, curve| {
            buffers!(curve => {
                k, y,
                sw_k, sw_y
            });

            fill_random(prime_field, k);
            fill_random(prime_field, y);

            sw_y.copy_from_slice(y);
            sw_k.copy_from_slice(k);

            reverse!(k, y);
            let result = ctx
                .ecc
                .finite_field_division(curve, k, y)
                .expect("Input data doesn't match the key length selected.");
            result
                .read_scalar_result(y)
                .expect("Output data doesn't match the key length selected.");
            reverse!(y);

            finite_field_division(curve, prime_field, sw_k, sw_y);

            assert_eq!(y, sw_y);
        })
    }

    #[cfg(ecc_has_point_addition)]
    struct PointAddTestCase<const N: usize> {
        curve: EllipticCurve,
        x: [u8; N],
        y: [u8; N],
    }

    #[cfg(ecc_has_point_addition)]
    fn test_ecc_point_addition<const N: usize>(
        mut ctx: Context<'static>,
        test_case: PointAddTestCase<N>,
    ) {
        let mut one = [0u8; N];
        one[0] = 1;

        let mut two = [0u8; N];
        two[0] = 2;

        let result = ctx
            .ecc
            .affine_point_addition(
                test_case.curve,
                &test_case.x,
                &test_case.y,
                &test_case.x,
                &test_case.y,
                &one,
            )
            .unwrap();

        let mut result_px = [0; N];
        let mut result_py = [0; N];
        let mut result_qx = [0; N];
        let mut result_qy = [0; N];
        let mut result_qz = [0; N];

        result
            .read_affine_point_result(&mut result_px, &mut result_py)
            .unwrap();

        result
            .read_jacobian_point_result(&mut result_qx, &mut result_qy, &mut result_qz)
            .unwrap();

        // Now verify that P + P = 2*P

        let mut mul_result_px = [0; N];
        let mut mul_result_py = [0; N];
        let mut mul_result_qx = [0; N];
        let mut mul_result_qy = [0; N];
        let mut mul_result_qz = [0; N];

        let result = ctx
            .ecc
            .affine_point_multiplication(test_case.curve, &two, &test_case.x, &test_case.y)
            .unwrap();

        result
            .read_affine_point_result(&mut mul_result_px, &mut mul_result_py)
            .unwrap();

        let result = ctx
            .ecc
            .jacobian_point_multiplication(test_case.curve, &two, &test_case.x, &test_case.y)
            .unwrap();

        result
            .read_jacobian_point_result(&mut mul_result_qx, &mut mul_result_qy, &mut mul_result_qz)
            .unwrap();

        assert_eq!(result_px, mul_result_px);
        assert_eq!(result_py, mul_result_py);

        assert_eq!(result_qx, mul_result_qx);
        assert_eq!(result_qy, mul_result_qy);
        assert_eq!(result_qz, mul_result_qz);
    }

    #[test]
    #[cfg(ecc_has_point_addition)]
    fn test_ecc_point_addition_192(ctx: Context<'static>) {
        test_ecc_point_addition(
            ctx,
            PointAddTestCase {
                curve: EllipticCurve::P192,
                x: [
                    0x12, 0x10, 0xFF, 0x82, 0xFD, 0x0A, 0xFF, 0xF4, 0x00, 0x88, 0xA1, 0x43, 0xEB,
                    0x20, 0xBF, 0x7C, 0xF6, 0x90, 0x30, 0xB0, 0x0E, 0xA8, 0x8D, 0x18,
                ],
                y: [
                    0x11, 0x48, 0x79, 0x1E, 0xA1, 0x77, 0xF9, 0x73, 0xD5, 0xCD, 0x24, 0x6B, 0xED,
                    0x11, 0x10, 0x63, 0x78, 0xDA, 0xC8, 0xFF, 0x95, 0x2B, 0x19, 0x07,
                ],
            },
        );
    }

    #[test]
    #[cfg(ecc_has_point_addition)]
    fn test_ecc_point_addition_256(ctx: Context<'static>) {
        test_ecc_point_addition(
            ctx,
            PointAddTestCase {
                curve: EllipticCurve::P256,
                x: [
                    0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4, 0xA0, 0x33, 0xEB, 0x2D, 0x81,
                    0x7D, 0x03, 0x77, 0xF2, 0x40, 0xA4, 0x63, 0xE5, 0xE6, 0xBC, 0xF8, 0x47, 0x42,
                    0x2C, 0xE1, 0xF2, 0xD1, 0x17, 0x6B,
                ],
                y: [
                    0xF5, 0x51, 0xBF, 0x37, 0x68, 0x40, 0xB6, 0xCB, 0xCE, 0x5E, 0x31, 0x6B, 0x57,
                    0x33, 0xCE, 0x2B, 0x16, 0x9E, 0x0F, 0x7C, 0x4A, 0xEB, 0xE7, 0x8E, 0x9B, 0x7F,
                    0x1A, 0xFE, 0xE2, 0x42, 0xE3, 0x4F,
                ],
            },
        );
    }

    #[test]
    #[cfg(all(ecc_has_point_addition, ecc_has_curve_p384))]
    fn test_ecc_point_addition_384(ctx: Context<'static>) {
        test_ecc_point_addition(
            ctx,
            PointAddTestCase {
                curve: EllipticCurve::P384,
                x: [
                    0xaa, 0x87, 0xca, 0x22, 0xbe, 0x8b, 0x05, 0x37, 0x8e, 0xb1, 0xc7, 0x1e, 0xf3,
                    0x20, 0xad, 0x74, 0x6e, 0x1d, 0x3b, 0x62, 0x8b, 0xa7, 0x9b, 0x98, 0x59, 0xf7,
                    0x41, 0xe0, 0x82, 0x54, 0x2a, 0x38, 0x55, 0x02, 0xf2, 0x5d, 0xbf, 0x55, 0x29,
                    0x6c, 0x3a, 0x54, 0x5e, 0x38, 0x72, 0x76, 0x0a, 0xb7,
                ],
                y: [
                    0x36, 0x17, 0xde, 0x4a, 0x96, 0x26, 0x2c, 0x6f, 0x5d, 0x9e, 0x98, 0xbf, 0x92,
                    0x92, 0xdc, 0x29, 0xf8, 0xf4, 0x1d, 0xbd, 0x28, 0x9a, 0x14, 0x7c, 0xe9, 0xda,
                    0x31, 0x13, 0xb5, 0xf0, 0xb8, 0xc0, 0x0a, 0x60, 0xb1, 0xce, 0x1d, 0x7e, 0x81,
                    0x9d, 0x7a, 0x43, 0x1d, 0x7c, 0x90, 0xea, 0x0e, 0x5f,
                ],
            },
        );
    }

    #[cfg(ecc_has_modular_arithmetic)]
    struct ModOpTestCase<const N: usize> {
        curve: EllipticCurve,
        x: [u8; N],
        y: [u8; N],
        num: [u8; N],
        den: [u8; N],
        expected_sum: [u8; N],
        expected_diff: [u8; N],
        expected_prod: [u8; N],
        expected_div: [u8; N],
    }

    #[cfg(ecc_has_modular_arithmetic)]
    fn run_mod_tests<const N: usize>(mut ctx: Context<'static>, test_case: ModOpTestCase<N>) {
        let result = &mut [0; N][..];

        ctx.ecc
            .modular_addition(test_case.curve, &test_case.x, &test_case.y)
            .unwrap()
            .read_scalar_result(result)
            .unwrap();
        assert_eq!(result, test_case.expected_sum);

        ctx.ecc
            .modular_subtraction(test_case.curve, &test_case.x, &test_case.y)
            .unwrap()
            .read_scalar_result(result)
            .unwrap();
        assert_eq!(result, test_case.expected_diff);

        ctx.ecc
            .modular_multiplication(test_case.curve, &test_case.x, &test_case.y)
            .unwrap()
            .read_scalar_result(result)
            .unwrap();
        assert_eq!(result, test_case.expected_prod);

        ctx.ecc
            .modular_division(test_case.curve, &test_case.num, &test_case.den)
            .unwrap()
            .read_scalar_result(result)
            .unwrap();
        assert_eq!(result, test_case.expected_div);
    }

    #[test]
    #[cfg(ecc_has_modular_arithmetic)]
    fn test_ecc_mod_operations_256(ctx: Context<'static>) {
        run_mod_tests(
            ctx,
            ModOpTestCase {
                curve: EllipticCurve::P256,
                x: [
                    0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4, 0xA0, 0x33, 0xEB, 0x2D, 0x81,
                    0x7D, 0x03, 0x77, 0xF2, 0x40, 0xA4, 0x63, 0xE5, 0xE6, 0xBC, 0xF8, 0x47, 0x42,
                    0x2C, 0xE1, 0xF2, 0xD1, 0x17, 0x6B,
                ],
                y: [
                    0xF5, 0x51, 0xBF, 0x37, 0x68, 0x40, 0xB6, 0xCB, 0xCE, 0x5E, 0x31, 0x6B, 0x57,
                    0x33, 0xCE, 0x2B, 0x16, 0x9E, 0x0F, 0x7C, 0x4A, 0xEB, 0xE7, 0x8E, 0x9B, 0x7F,
                    0x1A, 0xFE, 0xE2, 0x42, 0xE3, 0x4F,
                ],
                num: [
                    0x20, 0x56, 0x14, 0xB6, 0xAF, 0x94, 0xA0, 0xB6, 0x0C, 0xDF, 0x13, 0x1A, 0xE6,
                    0xBF, 0x57, 0x87, 0xF1, 0x02, 0x73, 0x96, 0x53, 0x1A, 0xBC, 0xA9, 0x0F, 0x5E,
                    0xA1, 0xFC, 0x0E, 0xFC, 0x9D, 0x9B,
                ],
                den: [
                    0x54, 0x3B, 0x11, 0x78, 0xC4, 0xCA, 0x52, 0xFD, 0xCC, 0x89, 0x51, 0x0F, 0xFE,
                    0x7D, 0x37, 0x83, 0x81, 0xD5, 0x2E, 0x58, 0x42, 0xF9, 0x4F, 0x19, 0x9A, 0x79,
                    0x78, 0x98, 0xFA, 0x95, 0x40, 0x2E,
                ],
                expected_sum: [
                    0x8B, 0x14, 0x58, 0x10, 0xAE, 0x79, 0x57, 0xC0, 0x6F, 0x92, 0x1C, 0x99, 0xD8,
                    0xB0, 0xD1, 0xA2, 0x08, 0xDF, 0xB3, 0xDF, 0x2F, 0xD2, 0xA4, 0x87, 0xE3, 0xC1,
                    0x46, 0xDF, 0xD5, 0x14, 0xFB, 0xBA,
                ],
                expected_diff: [
                    0xA1, 0x70, 0xD9, 0xA0, 0xDD, 0xF8, 0xEA, 0x28, 0xD2, 0xD4, 0xB9, 0xC2, 0x29,
                    0x4A, 0x35, 0x4B, 0xDC, 0xA2, 0x94, 0xE7, 0x9A, 0xFB, 0xD4, 0x69, 0xAC, 0xC2,
                    0x11, 0xE3, 0x0F, 0x8F, 0x34, 0x1B,
                ],
                expected_prod: [
                    0x18, 0x4D, 0xCE, 0xCC, 0x1A, 0xA8, 0xEC, 0x72, 0xD7, 0x31, 0xDA, 0x41, 0x8C,
                    0x75, 0x6B, 0xF1, 0x2A, 0x2E, 0x5B, 0x53, 0x8D, 0xCA, 0x79, 0x61, 0x6B, 0x46,
                    0xF9, 0x2E, 0x27, 0xB5, 0x43, 0x15,
                ],
                expected_div: [
                    0x33, 0xF3, 0x55, 0x3B, 0x46, 0x8A, 0x13, 0xC0, 0x1D, 0x7E, 0x41, 0xA6, 0xFF,
                    0x53, 0xFD, 0x78, 0xD5, 0xC0, 0xE5, 0x9F, 0x78, 0xD1, 0x86, 0x66, 0x77, 0x3C,
                    0x6E, 0xEF, 0x58, 0xF6, 0x29, 0x34,
                ],
            },
        );
    }

    #[test]
    #[cfg(ecc_has_modular_arithmetic)]
    fn test_ecc_mod_operations_192(ctx: Context<'static>) {
        run_mod_tests::<24>(
            ctx,
            ModOpTestCase {
                curve: EllipticCurve::P192,
                x: [
                    0x1A, 0x80, 0xA1, 0x5F, 0x1F, 0xB7, 0x59, 0x1B, 0x9F, 0xD7, 0xFB, 0xAE, 0xA9,
                    0xF9, 0x1E, 0xBA, 0x67, 0xAE, 0x57, 0xB7, 0x27, 0x80, 0x9E, 0x1A,
                ],
                y: [
                    0x59, 0xC6, 0x3D, 0xD3, 0xD7, 0xDF, 0xA3, 0x44, 0x7C, 0x75, 0x52, 0xB4, 0x42,
                    0xF3, 0xFC, 0xA6, 0x0F, 0xA8, 0x8A, 0x8D, 0x1F, 0xA3, 0xDF, 0x54,
                ],
                num: [
                    0xBA, 0x0F, 0x2C, 0xD8, 0xBE, 0xCC, 0x2D, 0xD3, 0xD5, 0x74, 0xBD, 0x8C, 0xF3,
                    0x3E, 0x3B, 0x7A, 0xA4, 0xD0, 0x71, 0xEC, 0x85, 0xF6, 0x70, 0x00,
                ],
                den: [
                    0x15, 0xF9, 0x20, 0xD8, 0x46, 0x5C, 0x03, 0x97, 0x4A, 0x10, 0xEF, 0x8A, 0xFB,
                    0x12, 0x2E, 0x65, 0x6E, 0xD6, 0x79, 0x1E, 0x65, 0x6F, 0x3E, 0x64,
                ],
                expected_sum: [
                    0x73, 0x46, 0xDF, 0x32, 0xF7, 0x96, 0xFD, 0x5F, 0x1B, 0x4D, 0x4E, 0x63, 0xEC,
                    0xEC, 0x1B, 0x61, 0x77, 0x56, 0xE2, 0x44, 0x47, 0x23, 0x7E, 0x6F,
                ],
                expected_diff: [
                    0xF2, 0xE1, 0x35, 0x41, 0xF9, 0xA0, 0x21, 0xEB, 0x58, 0x5A, 0x88, 0x94, 0x66,
                    0x06, 0x22, 0x13, 0x58, 0x06, 0xCD, 0x29, 0x08, 0xDD, 0xBE, 0xC5,
                ],
                expected_prod: [
                    0xB5, 0xB9, 0xFF, 0xBC, 0x52, 0xC8, 0xB8, 0x36, 0x8C, 0xFB, 0xA5, 0xCE, 0x1E,
                    0x7B, 0xE6, 0xF3, 0x8F, 0x79, 0x71, 0xCF, 0xD6, 0xF3, 0x41, 0xE6,
                ],
                expected_div: [
                    0x6B, 0xB3, 0x6B, 0x2B, 0x56, 0x6A, 0xE5, 0xF7, 0x75, 0x82, 0xF0, 0xCC, 0x93,
                    0x63, 0x40, 0xF8, 0xEF, 0x35, 0x2A, 0xAF, 0xBD, 0x56, 0xE9, 0x29,
                ],
            },
        );
    }

    #[test]
    #[cfg(all(ecc_has_modular_arithmetic, ecc_has_curve_p384))]
    fn test_ecc_mod_operations_384(ctx: Context<'static>) {
        run_mod_tests::<48>(
            ctx,
            ModOpTestCase {
                curve: EllipticCurve::P384,
                x: [
                    0xaa, 0x87, 0xca, 0x22, 0xbe, 0x8b, 0x05, 0x37, 0x8e, 0xb1, 0xc7, 0x1e, 0xf3,
                    0x20, 0xad, 0x74, 0x6e, 0x1d, 0x3b, 0x62, 0x8b, 0xa7, 0x9b, 0x98, 0x59, 0xf7,
                    0x41, 0xe0, 0x82, 0x54, 0x2a, 0x38, 0x55, 0x02, 0xf2, 0x5d, 0xbf, 0x55, 0x29,
                    0x6c, 0x3a, 0x54, 0x5e, 0x38, 0x72, 0x76, 0x0a, 0xb7,
                ],
                y: [
                    0x36, 0x17, 0xde, 0x4a, 0x96, 0x26, 0x2c, 0x6f, 0x5d, 0x9e, 0x98, 0xbf, 0x92,
                    0x92, 0xdc, 0x29, 0xf8, 0xf4, 0x1d, 0xbd, 0x28, 0x9a, 0x14, 0x7c, 0xe9, 0xda,
                    0x31, 0x13, 0xb5, 0xf0, 0xb8, 0xc0, 0x0a, 0x60, 0xb1, 0xce, 0x1d, 0x7e, 0x81,
                    0x9d, 0x7a, 0x43, 0x1d, 0x7c, 0x90, 0xea, 0x0e, 0x5f,
                ],
                num: [
                    0x68, 0xd1, 0x09, 0xa7, 0xc7, 0x7e, 0xeb, 0xbd, 0x43, 0x18, 0x7e, 0xdd, 0x69,
                    0x23, 0x7e, 0x0a, 0xef, 0x07, 0xc2, 0x0e, 0xc5, 0x3d, 0xe7, 0xcb, 0xd4, 0x36,
                    0xad, 0x9b, 0xdc, 0xf8, 0x6c, 0x5c, 0x0c, 0x3d, 0xce, 0x45, 0xcd, 0x6f, 0x7f,
                    0x18, 0x40, 0xc5, 0x29, 0xf3, 0xcd, 0x12, 0x1d, 0xc2,
                ],
                den: [
                    0x68, 0xd1, 0x09, 0xa7, 0xc7, 0x7e, 0xeb, 0xbd, 0x43, 0x18, 0x7e, 0xdd, 0x69,
                    0x23, 0x7e, 0x0a, 0xef, 0x07, 0xc2, 0x0e, 0xc5, 0x3d, 0xe7, 0xcb, 0xd4, 0x36,
                    0xad, 0x9b, 0xdc, 0xf8, 0x6c, 0x5c, 0x0c, 0x3d, 0xce, 0x45, 0xcd, 0x6f, 0x7f,
                    0x18, 0x40, 0xc5, 0x8c, 0x56, 0x68, 0xb7, 0xb8, 0x67,
                ],
                expected_sum: [
                    0x6D, 0x75, 0xE3, 0xA0, 0xE9, 0x98, 0x45, 0xB9, 0x70, 0xA8, 0xAF, 0x95, 0xD3,
                    0xA5, 0x6F, 0x46, 0x87, 0xE4, 0x21, 0x2B, 0x32, 0xF4, 0x4C, 0x4D, 0x43, 0xD2,
                    0x73, 0xF3, 0x37, 0x45, 0xE3, 0xF8, 0x5F, 0x62, 0xA3, 0x2C, 0xDD, 0xD3, 0xAA,
                    0x09, 0xB5, 0x97, 0x7B, 0xB4, 0x02, 0x61, 0x19, 0x16,
                ],
                expected_diff: [
                    0x74, 0x70, 0xEC, 0xD7, 0x27, 0x65, 0xD9, 0xC7, 0x30, 0x13, 0x2F, 0x5F, 0x60,
                    0x8E, 0xD0, 0x4A, 0x76, 0x28, 0x1D, 0xA5, 0x62, 0x0D, 0x87, 0x1C, 0x70, 0x1C,
                    0x10, 0xCD, 0xCD, 0x63, 0x71, 0x77, 0x4A, 0xA2, 0x40, 0x8F, 0xA1, 0xD7, 0xA7,
                    0xCE, 0xBF, 0x10, 0x41, 0xBC, 0xE1, 0x8B, 0xFB, 0x57,
                ],
                expected_prod: [
                    0x63, 0x67, 0x7D, 0x8B, 0x32, 0x4C, 0x13, 0xE6, 0x49, 0xAB, 0xDE, 0x9F, 0xDB,
                    0x68, 0x57, 0x49, 0xDE, 0x88, 0x77, 0x56, 0x45, 0xB0, 0x7B, 0xD7, 0xAB, 0xFB,
                    0xF4, 0x55, 0xC0, 0xD3, 0xD0, 0x2D, 0x37, 0x14, 0x8F, 0x3A, 0x1E, 0x72, 0x7E,
                    0x49, 0x77, 0xA0, 0xB9, 0xC8, 0xD0, 0x44, 0xDD, 0x16,
                ],
                expected_div: [
                    0x05, 0x35, 0xe1, 0x77, 0xd0, 0xd0, 0x47, 0x38, 0x65, 0xe8, 0x4c, 0xeb, 0x31,
                    0x96, 0xd9, 0xfc, 0x18, 0x0a, 0xe2, 0xd9, 0x5d, 0x40, 0xf4, 0x89, 0x8a, 0xc6,
                    0xfc, 0x32, 0x5f, 0xd8, 0xc7, 0x74, 0x15, 0x44, 0xa5, 0xd6, 0xca, 0x72, 0xc1,
                    0x2f, 0xd7, 0xb3, 0x17, 0xda, 0x6f, 0x49, 0x17, 0xb7,
                ],
            },
        );
    }
}
