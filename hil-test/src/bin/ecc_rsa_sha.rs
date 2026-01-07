//! ECC, RSA and SHA Tests

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: unstable

#![no_std]
#![no_main]

use hil_test as _;

#[cfg(any(esp32c2, esp32c6, esp32h2))]
#[embedded_test::tests(default_timeout = 10)]
mod ecc_tests {
    use core::ops::Mul;

    use crypto_bigint::{
        Encoding,
        U192,
        U256,
        modular::runtime_mod::{DynResidue, DynResidueParams},
    };
    use elliptic_curve::sec1::ToEncodedPoint;
    #[cfg(feature = "esp32h2")]
    use esp_hal::ecc::WorkMode;
    use esp_hal::{
        Blocking,
        clock::CpuClock,
        ecc::{Ecc, EllipticCurve, Error},
        rng::{Rng, TrngSource},
    };
    use hex_literal::hex;

    struct TestParams<'a> {
        prime_fields: &'a [&'a [u8]],
        nb_loop_mul: usize,
        #[cfg(feature = "esp32c2")]
        nb_loop_inv: usize,
    }

    const TEST_PARAMS_VECTOR: TestParams = TestParams {
        prime_fields: &[
            &hex!("fffffffffffffffffffffffffffffffeffffffffffffffff"),
            &hex!("ffffffff00000001000000000000000000000000ffffffffffffffffffffffff"),
        ],
        nb_loop_mul: 10,
        #[cfg(feature = "esp32c2")]
        nb_loop_inv: 20,
    };

    struct Context<'a> {
        ecc: Ecc<'a, Blocking>,
        _rng_source: TrngSource<'a>,
    }

    #[init]
    fn init() -> Context<'static> {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);

        Context {
            ecc: Ecc::new(p.ECC),
            _rng_source: TrngSource::new(p.RNG, p.ADC1),
        }
    }

    #[test]
    fn test_ecc_affine_point_multiplication(mut ctx: Context<'static>) {
        let rng = Rng::new();
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            match prime_field.len() {
                24 => (),
                _ => (),
            };
            let t1 = &mut [0_u8; 96];
            let (k, x) = t1.split_at_mut(prime_field.len());
            let (x, y) = x.split_at_mut(prime_field.len());
            let (y, _) = y.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                loop {
                    rng.read(k);
                    let is_zero = k.iter().all(|&elt| elt == 0);
                    let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b);
                    if is_zero == false && is_modulus == false {
                        break;
                    }
                }
                let curve = match prime_field.len() {
                    24 => {
                        x.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        y.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P192
                    }
                    32 => {
                        x.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        y.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P256
                    }
                    _ => unimplemented!(),
                };

                ctx.ecc
                    .affine_point_multiplication(curve, k, x, y)
                    .expect("Inputs data doesn't match the key length selected.");

                let t2 = &mut [0_u8; 64];

                let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
                let (sw_y, _) = sw_y.split_at_mut(prime_field.len());

                match prime_field.len() {
                    24 => {
                        let sw_k = p192::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p192::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        sw_x.copy_from_slice(q.x().unwrap().as_slice());
                        sw_y.copy_from_slice(q.y().unwrap().as_slice());
                    }
                    32 => {
                        let sw_k = p256::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p256::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        sw_x.copy_from_slice(q.x().unwrap().as_slice());
                        sw_y.copy_from_slice(q.y().unwrap().as_slice());
                    }
                    _ => unimplemented!(),
                };

                for (a, b) in x.iter().zip(sw_x) {
                    assert_eq!(a, b);
                }

                for (a, b) in y.iter().zip(sw_y) {
                    assert_eq!(a, b);
                }
            }
        }
    }

    #[test]
    fn test_ecc_affine_point_verification(mut ctx: Context<'static>) {
        let rng = Rng::new();
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 96];
            let (k, x) = t1.split_at_mut(prime_field.len());
            let (x, y) = x.split_at_mut(prime_field.len());
            let (y, _) = y.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                loop {
                    rng.read(k);
                    let is_zero = k.iter().all(|&elt| elt == 0);
                    let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b);
                    if is_zero == false && is_modulus == false {
                        break;
                    }
                }

                let curve = match prime_field.len() {
                    24 => {
                        let sw_k = p192::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p192::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        x.copy_from_slice(q.x().unwrap().as_slice());
                        y.copy_from_slice(q.y().unwrap().as_slice());
                        &EllipticCurve::P192
                    }
                    32 => {
                        let sw_k = p256::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p256::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        x.copy_from_slice(q.x().unwrap().as_slice());
                        y.copy_from_slice(q.y().unwrap().as_slice());
                        &EllipticCurve::P256
                    }
                    _ => unimplemented!(),
                };

                match ctx.ecc.affine_point_verification(&curve, x, y) {
                    Err(Error::SizeMismatchCurve) => {
                        assert!(false, "Inputs data doesn't match the key length selected.")
                    }
                    Err(Error::PointNotOnSelectedCurve) => assert!(
                        false,
                        "ECC failed while affine point verification with x = {:02X?} and y = {:02X?}.",
                        x, y,
                    ),
                    _ => {}
                }
            }
        }
    }

    #[test]
    fn test_ecc_affine_point_verification_multiplication(mut ctx: Context<'static>) {
        let rng = Rng::new();
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 96];
            let (k, px) = t1.split_at_mut(prime_field.len());
            let (px, py) = px.split_at_mut(prime_field.len());
            let (py, _) = py.split_at_mut(prime_field.len());
            #[cfg(feature = "esp32h2")]
            let qx = &mut [0u8; 8];
            #[cfg(feature = "esp32h2")]
            let qy = &mut [0u8; 8];
            #[cfg(feature = "esp32h2")]
            let qz = &mut [0u8; 8];
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                loop {
                    rng.read(k);
                    let is_zero = k.iter().all(|&elt| elt == 0);
                    let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b);
                    if is_zero == false && is_modulus == false {
                        break;
                    }
                }
                let curve = match prime_field.len() {
                    24 => {
                        px.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        py.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P192
                    }
                    32 => {
                        px.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        py.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P256
                    }
                    _ => unimplemented!(),
                };

                #[cfg(not(feature = "esp32h2"))]
                let result = ctx
                    .ecc
                    .affine_point_verification_multiplication(curve, k, px, py);
                #[cfg(feature = "esp32h2")]
                let result = ctx
                    .ecc
                    .affine_point_verification_multiplication(curve, k, px, py, qx, qy, qz);
                match result {
                    Err(Error::SizeMismatchCurve) => {
                        assert!(false, "Inputs data doesn't match the key length selected.")
                    }
                    Err(Error::PointNotOnSelectedCurve) => assert!(
                        false,
                        "ECC failed while affine point verification + multiplication with x = {:02X?} and y = {:02X?}.",
                        px, py,
                    ),
                    _ => {}
                }

                let t2 = &mut [0_u8; 64];

                let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
                let (sw_y, _) = sw_y.split_at_mut(prime_field.len());

                match prime_field.len() {
                    24 => {
                        let sw_k = p192::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p192::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        sw_x.copy_from_slice(q.x().unwrap().as_slice());
                        sw_y.copy_from_slice(q.y().unwrap().as_slice());
                    }
                    32 => {
                        let sw_k = p256::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p256::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        sw_x.copy_from_slice(q.x().unwrap().as_slice());
                        sw_y.copy_from_slice(q.y().unwrap().as_slice());
                    }
                    _ => unimplemented!(),
                };

                for (a, b) in px.iter().zip(sw_x) {
                    assert_eq!(a, b);
                }

                for (a, b) in py.iter().zip(sw_y) {
                    assert_eq!(a, b);
                }
            }
        }
    }
    #[test]
    fn test_ecc_jacobian_point_multiplication(mut ctx: Context<'static>) {
        let rng = Rng::new();
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 96];
            let (k, x) = t1.split_at_mut(prime_field.len());
            let (x, y) = x.split_at_mut(prime_field.len());
            let (y, _) = y.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                let t2 = &mut [0_u8; 96];

                let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
                let (sw_y, sw_k) = sw_y.split_at_mut(prime_field.len());
                let (sw_k, _) = sw_k.split_at_mut(prime_field.len());

                loop {
                    rng.read(k);
                    let is_zero = k.iter().all(|&elt| elt == 0);
                    let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b);
                    if is_zero == false && is_modulus == false {
                        break;
                    }
                }
                sw_k.copy_from_slice(k);
                let curve = match prime_field.len() {
                    24 => {
                        x.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        y.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P192
                    }
                    32 => {
                        x.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        y.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P256
                    }
                    _ => unimplemented!(),
                };

                ctx.ecc
                    .jacobian_point_multiplication(curve, k, x, y)
                    .expect("Inputs data doesn't match the key length selected.");
                match prime_field.len() {
                    24 => {
                        let sw_k = p192::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(sw_k).unwrap(),
                        );
                        let q = p192::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        let modulus = DynResidueParams::new(&U192::from_be_slice(prime_field));
                        let x_affine = DynResidue::new(
                            &U192::from_be_slice(q.x().unwrap().as_slice()),
                            modulus,
                        );
                        let y_affine = DynResidue::new(
                            &U192::from_be_slice(q.y().unwrap().as_slice()),
                            modulus,
                        );
                        let z = DynResidue::new(&U192::from_be_slice(k), modulus);
                        let x_jacobian = x_affine * z * z;
                        let y_jacobian = y_affine * z * z * z;
                        sw_x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                        sw_y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                    }
                    32 => {
                        let sw_k = p256::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(sw_k).unwrap(),
                        );
                        let q = p256::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        let modulus = DynResidueParams::new(&U256::from_be_slice(prime_field));
                        let x_affine = DynResidue::new(
                            &U256::from_be_slice(q.x().unwrap().as_slice()),
                            modulus,
                        );
                        let y_affine = DynResidue::new(
                            &U256::from_be_slice(q.y().unwrap().as_slice()),
                            modulus,
                        );
                        let z = DynResidue::new(&U256::from_be_slice(k), modulus);
                        let x_jacobian = x_affine * z * z;
                        let y_jacobian = y_affine * z * z * z;
                        sw_x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                        sw_y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                    }
                    _ => unimplemented!(),
                };

                for (a, b) in x.iter().zip(sw_x.iter()) {
                    assert_eq!(a, b);
                }

                for (a, b) in y.iter().zip(sw_y.iter()) {
                    assert_eq!(a, b);
                }
            }
        }
    }

    #[test]
    fn test_jacobian_point_verification(mut ctx: Context<'static>) {
        let rng = Rng::new();
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 128];
            let (k, x) = t1.split_at_mut(prime_field.len());
            let (x, y) = x.split_at_mut(prime_field.len());
            let (y, z) = y.split_at_mut(prime_field.len());
            let (z, _) = z.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                loop {
                    rng.read(k);
                    rng.read(z);
                    let is_zero = k.iter().all(|&elt| elt == 0) || z.iter().all(|&elt| elt == 0);
                    let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b)
                        || z.iter().zip(prime_field).all(|(&a, &b)| a == b);
                    if is_zero == false && is_modulus == false {
                        break;
                    }
                }

                let curve = match prime_field.len() {
                    24 => {
                        let sw_k = p192::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p192::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        let modulus = DynResidueParams::new(&U192::from_be_slice(prime_field));
                        let x_affine = DynResidue::new(
                            &U192::from_be_slice(q.x().unwrap().as_slice()),
                            modulus,
                        );
                        let y_affine = DynResidue::new(
                            &U192::from_be_slice(q.y().unwrap().as_slice()),
                            modulus,
                        );
                        let z = DynResidue::new(&U192::from_be_slice(z), modulus);
                        let x_jacobian = x_affine * z * z;
                        let y_jacobian = y_affine * z * z * z;
                        x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                        y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                        &EllipticCurve::P192
                    }
                    32 => {
                        let sw_k = p256::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(k).unwrap(),
                        );
                        let q = p256::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        let modulus = DynResidueParams::new(&U256::from_be_slice(prime_field));
                        let x_affine = DynResidue::new(
                            &U256::from_be_slice(q.x().unwrap().as_slice()),
                            modulus,
                        );
                        let y_affine = DynResidue::new(
                            &U256::from_be_slice(q.y().unwrap().as_slice()),
                            modulus,
                        );
                        let z = DynResidue::new(&U256::from_be_slice(z), modulus);
                        let x_jacobian = x_affine * z * z;
                        let y_jacobian = y_affine * z * z * z;
                        x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                        y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                        &EllipticCurve::P256
                    }
                    _ => unimplemented!(),
                };

                match ctx.ecc.jacobian_point_verification(&curve, x, y, z) {
                    Err(Error::SizeMismatchCurve) => {
                        assert!(false, "Inputs data doesn't match the key length selected.")
                    }
                    Err(Error::PointNotOnSelectedCurve) => assert!(
                        false,
                        "ECC failed while base point verification with x = {:02X?} and y = {:02X?}.",
                        x, y,
                    ),
                    _ => {}
                }
            }
        }
    }

    #[test]
    fn test_ecc_affine_point_verification_jacobian_multiplication(mut ctx: Context<'static>) {
        let rng = Rng::new();
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 96];
            let (k, x) = t1.split_at_mut(prime_field.len());
            let (x, y) = x.split_at_mut(prime_field.len());
            let (y, _) = y.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                let t2 = &mut [0_u8; 96];

                let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
                let (sw_y, sw_k) = sw_y.split_at_mut(prime_field.len());
                let (sw_k, _) = sw_k.split_at_mut(prime_field.len());

                loop {
                    rng.read(k);
                    let is_zero = k.iter().all(|&elt| elt == 0);
                    let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b);
                    if is_zero == false && is_modulus == false {
                        break;
                    }
                }
                sw_k.copy_from_slice(k);
                let curve = match prime_field.len() {
                    24 => {
                        x.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        y.copy_from_slice(
                            p192::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P192
                    }
                    32 => {
                        x.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .x()
                                .unwrap(),
                        );
                        y.copy_from_slice(
                            p256::AffinePoint::GENERATOR
                                .to_encoded_point(false)
                                .y()
                                .unwrap(),
                        );
                        &EllipticCurve::P256
                    }
                    _ => unimplemented!(),
                };

                match ctx
                    .ecc
                    .affine_point_verification_jacobian_multiplication(curve, k, x, y)
                {
                    Err(Error::SizeMismatchCurve) => {
                        assert!(false, "Inputs data doesn't match the key length selected.")
                    }
                    Err(Error::PointNotOnSelectedCurve) => assert!(
                        false,
                        "ECC failed while affine point verification + multiplication with x = {:02X?} and y = {:02X?}.",
                        x, y,
                    ),
                    _ => {}
                }

                match prime_field.len() {
                    24 => {
                        let sw_k = p192::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(sw_k).unwrap(),
                        );
                        let q = p192::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        let modulus = DynResidueParams::new(&U192::from_be_slice(prime_field));
                        let x_affine = DynResidue::new(
                            &U192::from_be_slice(q.x().unwrap().as_slice()),
                            modulus,
                        );
                        let y_affine = DynResidue::new(
                            &U192::from_be_slice(q.y().unwrap().as_slice()),
                            modulus,
                        );
                        let z = DynResidue::new(&U192::from_be_slice(k), modulus);
                        let x_jacobian = x_affine * z * z;
                        let y_jacobian = y_affine * z * z * z;
                        sw_x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                        sw_y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                    }
                    32 => {
                        let sw_k = p256::Scalar::from(
                            elliptic_curve::ScalarPrimitive::from_slice(sw_k).unwrap(),
                        );
                        let q = p256::AffinePoint::GENERATOR
                            .mul(sw_k)
                            .to_affine()
                            .to_encoded_point(false);
                        let modulus = DynResidueParams::new(&U256::from_be_slice(prime_field));
                        let x_affine = DynResidue::new(
                            &U256::from_be_slice(q.x().unwrap().as_slice()),
                            modulus,
                        );
                        let y_affine = DynResidue::new(
                            &U256::from_be_slice(q.y().unwrap().as_slice()),
                            modulus,
                        );
                        let z = DynResidue::new(&U256::from_be_slice(k), modulus);
                        let x_jacobian = x_affine * z * z;
                        let y_jacobian = y_affine * z * z * z;
                        sw_x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                        sw_y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                    }
                    _ => unimplemented!(),
                };

                for (a, b) in x.iter().zip(sw_x.iter()) {
                    assert_eq!(a, b);
                }

                for (a, b) in y.iter().zip(sw_y.iter()) {
                    assert_eq!(a, b);
                }
            }
        }
    }

    #[test]
    #[cfg(feature = "esp32c2")]
    fn test_ecc_finite_field_division(mut ctx: Context<'static>) {
        let rng = Rng::new();
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 64];
            let (k, y) = t1.split_at_mut(prime_field.len());
            let (y, _) = y.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_inv {
                loop {
                    rng.read(k);
                    rng.read(y);
                    let is_zero = k.iter().all(|&elt| elt == 0) || y.iter().all(|&elt| elt == 0);
                    let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b)
                        || y.iter().zip(prime_field).all(|(&a, &b)| a == b);
                    if is_zero == false && is_modulus == false {
                        break;
                    }
                }
                let t2 = &mut [0_u8; 96];
                let (sw_y, sw_k) = t2.split_at_mut(prime_field.len());
                let (sw_k, sw_res) = sw_k.split_at_mut(prime_field.len());
                let (sw_res, _) = sw_res.split_at_mut(prime_field.len());
                sw_y.copy_from_slice(y);
                sw_k.copy_from_slice(k);
                let curve = match prime_field.len() {
                    24 => &EllipticCurve::P192,
                    32 => &EllipticCurve::P256,
                    _ => unimplemented!(),
                };

                ctx.ecc
                    .finite_field_division(curve, k, y)
                    .expect("Inputs data doesn't match the key length selected.");

                match prime_field.len() {
                    24 => {
                        let modulus = DynResidueParams::new(&U192::from_be_slice(prime_field));
                        let sw_y = DynResidue::new(&U192::from_be_slice(sw_y), modulus);
                        let sw_k = DynResidue::new(&U192::from_be_slice(sw_k), modulus);
                        let sw_inv_k = sw_k.invert().0;
                        sw_res.copy_from_slice(
                            sw_y.mul(&sw_inv_k).retrieve().to_be_bytes().as_slice(),
                        );
                    }
                    32 => {
                        let modulus = DynResidueParams::new(&U256::from_be_slice(prime_field));
                        let sw_y = DynResidue::new(&U256::from_be_slice(sw_y), modulus);
                        let sw_k = DynResidue::new(&U256::from_be_slice(sw_k), modulus);
                        let sw_inv_k = sw_k.invert().0;
                        sw_res.copy_from_slice(
                            sw_y.mul(&sw_inv_k).retrieve().to_be_bytes().as_slice(),
                        );
                    }
                    _ => unimplemented!(),
                };

                for (a, b) in y.iter().zip(sw_res) {
                    assert_eq!(a, b);
                }
            }
        }
    }

    #[test]
    #[cfg(feature = "esp32h2")]
    fn test_ecc_point_addition_256(mut ctx: Context<'static>) {
        const ECC_256_X: [u8; 32] = [
            0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4, 0xA0, 0x33, 0xEB, 0x2D, 0x81, 0x7D,
            0x03, 0x77, 0xF2, 0x40, 0xA4, 0x63, 0xE5, 0xE6, 0xBC, 0xF8, 0x47, 0x42, 0x2C, 0xE1,
            0xF2, 0xD1, 0x17, 0x6B,
        ];

        const ECC_256_Y: [u8; 32] = [
            0xF5, 0x51, 0xBF, 0x37, 0x68, 0x40, 0xB6, 0xCB, 0xCE, 0x5E, 0x31, 0x6B, 0x57, 0x33,
            0xCE, 0x2B, 0x16, 0x9E, 0x0F, 0x7C, 0x4A, 0xEB, 0xE7, 0x8E, 0x9B, 0x7F, 0x1A, 0xFE,
            0xE2, 0x42, 0xE3, 0x4F,
        ];

        const ECC_256_RES_X: [u8; 32] = [
            0xf, 0xc7, 0xd6, 0x91, 0x83, 0x84, 0x7d, 0x1e, 0xcf, 0xdd, 0x61, 0x4f, 0x27, 0x42,
            0x4b, 0x80, 0x43, 0xde, 0xfc, 0xdc, 0x52, 0x7d, 0x0e, 0x57, 0xad, 0xb5, 0xd1, 0xac,
            0x59, 0x8f, 0x97, 0x9a,
        ];

        const ECC_256_RES_Y: [u8; 32] = [
            0xd1, 0xf5, 0xd2, 0x90, 0x31, 0xbe, 0x59, 0xfd, 0xd0, 0x8b, 0x66, 0x88, 0x63, 0xc4,
            0x7f, 0xe7, 0x5f, 0x6d, 0x34, 0xe5, 0x38, 0x82, 0x33, 0x05, 0xf9, 0x6a, 0x78, 0x7f,
            0x5e, 0x88, 0x26, 0x41,
        ];

        const ECC_256_RES_Z: [u8; 32] = [
            0xea, 0xa3, 0x7e, 0x6f, 0xd0, 0x80, 0x6c, 0x97, 0x9d, 0xbd, 0x62, 0xd6, 0xae, 0x66,
            0x9c, 0x57, 0x2c, 0x3c, 0x1f, 0xf8, 0x94, 0xd6, 0xcf, 0x1d, 0x37, 0xff, 0x34, 0xfc,
            0xc5, 0x85, 0xc6, 0x9f,
        ];

        let mut x_256 = ECC_256_X.clone();
        let mut y_256 = ECC_256_Y.clone();

        let mut z: [u8; 32] = [0u8; 32];
        z[0] = 0x1;

        let mut x_256_1 = ECC_256_X.clone();
        let mut y_256_1 = ECC_256_Y.clone();

        ctx.ecc
            .affine_point_addition(
                &EllipticCurve::P256,
                &mut x_256,
                &mut y_256,
                &mut x_256_1,
                &mut y_256_1,
                &mut z,
            )
            .unwrap();

        assert_eq!(x_256_1, ECC_256_RES_X);
        assert_eq!(y_256_1, ECC_256_RES_Y);
        assert_eq!(z, ECC_256_RES_Z);
    }

    #[test]
    #[cfg(feature = "esp32h2")]
    fn test_ecc_point_addition_192(mut ctx: Context<'static>) {
        const ECC_192_X: [u8; 24] = [
            0x12, 0x10, 0xFF, 0x82, 0xFD, 0x0A, 0xFF, 0xF4, 0x00, 0x88, 0xA1, 0x43, 0xEB, 0x20,
            0xBF, 0x7C, 0xF6, 0x90, 0x30, 0xB0, 0x0E, 0xA8, 0x8D, 0x18,
        ];

        const ECC_192_Y: [u8; 24] = [
            0x11, 0x48, 0x79, 0x1E, 0xA1, 0x77, 0xF9, 0x73, 0xD5, 0xCD, 0x24, 0x6B, 0xED, 0x11,
            0x10, 0x63, 0x78, 0xDA, 0xC8, 0xFF, 0x95, 0x2B, 0x19, 0x07,
        ];

        const ECC_192_RES_X: [u8; 24] = [
            0x6, 0x6c, 0xd8, 0x6e, 0x9b, 0xef, 0x62, 0x7a, 0x54, 0xd3, 0x75, 0xfa, 0xdb, 0x36,
            0x83, 0xc1, 0x8f, 0x2b, 0xeb, 0xbd, 0x18, 0x1, 0xf, 0xe1,
        ];

        const ECC_192_RES_Y: [u8; 24] = [
            0x42, 0xc0, 0x1d, 0x71, 0x0c, 0x1e, 0x4e, 0x29, 0x22, 0x69, 0x21, 0x5b, 0x05, 0x91,
            0xea, 0x60, 0xcf, 0x05, 0x07, 0xfd, 0x79, 0x29, 0x6c, 0x19,
        ];

        const ECC_192_RES_Z: [u8; 24] = [
            0x22, 0x90, 0xf2, 0x3c, 0x42, 0xef, 0xf2, 0xe7, 0xaa, 0x9b, 0x49, 0xd6, 0xda, 0x23,
            0x20, 0xc6, 0xf0, 0xb4, 0x91, 0xff, 0x2b, 0x57, 0x32, 0xe,
        ];

        let mut x_192 = ECC_192_X.clone();
        let mut y_192 = ECC_192_Y.clone();
        let mut z: [u8; 24] = [0u8; 24];
        z[0] = 0x1;

        let mut x_192_1 = ECC_192_X.clone();
        let mut y_192_1 = ECC_192_Y.clone();

        ctx.ecc
            .affine_point_addition(
                &EllipticCurve::P192,
                &mut x_192,
                &mut y_192,
                &mut x_192_1,
                &mut y_192_1,
                &mut z,
            )
            .unwrap();

        assert_eq!(x_192_1, ECC_192_RES_X);
        assert_eq!(y_192_1, ECC_192_RES_Y);
        assert_eq!(z, ECC_192_RES_Z);
    }

    #[test]
    #[cfg(feature = "esp32h2")]
    fn test_ecc_mod_operations_256(mut ctx: Context<'static>) {
        const ECC_256_X: [u8; 32] = [
            0x96, 0xC2, 0x98, 0xD8, 0x45, 0x39, 0xA1, 0xF4, 0xA0, 0x33, 0xEB, 0x2D, 0x81, 0x7D,
            0x03, 0x77, 0xF2, 0x40, 0xA4, 0x63, 0xE5, 0xE6, 0xBC, 0xF8, 0x47, 0x42, 0x2C, 0xE1,
            0xF2, 0xD1, 0x17, 0x6B,
        ];

        const ECC_256_Y: [u8; 32] = [
            0xF5, 0x51, 0xBF, 0x37, 0x68, 0x40, 0xB6, 0xCB, 0xCE, 0x5E, 0x31, 0x6B, 0x57, 0x33,
            0xCE, 0x2B, 0x16, 0x9E, 0x0F, 0x7C, 0x4A, 0xEB, 0xE7, 0x8E, 0x9B, 0x7F, 0x1A, 0xFE,
            0xE2, 0x42, 0xE3, 0x4F,
        ];

        const ECC_256_NUM: [u8; 32] = [
            0x20, 0x56, 0x14, 0xB6, 0xAF, 0x94, 0xA0, 0xB6, 0x0C, 0xDF, 0x13, 0x1A, 0xE6, 0xBF,
            0x57, 0x87, 0xF1, 0x02, 0x73, 0x96, 0x53, 0x1A, 0xBC, 0xA9, 0x0F, 0x5E, 0xA1, 0xFC,
            0x0E, 0xFC, 0x9D, 0x9B,
        ];

        const ECC_256_DEN: [u8; 32] = [
            0x54, 0x3B, 0x11, 0x78, 0xC4, 0xCA, 0x52, 0xFD, 0xCC, 0x89, 0x51, 0x0F, 0xFE, 0x7D,
            0x37, 0x83, 0x81, 0xD5, 0x2E, 0x58, 0x42, 0xF9, 0x4F, 0x19, 0x9A, 0x79, 0x78, 0x98,
            0xFA, 0x95, 0x40, 0x2E,
        ];

        const ECC_256_ADD_RES: [u8; 32] = [
            0x8B, 0x14, 0x58, 0x10, 0xAE, 0x79, 0x57, 0xC0, 0x6F, 0x92, 0x1C, 0x99, 0xD8, 0xB0,
            0xD1, 0xA2, 0x08, 0xDF, 0xB3, 0xDF, 0x2F, 0xD2, 0xA4, 0x87, 0xE3, 0xC1, 0x46, 0xDF,
            0xD5, 0x14, 0xFB, 0xBA,
        ];

        const ECC_256_SUB_RES: [u8; 32] = [
            0xA1, 0x70, 0xD9, 0xA0, 0xDD, 0xF8, 0xEA, 0x28, 0xD2, 0xD4, 0xB9, 0xC2, 0x29, 0x4A,
            0x35, 0x4B, 0xDC, 0xA2, 0x94, 0xE7, 0x9A, 0xFB, 0xD4, 0x69, 0xAC, 0xC2, 0x11, 0xE3,
            0x0F, 0x8F, 0x34, 0x1B,
        ];

        const ECC_256_MUL_RES: [u8; 32] = [
            0x18, 0x4D, 0xCE, 0xCC, 0x1A, 0xA8, 0xEC, 0x72, 0xD7, 0x31, 0xDA, 0x41, 0x8C, 0x75,
            0x6B, 0xF1, 0x2A, 0x2E, 0x5B, 0x53, 0x8D, 0xCA, 0x79, 0x61, 0x6B, 0x46, 0xF9, 0x2E,
            0x27, 0xB5, 0x43, 0x15,
        ];

        const ECC_256_INV_MUL_RES: [u8; 32] = [
            0x33, 0xF3, 0x55, 0x3B, 0x46, 0x8A, 0x13, 0xC0, 0x1D, 0x7E, 0x41, 0xA6, 0xFF, 0x53,
            0xFD, 0x78, 0xD5, 0xC0, 0xE5, 0x9F, 0x78, 0xD1, 0x86, 0x66, 0x77, 0x3C, 0x6E, 0xEF,
            0x58, 0xF6, 0x29, 0x34,
        ];

        let mut x_256 = ECC_256_X.clone();
        let mut y_256 = ECC_256_Y.clone();

        ctx.ecc
            .mod_operations(
                &EllipticCurve::P256,
                &mut x_256,
                &mut y_256,
                WorkMode::ModAdd,
            )
            .unwrap();
        assert_eq!(x_256, ECC_256_ADD_RES);

        let mut x_256 = ECC_256_X.clone();
        let mut y_256 = ECC_256_Y.clone();
        ctx.ecc
            .mod_operations(
                &EllipticCurve::P256,
                &mut x_256,
                &mut y_256,
                WorkMode::ModSub,
            )
            .unwrap();
        assert_eq!(x_256, ECC_256_SUB_RES);

        let mut x_256 = ECC_256_X.clone();
        let mut y_256 = ECC_256_Y.clone();
        ctx.ecc
            .mod_operations(
                &EllipticCurve::P256,
                &mut x_256,
                &mut y_256,
                WorkMode::ModMulti,
            )
            .unwrap();
        assert_eq!(y_256, ECC_256_MUL_RES);

        let mut x_256 = ECC_256_NUM.clone();
        let mut y_256 = ECC_256_DEN.clone();
        ctx.ecc
            .mod_operations(
                &EllipticCurve::P256,
                &mut x_256,
                &mut y_256,
                WorkMode::ModDiv,
            )
            .unwrap();
        assert_eq!(y_256, ECC_256_INV_MUL_RES);
    }

    #[test]
    #[cfg(feature = "esp32h2")]
    fn test_ecc_mod_operations_192(mut ctx: Context<'static>) {
        const ECC_192_X: [u8; 24] = [
            0x1A, 0x80, 0xA1, 0x5F, 0x1F, 0xB7, 0x59, 0x1B, 0x9F, 0xD7, 0xFB, 0xAE, 0xA9, 0xF9,
            0x1E, 0xBA, 0x67, 0xAE, 0x57, 0xB7, 0x27, 0x80, 0x9E, 0x1A,
        ];

        const ECC_192_Y: [u8; 24] = [
            0x59, 0xC6, 0x3D, 0xD3, 0xD7, 0xDF, 0xA3, 0x44, 0x7C, 0x75, 0x52, 0xB4, 0x42, 0xF3,
            0xFC, 0xA6, 0x0F, 0xA8, 0x8A, 0x8D, 0x1F, 0xA3, 0xDF, 0x54,
        ];

        const ECC_192_NUM: [u8; 24] = [
            0xBA, 0x0F, 0x2C, 0xD8, 0xBE, 0xCC, 0x2D, 0xD3, 0xD5, 0x74, 0xBD, 0x8C, 0xF3, 0x3E,
            0x3B, 0x7A, 0xA4, 0xD0, 0x71, 0xEC, 0x85, 0xF6, 0x70, 0x00,
        ];

        const ECC_192_DEN: [u8; 24] = [
            0x15, 0xF9, 0x20, 0xD8, 0x46, 0x5C, 0x03, 0x97, 0x4A, 0x10, 0xEF, 0x8A, 0xFB, 0x12,
            0x2E, 0x65, 0x6E, 0xD6, 0x79, 0x1E, 0x65, 0x6F, 0x3E, 0x64,
        ];

        const ECC_192_ADD_RES: [u8; 24] = [
            0x73, 0x46, 0xDF, 0x32, 0xF7, 0x96, 0xFD, 0x5F, 0x1B, 0x4D, 0x4E, 0x63, 0xEC, 0xEC,
            0x1B, 0x61, 0x77, 0x56, 0xE2, 0x44, 0x47, 0x23, 0x7E, 0x6F,
        ];

        const ECC_192_SUB_RES: [u8; 24] = [
            0xF2, 0xE1, 0x35, 0x41, 0xF9, 0xA0, 0x21, 0xEB, 0x58, 0x5A, 0x88, 0x94, 0x66, 0x06,
            0x22, 0x13, 0x58, 0x06, 0xCD, 0x29, 0x08, 0xDD, 0xBE, 0xC5,
        ];

        const ECC_192_MUL_RES: [u8; 24] = [
            0xB5, 0xB9, 0xFF, 0xBC, 0x52, 0xC8, 0xB8, 0x36, 0x8C, 0xFB, 0xA5, 0xCE, 0x1E, 0x7B,
            0xE6, 0xF3, 0x8F, 0x79, 0x71, 0xCF, 0xD6, 0xF3, 0x41, 0xE6,
        ];

        const ECC_192_INV_MUL_RES: [u8; 24] = [
            0x6B, 0xB3, 0x6B, 0x2B, 0x56, 0x6A, 0xE5, 0xF7, 0x75, 0x82, 0xF0, 0xCC, 0x93, 0x63,
            0x40, 0xF8, 0xEF, 0x35, 0x2A, 0xAF, 0xBD, 0x56, 0xE9, 0x29,
        ];

        let mut x_192 = ECC_192_X.clone();
        let mut y_192 = ECC_192_Y.clone();

        ctx.ecc
            .mod_operations(
                &EllipticCurve::P192,
                &mut x_192,
                &mut y_192,
                WorkMode::ModAdd,
            )
            .unwrap();
        assert_eq!(x_192, ECC_192_ADD_RES);

        let mut x_192 = ECC_192_X.clone();
        let mut y_192 = ECC_192_Y.clone();
        ctx.ecc
            .mod_operations(
                &EllipticCurve::P192,
                &mut x_192,
                &mut y_192,
                WorkMode::ModSub,
            )
            .unwrap();
        assert_eq!(x_192, ECC_192_SUB_RES);

        let mut x_192 = ECC_192_X.clone();
        let mut y_192 = ECC_192_Y.clone();
        ctx.ecc
            .mod_operations(
                &EllipticCurve::P192,
                &mut x_192,
                &mut y_192,
                WorkMode::ModMulti,
            )
            .unwrap();
        assert_eq!(y_192, ECC_192_MUL_RES);

        let mut x_192 = ECC_192_NUM.clone();
        let mut y_192 = ECC_192_DEN.clone();
        ctx.ecc
            .mod_operations(
                &EllipticCurve::P192,
                &mut x_192,
                &mut y_192,
                WorkMode::ModDiv,
            )
            .unwrap();
        assert_eq!(y_192, ECC_192_INV_MUL_RES);
    }
}

#[cfg(not(esp32c2))]
#[embedded_test::tests(default_timeout = 5, executor = hil_test::Executor::new())]
mod rsa_tests {
    use crypto_bigint::{U512, U1024, Uint};
    use esp_hal::{
        Blocking,
        rsa::{
            Rsa,
            RsaBackend,
            RsaContext,
            RsaModularExponentiation,
            RsaModularMultiplication,
            RsaMultiplication,
            operand_sizes::*,
        },
    };

    const BIGNUM_1: U512 = Uint::from_be_hex(
        "c7f61058f96db3bd87dbab08ab03b4f7f2f864eac249144adea6a65f97803b719d8ca980b7b3c0389c1c7c6\
    7dc353c5e0ec11f5fc8ce7f6073796cc8f73fa878",
    );
    const BIGNUM_2: U512 = Uint::from_be_hex(
        "1763db3344e97be15d04de4868badb12a38046bb793f7630d87cf100aa1c759afac15a01f3c4c83ec2d2f66\
    6bd22f71c3c1f075ec0e2cb0cb29994d091b73f51",
    );
    const BIGNUM_3: U512 = Uint::from_be_hex(
        "6b6bb3d2b6cbeb45a769eaa0384e611e1b89b0c9b45a045aca1c5fd6e8785b38df7118cf5dd45b9b63d293b\
    67aeafa9ba25feb8712f188cb139b7d9b9af1c361",
    );

    const EXPECTED_EXPONENTIATION_OUTPUT: [u32; U512::LIMBS] = [
        1601059419, 3994655875, 2600857657, 1530060852, 64828275, 4221878473, 2751381085,
        1938128086, 625895085, 2087010412, 2133352910, 101578249, 3798099415, 3357588690,
        2065243474, 330914193,
    ];
    const EXPECTED_MODULAR_MULT_OUTPUT: [u32; U512::LIMBS] = [
        1868256644, 833470784, 4187374062, 2684021027, 191862388, 1279046003, 1929899870,
        4209598061, 3830489207, 1317083344, 2666864448, 3701382766, 3232598924, 2904609522,
        747558855, 479377985,
    ];
    const EXPECTED_MULT_OUTPUT: [u32; U1024::LIMBS] = [
        1264702968, 3552243420, 2602501218, 498422249, 2431753435, 2307424767, 349202767,
        2269697177, 1525551459, 3623276361, 3146383138, 191420847, 4252021895, 9176459, 301757643,
        4220806186, 434407318, 3722444851, 1850128766, 928651940, 107896699, 563405838, 1834067613,
        1289630401, 3145128058, 3300293535, 3077505758, 1926648662, 1264151247, 3626086486,
        3701894076, 306518743,
    ];

    struct Context<'a> {
        rsa: Rsa<'a, Blocking>,
    }

    const fn compute_r(modulus: &U512) -> U512 {
        let mut d = [0_u32; U512::LIMBS * 2 + 1];
        d[d.len() - 1] = 1;
        let d = Uint::from_words(d);
        d.const_rem(&modulus.resize()).0.resize()
    }

    const fn compute_mprime(modulus: &U512) -> u32 {
        let m_inv = modulus.inv_mod2k(32).to_words()[0];
        (-1 * m_inv as i64 & (u32::MAX as i64)) as u32
    }

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        #[allow(unused_mut)]
        let mut rsa = Rsa::new(peripherals.RSA);

        #[cfg(not(esp32))]
        {
            rsa.disable_constant_time(true);
            rsa.search_acceleration(true);
        }

        Context { rsa }
    }

    #[test]
    fn test_modular_exponentiation(mut ctx: Context<'static>) {
        let mut outbuf = [0_u32; U512::LIMBS];
        let mut mod_exp = RsaModularExponentiation::<Op512, _>::new(
            &mut ctx.rsa,
            BIGNUM_2.as_words(),
            BIGNUM_3.as_words(),
            compute_mprime(&BIGNUM_3),
        );
        let r = compute_r(&BIGNUM_3);
        mod_exp.start_exponentiation(BIGNUM_1.as_words(), r.as_words());
        mod_exp.read_results(&mut outbuf);
        hil_test::assert_eq!(EXPECTED_EXPONENTIATION_OUTPUT, outbuf);
    }

    #[test]
    async fn modular_async_exponentiation(ctx: Context<'static>) {
        let mut rsa = ctx.rsa.into_async();

        let mut outbuf = [0_u32; U512::LIMBS];
        let mut mod_exp = RsaModularExponentiation::<Op512, _>::new(
            &mut rsa,
            BIGNUM_2.as_words(),
            BIGNUM_3.as_words(),
            compute_mprime(&BIGNUM_3),
        );
        let r = compute_r(&BIGNUM_3);
        mod_exp
            .exponentiation(BIGNUM_1.as_words(), r.as_words(), &mut outbuf)
            .await;
        hil_test::assert_eq!(EXPECTED_EXPONENTIATION_OUTPUT, outbuf);
    }

    #[test]
    fn test_modular_multiplication(mut ctx: Context<'static>) {
        let mut outbuf = [0_u32; U512::LIMBS];
        let r = compute_r(&BIGNUM_3);
        let mut mod_multi = RsaModularMultiplication::<Op512, _>::new(
            &mut ctx.rsa,
            BIGNUM_1.as_words(),
            BIGNUM_3.as_words(),
            r.as_words(),
            compute_mprime(&BIGNUM_3),
        );
        mod_multi.start_modular_multiplication(BIGNUM_2.as_words());
        mod_multi.read_results(&mut outbuf);
        hil_test::assert_eq!(EXPECTED_MODULAR_MULT_OUTPUT, outbuf);
    }

    #[test]
    async fn test_async_modular_multiplication(ctx: Context<'static>) {
        let mut rsa = ctx.rsa.into_async();

        let mut outbuf = [0_u32; U512::LIMBS];
        let r = compute_r(&BIGNUM_3);
        let mut mod_multi = RsaModularMultiplication::<Op512, _>::new(
            &mut rsa,
            BIGNUM_1.as_words(),
            BIGNUM_3.as_words(),
            r.as_words(),
            compute_mprime(&BIGNUM_3),
        );
        mod_multi
            .modular_multiplication(BIGNUM_2.as_words(), &mut outbuf)
            .await;
        hil_test::assert_eq!(EXPECTED_MODULAR_MULT_OUTPUT, outbuf);
    }

    #[test]
    fn test_multiplication(mut ctx: Context<'static>) {
        let mut outbuf = [0_u32; U1024::LIMBS];

        let operand_a = BIGNUM_1.as_words();
        let operand_b = BIGNUM_2.as_words();

        let mut rsamulti = RsaMultiplication::<Op512, _>::new(&mut ctx.rsa, operand_a);
        rsamulti.start_multiplication(operand_b);
        rsamulti.read_results(&mut outbuf);

        hil_test::assert_eq!(EXPECTED_MULT_OUTPUT, outbuf)
    }

    #[test]
    async fn test_async_multiplication(ctx: Context<'static>) {
        let mut outbuf = [0_u32; U1024::LIMBS];

        let mut rsa = ctx.rsa.into_async();

        let operand_a = BIGNUM_1.as_words();
        let operand_b = BIGNUM_2.as_words();

        let mut rsamulti = RsaMultiplication::<Op512, _>::new(&mut rsa, operand_a);
        rsamulti.multiplication(operand_b, &mut outbuf).await;

        hil_test::assert_eq!(EXPECTED_MULT_OUTPUT, outbuf);
    }

    #[test]
    async fn test_rsa_work_queue() {
        // FIXME
        let mut backend = RsaBackend::new(unsafe { esp_hal::peripherals::RSA::steal() });
        let _rsa = backend.start();

        let mut rsa = RsaContext::new();

        // Output buffers
        let mut multi_outbuf = [0_u32; U1024::LIMBS];
        let mut outbuf = [0_u32; U512::LIMBS];

        // Software-derived values
        let r = compute_r(&BIGNUM_3);
        let mprime = compute_mprime(&BIGNUM_3);

        defmt::info!("Multiply");

        let mut handle =
            rsa.multiply::<Op512>(BIGNUM_1.as_words(), BIGNUM_2.as_words(), &mut multi_outbuf);
        handle.wait().await;
        core::mem::drop(handle);

        hil_test::assert_eq!(EXPECTED_MULT_OUTPUT, multi_outbuf);

        defmt::info!("Modular multiply");

        let mut handle = rsa.modular_multiply::<Op512>(
            BIGNUM_1.as_words(),
            BIGNUM_2.as_words(),
            BIGNUM_3.as_words(),
            r.as_words(),
            mprime,
            &mut outbuf,
        );
        handle.wait().await;
        core::mem::drop(handle);

        hil_test::assert_eq!(EXPECTED_MODULAR_MULT_OUTPUT, outbuf);

        defmt::info!("Modular exponentiate");

        let mut handle = rsa.modular_exponentiate::<Op512>(
            BIGNUM_1.as_words(),
            BIGNUM_2.as_words(),
            BIGNUM_3.as_words(),
            r.as_words(),
            mprime,
            &mut outbuf,
        );
        handle.wait().await;
        core::mem::drop(handle);

        hil_test::assert_eq!(EXPECTED_EXPONENTIATION_OUTPUT, outbuf);
    }
}

#[embedded_test::tests(default_timeout = 6)]
mod sha_tests {
    use digest::{Digest, Update};
    #[cfg(not(esp32))]
    use esp_hal::sha::Sha224;
    #[cfg(any(esp32, esp32s2, esp32s3))]
    use esp_hal::sha::{Sha384, Sha512};
    #[cfg(any(esp32s2, esp32s3))]
    use esp_hal::sha::{Sha512_224, Sha512_256};
    use esp_hal::{
        clock::CpuClock,
        rng::{Rng, TrngSource},
        sha::{Sha, Sha1, Sha256, ShaAlgorithm, ShaBackend, ShaDigest},
    };
    use nb::block;

    const SOURCE_DATA: &[u8] = &[b'a'; 258];

    pub struct Context {
        _rng_source: TrngSource<'static>,
        sha: Sha<'static>,
    }

    #[track_caller]
    fn assert_sw_hash<D: Digest>(algo: &str, input: &[u8], expected_output: &[u8]) {
        let mut hasher = D::new();
        hasher.update(input);
        let soft_result = hasher.finalize();

        hil_test::assert_eq!(
            expected_output,
            &soft_result[..],
            "Output mismatch with {}",
            algo
        );
    }

    fn hash_sha<S: ShaAlgorithm>(sha: &mut Sha<'static>, mut input: &[u8], output: &mut [u8]) {
        let mut digest = sha.start::<S>();
        while !input.is_empty() {
            input = block!(digest.update(input)).unwrap();
        }
        block!(digest.finish(output)).unwrap();
    }

    fn hash_digest<'a, S: ShaAlgorithm>(
        sha: &'a mut Sha<'static>,
        input: &[u8],
        output: &mut [u8],
    ) {
        let mut hasher = ShaDigest::<S, _>::new(sha);
        Update::update(&mut hasher, input);
        output.copy_from_slice(&digest::FixedOutput::finalize_fixed(hasher));
    }

    /// A simple test using the Sha trait. This will compare the result with a
    /// software implementation.
    #[track_caller]
    fn assert_sha<S: ShaAlgorithm, const N: usize>(sha: &mut Sha<'static>, input: &[u8]) {
        let mut output = [0u8; N];
        hash_sha::<S>(sha, input, &mut output);

        // Compare against Software result.
        match N {
            20 => assert_sw_hash::<sha1::Sha1>("SHA-1", input, &output),
            28 => assert_sw_hash::<sha2::Sha224>("SHA-224", input, &output),
            32 => assert_sw_hash::<sha2::Sha256>("SHA-256", input, &output),
            48 => assert_sw_hash::<sha2::Sha384>("SHA-384", input, &output),
            64 => assert_sw_hash::<sha2::Sha512>("SHA-512", input, &output),
            _ => unreachable!(),
        }
    }

    /// A simple test using the Digest trait. This will compare the result with a
    /// software implementation.
    #[track_caller]
    fn assert_digest<'a, S: ShaAlgorithm, const N: usize>(sha: &'a mut Sha<'static>, input: &[u8]) {
        let mut output = [0u8; N];
        hash_digest::<S>(sha, input, &mut output);

        // Compare against Software result.
        match N {
            20 => assert_sw_hash::<sha1::Sha1>("SHA-1", input, &output),
            28 => assert_sw_hash::<sha2::Sha224>("SHA-224", input, &output),
            32 => assert_sw_hash::<sha2::Sha256>("SHA-256", input, &output),
            48 => assert_sw_hash::<sha2::Sha384>("SHA-384", input, &output),
            64 => assert_sw_hash::<sha2::Sha512>("SHA-512", input, &output),
            _ => unreachable!(),
        }
    }

    #[allow(unused_mut)]
    fn with_random_data(
        mut f: impl FnMut(
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
            (&[u8], &mut [u8]),
        ),
    ) {
        // Make sure this is not a multiple of the block size
        const BUFFER_LEN: usize = 264;

        let mut sha1_random = [0u8; BUFFER_LEN];
        let mut sha224_random = [0u8; BUFFER_LEN];
        let mut sha256_random = [0u8; BUFFER_LEN];
        let mut sha384_random = [0u8; BUFFER_LEN];
        let mut sha512_random = [0u8; BUFFER_LEN];

        let rng = Rng::new();

        // Fill source data with random data
        rng.read(&mut sha1_random);
        #[cfg(not(esp32))]
        rng.read(&mut sha224_random);
        rng.read(&mut sha256_random);
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rng.read(&mut sha384_random);
        #[cfg(any(esp32, esp32s2, esp32s3))]
        rng.read(&mut sha512_random);

        for size in [1, 64, 128, 256, BUFFER_LEN] {
            let mut sha1_output = [0u8; 20];
            let mut sha224_output = [0u8; 28];
            let mut sha256_output = [0u8; 32];
            let mut sha384_output = [0u8; 48];
            let mut sha512_output = [0u8; 64];
            f(
                (&sha1_random[..size], &mut sha1_output[..]),
                (&sha224_random[..size], &mut sha224_output[..]),
                (&sha256_random[..size], &mut sha256_output[..]),
                (&sha384_random[..size], &mut sha384_output[..]),
                (&sha512_random[..size], &mut sha512_output[..]),
            );

            // Calculate software result to compare against
            assert_sw_hash::<sha1::Sha1>("SHA-1", &sha1_random[..size], &sha1_output);

            #[cfg(not(esp32))]
            assert_sw_hash::<sha2::Sha224>("SHA-224", &sha224_random[..size], &sha224_output);

            assert_sw_hash::<sha2::Sha256>("SHA-256", &sha256_random[..size], &sha256_output);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            assert_sw_hash::<sha2::Sha384>("SHA-384", &sha384_random[..size], &sha384_output);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            assert_sw_hash::<sha2::Sha512>("SHA-512", &sha512_random[..size], &sha512_output);
        }
    }

    #[init]
    fn init() -> Context {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);

        Context {
            _rng_source: TrngSource::new(peripherals.RNG, peripherals.ADC1),
            sha: Sha::new(peripherals.SHA),
        }
    }

    #[test]
    #[cfg(any(esp32s2, esp32s3))]
    fn test_sha_512_224(mut ctx: Context) {
        let expected_output = [
            0x19, 0xf2, 0xb3, 0x88, 0x22, 0x86, 0x94, 0x38, 0xee, 0x24, 0xc1, 0xc3, 0xb0, 0xb1,
            0x21, 0x6a, 0xf4, 0x81, 0x14, 0x8f, 0x4, 0x34, 0xfd, 0xd7, 0x54, 0x3, 0x2b, 0x88,
        ];
        let mut output = [0u8; 28];
        hash_sha::<Sha512_224>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);

        let mut output = [0u8; 28];
        hash_digest::<Sha512_224>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);
    }

    #[test]
    #[cfg(any(esp32s2, esp32s3))]
    fn test_sha_512_256(mut ctx: Context) {
        let expected_output = [
            0xb7, 0x49, 0x4e, 0xe1, 0xdb, 0xcd, 0xe5, 0x47, 0x5a, 0x61, 0x25, 0xac, 0x27, 0xc2,
            0x1b, 0x53, 0xcd, 0x6b, 0x16, 0x33, 0xb4, 0x94, 0xac, 0xa4, 0x2a, 0xe6, 0x99, 0x2f,
            0xe7, 0xd, 0x83, 0x19,
        ];
        let mut output = [0u8; 32];
        hash_sha::<Sha512_256>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);

        let mut output = [0u8; 32];
        hash_digest::<Sha512_256>(&mut ctx.sha, SOURCE_DATA, &mut output);
        assert_eq!(output, expected_output);
    }

    /// A test that runs a hashing on a digest of every size between 1 and 200
    /// inclusively.
    #[test]
    #[timeout(15)]
    fn test_digest_of_size_1_to_200(mut ctx: Context) {
        for i in 1..=200 {
            assert_sha::<Sha1, 20>(&mut ctx.sha, &SOURCE_DATA[..i]);
            assert_digest::<Sha1, 20>(&mut ctx.sha, &SOURCE_DATA[..i]);

            #[cfg(not(esp32))]
            {
                assert_sha::<Sha224, 28>(&mut ctx.sha, &SOURCE_DATA[..i]);
                assert_digest::<Sha224, 28>(&mut ctx.sha, &SOURCE_DATA[..i]);
            }

            assert_sha::<Sha256, 32>(&mut ctx.sha, &SOURCE_DATA[..i]);
            assert_digest::<Sha256, 32>(&mut ctx.sha, &SOURCE_DATA[..i]);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                assert_sha::<Sha384, 48>(&mut ctx.sha, &SOURCE_DATA[..i]);
                assert_digest::<Sha384, 48>(&mut ctx.sha, &SOURCE_DATA[..i]);

                assert_sha::<Sha512, 64>(&mut ctx.sha, &SOURCE_DATA[..i]);
                assert_digest::<Sha512, 64>(&mut ctx.sha, &SOURCE_DATA[..i]);
            }
        }
    }

    #[cfg(not(esp32))]
    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Sha trait implementation
    #[test]
    fn test_sha_rolling(mut ctx: Context) {
        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            let mut sha1_remaining = sha1_p.0;
            let mut sha224_remaining = sha224_p.0;
            let mut sha256_remaining = sha256_p.0;
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha384_remaining = sha384_p.0;
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha512_remaining = sha512_p.0;

            let mut sha1 = esp_hal::sha::Context::<esp_hal::sha::Sha1>::new();
            let mut sha224 = esp_hal::sha::Context::<esp_hal::sha::Sha224>::new();
            let mut sha256 = esp_hal::sha::Context::<esp_hal::sha::Sha256>::new();
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha384 = esp_hal::sha::Context::<esp_hal::sha::Sha384>::new();
            #[cfg(any(esp32, esp32s2, esp32s3))]
            let mut sha512 = esp_hal::sha::Context::<esp_hal::sha::Sha512>::new();

            loop {
                let mut all_done = true;
                if !sha1_remaining.is_empty() {
                    let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha1);
                    sha1_remaining = block!(digest.update(sha1_remaining)).unwrap();
                    block!(digest.save(&mut sha1));
                    all_done = false;
                }
                if !sha224_remaining.is_empty() {
                    let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha224);
                    sha224_remaining = block!(digest.update(sha224_remaining)).unwrap();
                    block!(digest.save(&mut sha224));
                    all_done = false;
                }

                if !sha256_remaining.is_empty() {
                    let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha256);
                    sha256_remaining = block!(digest.update(sha256_remaining)).unwrap();
                    block!(digest.save(&mut sha256));
                    all_done = false;
                }

                #[cfg(any(esp32, esp32s2, esp32s3))]
                {
                    if !sha384_remaining.is_empty() {
                        let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha384);
                        sha384_remaining = block!(digest.update(sha384_remaining)).unwrap();
                        block!(digest.save(&mut sha384));
                        all_done = false;
                    }

                    if !sha512_remaining.is_empty() {
                        let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha512);
                        sha512_remaining = block!(digest.update(sha512_remaining)).unwrap();
                        block!(digest.save(&mut sha512));
                        all_done = false;
                    }
                }

                if all_done {
                    break;
                }
            }

            let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha1);
            block!(digest.finish(sha1_p.1)).unwrap();
            let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha224);
            block!(digest.finish(sha224_p.1)).unwrap();
            let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha256);
            block!(digest.finish(sha256_p.1)).unwrap();
            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha384);
                block!(digest.finish(sha384_p.1)).unwrap();
                let mut digest = ShaDigest::restore(&mut ctx.sha, &mut sha512);
                block!(digest.finish(sha512_p.1)).unwrap();
            }
        });
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically test the Digest trait implementation
    #[test]
    fn test_for_digest_rolling(mut ctx: Context) {
        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            // The Digest::update will consume the entirety of remaining. We don't need to
            // loop until remaining is fully consumed.

            let mut sha1 = ctx.sha.start::<esp_hal::sha::Sha1>();
            Update::update(&mut sha1, sha1_p.0);
            let sha1_output = digest::FixedOutput::finalize_fixed(sha1);
            sha1_p.1.copy_from_slice(&sha1_output);

            #[cfg(not(esp32))]
            {
                let mut sha224 = ctx.sha.start::<esp_hal::sha::Sha224>();
                Update::update(&mut sha224, sha224_p.0);
                let sha224_output = digest::FixedOutput::finalize_fixed(sha224);
                sha224_p.1.copy_from_slice(&sha224_output);
            }

            let mut sha256 = ctx.sha.start::<esp_hal::sha::Sha256>();
            Update::update(&mut sha256, sha256_p.0);
            let sha256_output = digest::FixedOutput::finalize_fixed(sha256);
            sha256_p.1.copy_from_slice(&sha256_output);

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut sha384 = ctx.sha.start::<esp_hal::sha::Sha384>();
                Update::update(&mut sha384, sha384_p.0);
                let sha384_output = digest::FixedOutput::finalize_fixed(sha384);
                sha384_p.1.copy_from_slice(&sha384_output);

                let mut sha512 = ctx.sha.start::<esp_hal::sha::Sha512>();
                Update::update(&mut sha512, sha512_p.0);
                let sha512_output = digest::FixedOutput::finalize_fixed(sha512);
                sha512_p.1.copy_from_slice(&sha512_output);
            }
        });
    }

    /// Calling finalize repeatedly will first return the result of the first hashing operation,
    /// then return result for hashing 0 bytes.
    #[test]
    fn test_repeated_finalize_is_empty_hash(_ctx: Context) {
        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        let mut sha1 = esp_hal::sha::Sha1Context::new();
        let sha1 = &mut sha1; // Trick to not pick Digest methods erroneously

        let mut empty_result = [0; 20];
        let mut hash_result = [0; 20];
        let mut repeated_result = [0; 20];

        sha1.finalize(&mut empty_result).wait_blocking();

        sha1.update(&SOURCE_DATA).wait_blocking();
        sha1.finalize(&mut hash_result).wait_blocking();

        sha1.finalize(&mut repeated_result).wait_blocking();

        assert_sw_hash::<sha1::Sha1>("SHA-1", &SOURCE_DATA, &hash_result);
        assert_sw_hash::<sha1::Sha1>("SHA-1", &[], &empty_result);
        hil_test::assert_eq!(empty_result, repeated_result);
    }

    #[test]
    #[cfg(not(esp32))]
    fn test_clone_separates_state(_ctx: Context) {
        use esp_hal::sha::Sha1Context;

        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        let mut sha1 = Sha1Context::new();

        let mut hash_result = [0; 20];
        let mut cloned_result = [0; 20];

        Sha1Context::update(&mut sha1, &SOURCE_DATA).wait_blocking();

        let mut sha1_clone = sha1.clone();

        Sha1Context::finalize(&mut sha1, &mut hash_result).wait_blocking();
        Sha1Context::finalize(&mut sha1_clone, &mut cloned_result).wait_blocking();

        assert_sw_hash::<sha1::Sha1>("SHA-1", &SOURCE_DATA, &hash_result);
        hil_test::assert_eq!(hash_result, cloned_result);
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically tests the SHA backend implementation
    #[test]
    fn test_for_digest_rolling_context(_ctx: Context) {
        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            {
                let mut sha1 = esp_hal::sha::Sha1Context::new();
                sha1.update(sha1_p.0).wait_blocking();
                sha1.finalize_into_slice(sha1_p.1).unwrap().wait_blocking();
            }

            #[cfg(not(esp32))]
            {
                let mut sha224 = esp_hal::sha::Sha224Context::new();
                sha224.update(sha224_p.0).wait_blocking();
                sha224
                    .finalize_into_slice(sha224_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            {
                let mut sha256 = esp_hal::sha::Sha256Context::new();
                sha256.update(sha256_p.0).wait_blocking();
                sha256
                    .finalize_into_slice(sha256_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut sha384 = esp_hal::sha::Sha384Context::new();
                sha384.update(sha384_p.0).wait_blocking();
                sha384
                    .finalize_into_slice(sha384_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                let mut sha512 = esp_hal::sha::Sha512Context::new();
                sha512.update(sha512_p.0).wait_blocking();
                sha512
                    .finalize_into_slice(sha512_p.1)
                    .unwrap()
                    .wait_blocking();
            }
        });
    }

    /// A rolling test that loops between hasher for every step to test
    /// interleaving. This specifically tests the SHA backend implementation
    #[test]
    fn test_for_digest_rolling_context_interleaved(_ctx: Context) {
        let mut sha_backend = ShaBackend::new(unsafe { esp_hal::peripherals::SHA::steal() });
        let _sha_driver = sha_backend.start();

        let mut sha1 = esp_hal::sha::Sha1Context::new();

        #[cfg(not(esp32))]
        let mut sha224 = esp_hal::sha::Sha224Context::new();

        let mut sha256 = esp_hal::sha::Sha256Context::new();

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let mut sha384 = esp_hal::sha::Sha384Context::new();

        #[cfg(any(esp32, esp32s2, esp32s3))]
        let mut sha512 = esp_hal::sha::Sha512Context::new();

        #[allow(unused)]
        with_random_data(|sha1_p, sha224_p, sha256_p, sha384_p, sha512_p| {
            {
                sha1.update(sha1_p.0).wait_blocking();
                sha1.finalize_into_slice(sha1_p.1).unwrap().wait_blocking();
            }

            #[cfg(not(esp32))]
            {
                sha224.update(sha224_p.0).wait_blocking();
                sha224
                    .finalize_into_slice(sha224_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            {
                sha256.update(sha256_p.0).wait_blocking();
                sha256
                    .finalize_into_slice(sha256_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                sha384.update(sha384_p.0).wait_blocking();
                sha384
                    .finalize_into_slice(sha384_p.1)
                    .unwrap()
                    .wait_blocking();
            }

            #[cfg(any(esp32, esp32s2, esp32s3))]
            {
                sha512.update(sha512_p.0).wait_blocking();
                sha512
                    .finalize_into_slice(sha512_p.1)
                    .unwrap()
                    .wait_blocking();
            }
        });
    }
}
