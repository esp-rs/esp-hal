//! ECC Test

//% CHIPS: esp32c2 esp32c6 esp32h2

#![no_std]
#![no_main]

use core::ops::Mul;

use crypto_bigint::{
    modular::runtime_mod::{DynResidue, DynResidueParams},
    Encoding,
    U192,
    U256,
};
use elliptic_curve::sec1::ToEncodedPoint;
#[cfg(feature = "esp32h2")]
use esp_hal::ecc::WorkMode;
use esp_hal::{
    ecc::{Ecc, EllipticCurve, Error},
    prelude::*,
    rng::Rng,
    Blocking,
};
use hex_literal::hex;
use hil_test as _;

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
    rng: Rng,
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let peripherals = esp_hal::init(config);

        let ecc = Ecc::new(peripherals.ECC);
        let rng = Rng::new(peripherals.RNG);

        Context { ecc, rng }
    }

    #[test]
    fn test_ecc_affine_point_multiplication(mut ctx: Context<'static>) {
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
                    ctx.rng.read(k);
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
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 96];
            let (k, x) = t1.split_at_mut(prime_field.len());
            let (x, y) = x.split_at_mut(prime_field.len());
            let (y, _) = y.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                loop {
                    ctx.rng.read(k);
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
    fn test_ecc_afine_point_verification_multiplication(mut ctx: Context<'static>) {
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
                    ctx.rng.read(k);
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
                    Err(Error::SizeMismatchCurve) => assert!(false, "Inputs data doesn't match the key length selected."),
                    Err(Error::PointNotOnSelectedCurve) => assert!(
                        false, "ECC failed while affine point verification + multiplication with x = {:02X?} and y = {:02X?}.",
                        px, py,
                    ),
                    _ => {},
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
                    ctx.rng.read(k);
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
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 128];
            let (k, x) = t1.split_at_mut(prime_field.len());
            let (x, y) = x.split_at_mut(prime_field.len());
            let (y, z) = y.split_at_mut(prime_field.len());
            let (z, _) = z.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
                loop {
                    ctx.rng.read(k);
                    ctx.rng.read(z);
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
    fn test_ecc_afine_point_verification_jacobian_multiplication(mut ctx: Context<'static>) {
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
                    ctx.rng.read(k);
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

                match ctx.ecc.affine_point_verification_jacobian_multiplication(curve, k, x, y) {
                    Err(Error::SizeMismatchCurve) => assert!(false, "Inputs data doesn't match the key length selected."),
                    Err(Error::PointNotOnSelectedCurve) => assert!(
                        false, "ECC failed while affine point verification + multiplication with x = {:02X?} and y = {:02X?}.",
                        x, y,
                    ),
                    _ => {},
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
        for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
            let t1 = &mut [0_u8; 64];
            let (k, y) = t1.split_at_mut(prime_field.len());
            let (y, _) = y.split_at_mut(prime_field.len());
            for _ in 0..TEST_PARAMS_VECTOR.nb_loop_inv {
                loop {
                    ctx.rng.read(k);
                    ctx.rng.read(y);
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
