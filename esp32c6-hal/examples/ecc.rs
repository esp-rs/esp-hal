//! Demonstrates the use of the ECC peripheral and compares the speed of
//! hardware-accelerated and pure software ECC.

#![no_std]
#![no_main]

use core::ops::Mul;
use crypto_bigint::{
    modular::runtime_mod::{DynResidue, DynResidueParams},
    Encoding, U192, U256,
};
use elliptic_curve::sec1::ToEncodedPoint;
use esp32c6_hal::{
    clock::ClockControl,
    ecc::{Ecc, EllipticCurve, Error},
    peripherals::Peripherals,
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    Rng, Rtc,
};
use esp_backtrace as _;
use esp_println::{print, println};
use hex_literal::hex;

struct TestParams<'a> {
    prime_fields: &'a [&'a [u8]],
    nb_loop_mul: usize,
    // nb_loop_inv: usize,
}

const TEST_PARAMS_VECTOR: TestParams = TestParams {
    prime_fields: &[
        &hex!("fffffffffffffffffffffffffffffffeffffffffffffffff"),
        &hex!("ffffffff00000001000000000000000000000000ffffffffffffffffffffffff"),
    ],
    nb_loop_mul: 10,
    // nb_loop_inv: 20,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;

    let mut rng = Rng::new(peripherals.RNG);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();

    println!("ECC example");

    let mut hw_ecc = Ecc::new(peripherals.ECC, &mut system.peripheral_clock_control);

    println!("Beginning stress tests...");
    test_affine_point_multiplication(&mut hw_ecc, &mut rng);
    //test_finite_field_division(&mut hw_ecc, &mut rng);
    test_affine_point_verification(&mut hw_ecc, &mut rng);
    test_afine_point_verification_multiplication(&mut hw_ecc, &mut rng);
    test_jacobian_point_multiplication(&mut hw_ecc, &mut rng);
    test_jacobian_point_verification(&mut hw_ecc, &mut rng);
    test_afine_point_verification_jacobian_multiplication(&mut hw_ecc, &mut rng);
    println!("Finished stress tests!");

    loop {}
}

fn test_affine_point_multiplication(ecc: &mut Ecc, rng: &mut Rng) {
    for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
        print!("Beginning affine point multiplication tests over ");
        match prime_field.len() {
            24 => print!("secp192r1..."),
            _ => print!("secp256r1..."),
        };
        let t1 = &mut [0_u8; 96];
        let (k, x) = t1.split_at_mut(prime_field.len());
        let (x, y) = x.split_at_mut(prime_field.len());
        let (y, _) = y.split_at_mut(prime_field.len());
        let mut delta_time = 0;
        for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
            loop {
                rng.read(k).unwrap();
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

            let begin_time = SystemTimer::now();
            ecc.affine_point_multiplication(curve, k, x, y)
                .expect("Inputs data doesn't match the key length selected.");
            let end_time = SystemTimer::now();
            delta_time += end_time - begin_time;

            let t2 = &mut [0_u8; 64];

            let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
            let (sw_y, _) = sw_y.split_at_mut(prime_field.len());

            match prime_field.len() {
                24 => {
                    let sw_k =
                        p192::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
                    let q = p192::AffinePoint::GENERATOR
                        .mul(sw_k)
                        .to_affine()
                        .to_encoded_point(false);
                    sw_x.copy_from_slice(q.x().unwrap().as_slice());
                    sw_y.copy_from_slice(q.y().unwrap().as_slice());
                }
                32 => {
                    let sw_k =
                        p256::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
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
                assert_eq!(
                    a, b,
                    "ECC failed during affine point multiplication with d_a = {:02X?} ({:02X?} != {:02X?})",
                    k, a, b,
                );
            }

            for (a, b) in y.iter().zip(sw_y) {
                assert_eq!(
                    a, b,
                    "ECC failed during affine point multiplication with d_a = {:02X?} ({:02X?} != {:02X?})",
                    k, a, b,
                );
            }
        }
        println!(
            "ok (it took {} cycles in average)",
            delta_time / (TEST_PARAMS_VECTOR.nb_loop_mul as u64)
        );
    }
}

/*fn test_finite_field_division(ecc: &mut Ecc, rng: &mut Rng) {
    for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
        print!("Beginning finite field division tests over ");
        match prime_field.len() {
            24 => print!("P-192..."),
            32 => print!("P-256..."),
            _ => unimplemented!(),
        };
        let t1 = &mut [0_u8; 64];
        let (k, y) = t1.split_at_mut(prime_field.len());
        let (y, _) = y.split_at_mut(prime_field.len());
        let mut delta_time = 0;
        for _ in 0..TEST_PARAMS_VECTOR.nb_loop_inv {
            loop {
                rng.read(k).unwrap();
                rng.read(y).unwrap();
                let is_zero = k.iter().all(|&elt| elt == 0) || y.iter().all(|&elt| elt == 0);
                let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b) || y.iter().zip(prime_field).all(|(&a, &b)| a == b);
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

            let begin_time = SystemTimer::now();
            ecc
                .finite_field_division(curve, k, y)
                .expect("Inputs data doesn't match the key length selected.");
            let end_time = SystemTimer::now();
            delta_time += end_time - begin_time;

            match prime_field.len() {
                24 => {
                    let modulus = DynResidueParams::new(&U192::from_be_slice(prime_field));
                    let sw_y = DynResidue::new(&U192::from_be_slice(sw_y), modulus);
                    let sw_k = DynResidue::new(&U192::from_be_slice(sw_k), modulus);
                    let sw_inv_k = sw_k.invert().0;
                    sw_res.copy_from_slice(sw_y.mul(&sw_inv_k).retrieve().to_be_bytes().as_slice());
                },
                32 => {
                    let modulus = DynResidueParams::new(&U256::from_be_slice(prime_field));
                    let sw_y = DynResidue::new(&U256::from_be_slice(sw_y), modulus);
                    let sw_k = DynResidue::new(&U256::from_be_slice(sw_k), modulus);
                    let sw_inv_k = sw_k.invert().0;
                    sw_res.copy_from_slice(sw_y.mul(&sw_inv_k).retrieve().to_be_bytes().as_slice());
                },
                _ => unimplemented!(),
            };

            for (a, b) in y.iter().zip(sw_res) {
                assert_eq!(
                    a, b,
                    "ECC failed during finite field division with \np_y = {:02X?}\nk= {:02X?}",
                    sw_y, sw_k,
                );
            }
        }
        println!(
            "ok (it took {} cycles in average)",
            delta_time / (TEST_PARAMS_VECTOR.nb_loop_inv as u64)
        );
    }
}*/

fn test_affine_point_verification(ecc: &mut Ecc, rng: &mut Rng) {
    for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
        print!("Beginning affine point verification tests over ");
        match prime_field.len() {
            24 => print!("secp192r1..."),
            _ => print!("secp256r1..."),
        };
        let t1 = &mut [0_u8; 96];
        let (k, x) = t1.split_at_mut(prime_field.len());
        let (x, y) = x.split_at_mut(prime_field.len());
        let (y, _) = y.split_at_mut(prime_field.len());
        let mut delta_time = 0;
        for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
            loop {
                rng.read(k).unwrap();
                let is_zero = k.iter().all(|&elt| elt == 0);
                let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b);
                if is_zero == false && is_modulus == false {
                    break;
                }
            }

            let curve = match prime_field.len() {
                24 => {
                    let sw_k =
                        p192::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
                    let q = p192::AffinePoint::GENERATOR
                        .mul(sw_k)
                        .to_affine()
                        .to_encoded_point(false);
                    x.copy_from_slice(q.x().unwrap().as_slice());
                    y.copy_from_slice(q.y().unwrap().as_slice());
                    &EllipticCurve::P192
                }
                32 => {
                    let sw_k =
                        p256::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
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

            let begin_time = SystemTimer::now();
            match ecc.affine_point_verification(&curve, x, y) {
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
            let end_time = SystemTimer::now();
            delta_time += end_time - begin_time;
        }
        println!(
            "ok (it took {} cycles in average)",
            delta_time / (TEST_PARAMS_VECTOR.nb_loop_mul as u64)
        );
    }
}

fn test_afine_point_verification_multiplication(ecc: &mut Ecc, rng: &mut Rng) {
    for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
        print!("Beginning affine point verification + multiplication tests over ");
        match prime_field.len() {
            24 => print!("secp192r1..."),
            _ => print!("secp256r1..."),
        };
        let t1 = &mut [0_u8; 96];
        let (k, x) = t1.split_at_mut(prime_field.len());
        let (x, y) = x.split_at_mut(prime_field.len());
        let (y, _) = y.split_at_mut(prime_field.len());
        let mut delta_time = 0;
        for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
            loop {
                rng.read(k).unwrap();
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

            let begin_time = SystemTimer::now();
            match ecc.affine_point_verification_multiplication(curve, k, x, y) {
                Err(Error::SizeMismatchCurve) => assert!(false, "Inputs data doesn't match the key length selected."),
                Err(Error::PointNotOnSelectedCurve) => assert!(
                    false, "ECC failed while affine point verification + multiplication with x = {:02X?} and y = {:02X?}.",
                    x, y,
                ),
                _ => {},
            }
            let end_time = SystemTimer::now();
            delta_time += end_time - begin_time;

            let t2 = &mut [0_u8; 64];

            let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
            let (sw_y, _) = sw_y.split_at_mut(prime_field.len());

            match prime_field.len() {
                24 => {
                    let sw_k =
                        p192::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
                    let q = p192::AffinePoint::GENERATOR
                        .mul(sw_k)
                        .to_affine()
                        .to_encoded_point(false);
                    sw_x.copy_from_slice(q.x().unwrap().as_slice());
                    sw_y.copy_from_slice(q.y().unwrap().as_slice());
                }
                32 => {
                    let sw_k =
                        p256::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
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
                assert_eq!(
                    a, b,
                    "ECC failed during affine point verification + multiplication with d_a = {:02X?} ({:02X?} != {:02X?})",
                    k, a, b,
                );
            }

            for (a, b) in y.iter().zip(sw_y) {
                assert_eq!(
                    a, b,
                    "ECC failed during affine point verification + multiplication with d_a = {:02X?} ({:02X?} != {:02X?})",
                    k, a, b,
                );
            }
        }
        println!(
            "ok (it took {} cycles in average)",
            delta_time / (TEST_PARAMS_VECTOR.nb_loop_mul as u64)
        );
    }
}

fn test_jacobian_point_multiplication(ecc: &mut Ecc, rng: &mut Rng) {
    for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
        print!("Beginning jacobian point multiplication tests over ");
        match prime_field.len() {
            24 => print!("secp192r1..."),
            _ => print!("secp256r1..."),
        };
        let t1 = &mut [0_u8; 96];
        let (k, x) = t1.split_at_mut(prime_field.len());
        let (x, y) = x.split_at_mut(prime_field.len());
        let (y, _) = y.split_at_mut(prime_field.len());
        let mut delta_time = 0;
        for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
            let t2 = &mut [0_u8; 96];

            let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
            let (sw_y, sw_k) = sw_y.split_at_mut(prime_field.len());
            let (sw_k, _) = sw_k.split_at_mut(prime_field.len());

            loop {
                rng.read(k).unwrap();
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

            let begin_time = SystemTimer::now();
            ecc.jacobian_point_multication(curve, k, x, y)
                .expect("Inputs data doesn't match the key length selected.");
            let end_time = SystemTimer::now();
            delta_time += end_time - begin_time;

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
                    let x_affine =
                        DynResidue::new(&U192::from_be_slice(q.x().unwrap().as_slice()), modulus);
                    let y_affine =
                        DynResidue::new(&U192::from_be_slice(q.y().unwrap().as_slice()), modulus);
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
                    let x_affine =
                        DynResidue::new(&U256::from_be_slice(q.x().unwrap().as_slice()), modulus);
                    let y_affine =
                        DynResidue::new(&U256::from_be_slice(q.y().unwrap().as_slice()), modulus);
                    let z = DynResidue::new(&U256::from_be_slice(k), modulus);
                    let x_jacobian = x_affine * z * z;
                    let y_jacobian = y_affine * z * z * z;
                    sw_x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                    sw_y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                }
                _ => unimplemented!(),
            };

            for (a, b) in x.iter().zip(sw_x.iter()) {
                assert_eq!(
                    a, b,
                    "ECC failed during jacobian point multiplication.\nX = {:02X?}\nX = {:02X?}",
                    x, sw_x,
                );
            }

            for (a, b) in y.iter().zip(sw_y.iter()) {
                assert_eq!(
                    a, b,
                    "ECC failed during jacobian point multiplication.\nY = {:02X?}\nY = {:02X?}",
                    y, sw_y,
                );
            }
        }
        println!(
            "ok (it took {} cycles in average)",
            delta_time / (TEST_PARAMS_VECTOR.nb_loop_mul as u64)
        );
    }
}

fn test_jacobian_point_verification(ecc: &mut Ecc, rng: &mut Rng) {
    for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
        print!("Beginning jacobian point verification tests over ");
        match prime_field.len() {
            24 => print!("secp192r1..."),
            _ => print!("secp256r1..."),
        };
        let t1 = &mut [0_u8; 128];
        let (k, x) = t1.split_at_mut(prime_field.len());
        let (x, y) = x.split_at_mut(prime_field.len());
        let (y, z) = y.split_at_mut(prime_field.len());
        let (z, _) = z.split_at_mut(prime_field.len());
        let mut delta_time = 0;
        for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
            loop {
                rng.read(k).unwrap();
                rng.read(z).unwrap();
                let is_zero = k.iter().all(|&elt| elt == 0) || z.iter().all(|&elt| elt == 0);
                let is_modulus = k.iter().zip(prime_field).all(|(&a, &b)| a == b)
                    || z.iter().zip(prime_field).all(|(&a, &b)| a == b);
                if is_zero == false && is_modulus == false {
                    break;
                }
            }

            let curve = match prime_field.len() {
                24 => {
                    let sw_k =
                        p192::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
                    let q = p192::AffinePoint::GENERATOR
                        .mul(sw_k)
                        .to_affine()
                        .to_encoded_point(false);
                    let modulus = DynResidueParams::new(&U192::from_be_slice(prime_field));
                    let x_affine =
                        DynResidue::new(&U192::from_be_slice(q.x().unwrap().as_slice()), modulus);
                    let y_affine =
                        DynResidue::new(&U192::from_be_slice(q.y().unwrap().as_slice()), modulus);
                    let z = DynResidue::new(&U192::from_be_slice(z), modulus);
                    let x_jacobian = x_affine * z * z;
                    let y_jacobian = y_affine * z * z * z;
                    x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                    y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                    &EllipticCurve::P192
                }
                32 => {
                    let sw_k =
                        p256::Scalar::from(elliptic_curve::ScalarPrimitive::from_slice(k).unwrap());
                    let q = p256::AffinePoint::GENERATOR
                        .mul(sw_k)
                        .to_affine()
                        .to_encoded_point(false);
                    let modulus = DynResidueParams::new(&U256::from_be_slice(prime_field));
                    let x_affine =
                        DynResidue::new(&U256::from_be_slice(q.x().unwrap().as_slice()), modulus);
                    let y_affine =
                        DynResidue::new(&U256::from_be_slice(q.y().unwrap().as_slice()), modulus);
                    let z = DynResidue::new(&U256::from_be_slice(z), modulus);
                    let x_jacobian = x_affine * z * z;
                    let y_jacobian = y_affine * z * z * z;
                    x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                    y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                    &EllipticCurve::P256
                }
                _ => unimplemented!(),
            };

            let begin_time = SystemTimer::now();
            match ecc.jacobian_point_verification(&curve, x, y, z) {
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
            let end_time = SystemTimer::now();
            delta_time += end_time - begin_time;
        }
        println!(
            "ok (it took {} cycles in average)",
            delta_time / (TEST_PARAMS_VECTOR.nb_loop_mul as u64)
        );
    }
}

fn test_afine_point_verification_jacobian_multiplication(ecc: &mut Ecc, rng: &mut Rng) {
    for &prime_field in TEST_PARAMS_VECTOR.prime_fields {
        print!("Beginning affine point verification + jacobian point multiplication tests over ");
        match prime_field.len() {
            24 => print!("secp192r1..."),
            _ => print!("secp256r1..."),
        };
        let t1 = &mut [0_u8; 96];
        let (k, x) = t1.split_at_mut(prime_field.len());
        let (x, y) = x.split_at_mut(prime_field.len());
        let (y, _) = y.split_at_mut(prime_field.len());
        let mut delta_time = 0;
        for _ in 0..TEST_PARAMS_VECTOR.nb_loop_mul {
            let t2 = &mut [0_u8; 96];

            let (sw_x, sw_y) = t2.split_at_mut(prime_field.len());
            let (sw_y, sw_k) = sw_y.split_at_mut(prime_field.len());
            let (sw_k, _) = sw_k.split_at_mut(prime_field.len());

            loop {
                rng.read(k).unwrap();
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

            let begin_time = SystemTimer::now();
            match ecc.affine_point_verification_jacobian_multiplication(curve, k, x, y) {
                Err(Error::SizeMismatchCurve) => assert!(false, "Inputs data doesn't match the key length selected."),
                Err(Error::PointNotOnSelectedCurve) => assert!(
                    false, "ECC failed while affine point verification + multiplication with x = {:02X?} and y = {:02X?}.",
                    x, y,
                ),
                _ => {},
            }
            let end_time = SystemTimer::now();
            delta_time += end_time - begin_time;

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
                    let x_affine =
                        DynResidue::new(&U192::from_be_slice(q.x().unwrap().as_slice()), modulus);
                    let y_affine =
                        DynResidue::new(&U192::from_be_slice(q.y().unwrap().as_slice()), modulus);
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
                    let x_affine =
                        DynResidue::new(&U256::from_be_slice(q.x().unwrap().as_slice()), modulus);
                    let y_affine =
                        DynResidue::new(&U256::from_be_slice(q.y().unwrap().as_slice()), modulus);
                    let z = DynResidue::new(&U256::from_be_slice(k), modulus);
                    let x_jacobian = x_affine * z * z;
                    let y_jacobian = y_affine * z * z * z;
                    sw_x.copy_from_slice(x_jacobian.retrieve().to_be_bytes().as_slice());
                    sw_y.copy_from_slice(y_jacobian.retrieve().to_be_bytes().as_slice());
                }
                _ => unimplemented!(),
            };

            for (a, b) in x.iter().zip(sw_x.iter()) {
                assert_eq!(
                    a, b,
                    "ECC failed during affine point verification + jacobian point multiplication.\nX = {:02X?}\nX = {:02X?}",
                    x, sw_x,
                );
            }

            for (a, b) in y.iter().zip(sw_y.iter()) {
                assert_eq!(
                    a, b,
                    "ECC failed during affine point verification + jacobian point multiplication.\nY = {:02X?}\nY = {:02X?}",
                    y, sw_y,
                );
            }
        }
        println!(
            "ok (it took {} cycles in average)",
            delta_time / (TEST_PARAMS_VECTOR.nb_loop_mul as u64)
        );
    }
}
