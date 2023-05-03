#![no_std]
#![no_main]

use crypto_bigint::{
    modular::runtime_mod::{DynResidue, DynResidueParams},
    Encoding,
    Uint,
    U1024,
    U512,
};
use esp32c6_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    rsa::{
        operand_sizes,
        Rsa,
        RsaModularExponentiation,
        RsaModularMultiplication,
        RsaMultiplication,
    },
    systimer::SystemTimer,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;
use nb::block;

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

const fn compute_r(modulus: &U512) -> U512 {
    let mut d = [0_u32; U512::LIMBS * 2 + 1];
    d[d.len() - 1] = 1;
    let d = Uint::from_words(d);
    d.const_rem(&modulus.resize()).0.resize()
}

const fn compute_mprime(modulus: &U512) -> u32 {
    let m_inv = modulus.inv_mod2k(32).to_words()[0];
    (-1 * m_inv as i64 % 4294967296) as u32
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    // Disable the watchdog timers. For the ESP32-C6, this includes the Super WDT,
    // and the TIMG WDTs.
    let mut rtc = Rtc::new(peripherals.LP_CLKRST);
    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt1 = timer_group1.wdt;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut rsa = Rsa::new(peripherals.RSA, &mut system.peripheral_clock_control);

    block!(rsa.ready()).unwrap();
    mod_exp_example(&mut rsa);
    mod_multi_example(&mut rsa);
    multiplication_example(&mut rsa);
    loop {}
}

fn mod_multi_example(rsa: &mut Rsa) {
    let mut outbuf = [0_u8; U512::BYTES];
    let mut mod_multi = RsaModularMultiplication::<operand_sizes::Op512>::new(
        rsa,
        &BIGNUM_1.to_le_bytes(),
        &BIGNUM_2.to_le_bytes(),
        &BIGNUM_3.to_le_bytes(),
        compute_mprime(&BIGNUM_3),
    );
    let r = compute_r(&BIGNUM_3).to_le_bytes();
    let pre_hw_modmul = SystemTimer::now();
    mod_multi.start_modular_multiplication(&r);
    block!(mod_multi.read_results(&mut outbuf)).unwrap();
    let post_hw_modmul = SystemTimer::now();
    println!(
        "it took {} cycles for hw modular multiplication",
        post_hw_modmul - pre_hw_modmul
    );
    let residue_params = DynResidueParams::new(&BIGNUM_3);
    let residue_num1 = DynResidue::new(&BIGNUM_1, residue_params);
    let residue_num2 = DynResidue::new(&BIGNUM_2, residue_params);
    let pre_sw_exp = SystemTimer::now();
    let sw_out = residue_num1.mul(&residue_num2);
    let post_sw_exp = SystemTimer::now();
    println!(
        "it took {} cycles for sw modular multiplication",
        post_sw_exp - pre_sw_exp
    );
    assert_eq!(U512::from_le_bytes(outbuf), sw_out.retrieve());
    println!("modular multiplication done");
}

fn mod_exp_example(rsa: &mut Rsa) {
    rsa.enable_disable_constant_time_acceleration(true);
    rsa.enable_disable_search_acceleration(true);
    let mut outbuf = [0_u8; U512::BYTES];
    let mut mod_exp = RsaModularExponentiation::<operand_sizes::Op512>::new(
        rsa,
        &BIGNUM_2.to_le_bytes(),
        &BIGNUM_3.to_le_bytes(),
        compute_mprime(&BIGNUM_3),
    );
    let r = compute_r(&BIGNUM_3).to_le_bytes();
    let base = &BIGNUM_1.to_le_bytes();
    let pre_hw_exp = SystemTimer::now();
    mod_exp.start_exponentiation(&base, &r);
    block!(mod_exp.read_results(&mut outbuf)).unwrap();
    let post_hw_exp = SystemTimer::now();
    println!(
        "it took {} cycles for hw modular exponentiation",
        post_hw_exp - pre_hw_exp
    );
    let residue_params = DynResidueParams::new(&BIGNUM_3);
    let residue = DynResidue::new(&BIGNUM_1, residue_params);
    let pre_sw_exp = SystemTimer::now();
    let sw_out = residue.pow(&BIGNUM_2);
    let post_sw_exp = SystemTimer::now();
    println!(
        "it took {} cycles for sw modular exponentiation",
        post_sw_exp - pre_sw_exp
    );
    assert_eq!(U512::from_le_bytes(outbuf), sw_out.retrieve());
    println!("modular exponentiation done");
}

fn multiplication_example(rsa: &mut Rsa) {
    let mut out = [0_u8; U1024::BYTES];
    let operand_a = &BIGNUM_1.to_le_bytes();
    let operand_b = &BIGNUM_2.to_le_bytes();
    let mut rsamulti = RsaMultiplication::<operand_sizes::Op512>::new(rsa, &operand_a);
    let pre_hw_mul = SystemTimer::now();
    rsamulti.start_multiplication(&operand_b);
    block!(rsamulti.read_results(&mut out)).unwrap();
    let post_hw_mul = SystemTimer::now();
    println!(
        "it took {} cycles for hw multiplication",
        post_hw_mul - pre_hw_mul
    );
    let pre_sw_mul = SystemTimer::now();
    let sw_out = BIGNUM_1.mul_wide(&BIGNUM_2);
    let post_sw_mul = SystemTimer::now();
    println!(
        "it took {} cycles for sw multiplication",
        post_sw_mul - pre_sw_mul
    );
    assert_eq!(U1024::from_le_bytes(out), sw_out.1.concat(&sw_out.0));
    println!("multiplication done");
}
