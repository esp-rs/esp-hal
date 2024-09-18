//! Async RSA Test

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use crypto_bigint::{Uint, U1024, U512};
use esp_hal::{
    prelude::*,
    rsa::{
        operand_sizes::*,
        Rsa,
        RsaModularExponentiation,
        RsaModularMultiplication,
        RsaMultiplication,
    },
    Async,
};
use hil_test as _;

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

struct Context<'a> {
    rsa: Rsa<'a, Async>,
}

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

#[cfg(test)]
#[embedded_test::tests(executor = esp_hal_embassy::Executor::new())]
mod tests {
    use defmt::assert_eq;

    use super::*;

    #[init]
    fn init() -> Context<'static> {
        let peripherals = esp_hal::init(esp_hal::config::Config::default());
        let mut rsa = Rsa::new_async(peripherals.RSA);
        nb::block!(rsa.ready()).unwrap();

        Context { rsa }
    }

    #[test]
    #[timeout(5)]
    async fn modular_exponentiation(mut ctx: Context<'static>) {
        const EXPECTED_OUTPUT: [u32; U512::LIMBS] = [
            1601059419, 3994655875, 2600857657, 1530060852, 64828275, 4221878473, 2751381085,
            1938128086, 625895085, 2087010412, 2133352910, 101578249, 3798099415, 3357588690,
            2065243474, 330914193,
        ];

        #[cfg(not(feature = "esp32"))]
        {
            ctx.rsa.enable_disable_constant_time_acceleration(true);
            ctx.rsa.enable_disable_search_acceleration(true);
        }
        let mut outbuf = [0_u32; U512::LIMBS];
        let mut mod_exp = RsaModularExponentiation::<Op512, _>::new(
            &mut ctx.rsa,
            BIGNUM_2.as_words(),
            BIGNUM_3.as_words(),
            compute_mprime(&BIGNUM_3),
        );
        let r = compute_r(&BIGNUM_3);
        mod_exp
            .exponentiation(BIGNUM_1.as_words(), r.as_words(), &mut outbuf)
            .await;
        assert_eq!(EXPECTED_OUTPUT, outbuf);
    }

    #[test]
    #[timeout(5)]
    async fn test_modular_multiplication(mut ctx: Context<'static>) {
        const EXPECTED_OUTPUT: [u32; U512::LIMBS] = [
            1868256644, 833470784, 4187374062, 2684021027, 191862388, 1279046003, 1929899870,
            4209598061, 3830489207, 1317083344, 2666864448, 3701382766, 3232598924, 2904609522,
            747558855, 479377985,
        ];

        let mut outbuf = [0_u32; U512::LIMBS];
        let r = compute_r(&BIGNUM_3);
        let mut mod_multi = RsaModularMultiplication::<Op512, _>::new(
            &mut ctx.rsa,
            BIGNUM_1.as_words(),
            BIGNUM_3.as_words(),
            r.as_words(),
            compute_mprime(&BIGNUM_3),
        );
        mod_multi
            .modular_multiplication(BIGNUM_2.as_words(), &mut outbuf)
            .await;
        assert_eq!(EXPECTED_OUTPUT, outbuf);
    }

    #[test]
    #[timeout(5)]
    async fn test_multiplication(mut ctx: Context<'static>) {
        const EXPECTED_OUTPUT: [u32; U1024::LIMBS] = [
            1264702968, 3552243420, 2602501218, 498422249, 2431753435, 2307424767, 349202767,
            2269697177, 1525551459, 3623276361, 3146383138, 191420847, 4252021895, 9176459,
            301757643, 4220806186, 434407318, 3722444851, 1850128766, 928651940, 107896699,
            563405838, 1834067613, 1289630401, 3145128058, 3300293535, 3077505758, 1926648662,
            1264151247, 3626086486, 3701894076, 306518743,
        ];
        let mut outbuf = [0_u32; U1024::LIMBS];

        let operand_a = BIGNUM_1.as_words();
        let operand_b = BIGNUM_2.as_words();

        let mut rsamulti = RsaMultiplication::<Op512, _>::new(&mut ctx.rsa, operand_a);
        rsamulti.multiplication(operand_b, &mut outbuf).await;

        assert_eq!(EXPECTED_OUTPUT, outbuf)
    }
}
