//! Encrypt/Decrypt a message using AES

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use aes::{
    cipher::{generic_array::GenericArray, BlockDecrypt, BlockEncrypt, KeyInit},
    Aes128 as Aes128SW,
};
use esp_backtrace as _;
use esp_hal::{
    aes::{Aes, Mode},
    peripherals::Peripherals,
    prelude::*,
};
use esp_println::println;
use examples::cycles;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let mut aes = Aes::new(peripherals.AES);

    let keytext = "SUp4SeCp@sSw0rd".as_bytes();
    let plaintext = "message".as_bytes();

    // create an array with aes128 key size
    let mut keybuf = [0_u8; 16];
    keybuf[..keytext.len()].copy_from_slice(keytext);

    // create an array with aes block size
    let mut block_buf = [0_u8; 16];
    block_buf[..plaintext.len()].copy_from_slice(plaintext);

    let mut block = block_buf.clone();
    let pre_hw_encrypt = cycles();
    aes.process(&mut block, Mode::Encryption128, keybuf.into());
    let post_hw_encrypt = cycles();
    println!(
        "it took {} cycles for hw encrypt",
        post_hw_encrypt - pre_hw_encrypt
    );
    let hw_encrypted = block.clone();
    let pre_hw_decrypt = cycles();
    aes.process(&mut block, Mode::Decryption128, keybuf.into());
    let post_hw_decrypt = cycles();
    println!(
        "it took {} cycles for hw decrypt",
        post_hw_decrypt - pre_hw_decrypt
    );
    let hw_decrypted = block;

    let key = GenericArray::from(keybuf);
    let mut block = GenericArray::from(block_buf);
    let cipher = Aes128SW::new(&key);
    let pre_sw_encrypt = cycles();
    cipher.encrypt_block(&mut block);
    let post_sw_encrypt = cycles();
    println!(
        "it took {} cycles for sw encrypt",
        post_sw_encrypt - pre_sw_encrypt
    );
    let sw_encrypted = block.clone();
    let pre_sw_decrypt = cycles();
    cipher.decrypt_block(&mut block);
    let post_sw_decrypt = cycles();
    println!(
        "it took {} cycles for sw decrypt",
        post_sw_decrypt - pre_sw_decrypt
    );
    let sw_decrypted = block;

    assert!(&sw_encrypted as &[u8] == &hw_encrypted);
    assert!(&sw_decrypted as &[u8] == &hw_decrypted);

    println!("done");

    loop {}
}
