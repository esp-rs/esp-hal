//! Encrypt/Decrypt a message using AES

#![no_std]
#![no_main]
use aes::{
    cipher::{generic_array::GenericArray, BlockDecrypt, BlockEncrypt, KeyInit},
    Aes128 as Aes128SW,
};
use esp32h2_hal::{
    aes::{Aes, Aes128, Cipher, Key},
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    Rtc,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.PCR.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the watchdog timers. For the ESP32-H2, this includes the Super WDT,
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

    let mut aes = Aes::new(peripherals.AES, &mut system.peripheral_clock_control);

    let keytext = "SUp4SeCp@sSw0rd".as_bytes();
    let plaintext = "message".as_bytes();

    // create an array with aes128 key size
    let mut keybuf = [0_u8; 16];
    keybuf[..keytext.len()].copy_from_slice(keytext);

    // create an array with aes block size
    let mut block_buf = [0_u8; 16];
    block_buf[..plaintext.len()].copy_from_slice(plaintext);

    let key = Key::<Aes128>::from(&keybuf);
    let mut cipher = Cipher::new(&mut aes, &key);
    let mut block = block_buf.clone();
    let pre_hw_encrypt = SystemTimer::now();
    cipher.encrypt_block(&mut block);
    let post_hw_encrypt = SystemTimer::now();
    println!(
        "it took {} cycles for hw encrypt",
        post_hw_encrypt - pre_hw_encrypt
    );
    let hw_encrypted = block.clone();
    let pre_hw_decrypt = SystemTimer::now();
    cipher.decrypt_block(&mut block);
    let post_hw_decrypt = SystemTimer::now();
    println!(
        "it took {} cycles for hw decrypt",
        post_hw_decrypt - pre_hw_decrypt
    );
    let hw_decrypted = block;

    let key = GenericArray::from(keybuf);
    let mut block = GenericArray::from(block_buf);
    let cipher = Aes128SW::new(&key);
    let pre_sw_encrypt = SystemTimer::now();
    cipher.encrypt_block(&mut block);
    let post_sw_encrypt = SystemTimer::now();
    println!(
        "it took {} cycles for sw encrypt",
        post_sw_encrypt - pre_sw_encrypt
    );
    let sw_encrypted = block.clone();
    let pre_sw_decrypt = SystemTimer::now();
    cipher.decrypt_block(&mut block);
    let post_sw_decrypt = SystemTimer::now();
    println!(
        "it took {} cycles for sw decrypt",
        post_sw_decrypt - pre_sw_decrypt
    );
    let sw_decrypted = block;

    assert!(eq(&sw_encrypted.into(), &hw_encrypted));
    assert!(eq(&sw_decrypted.into(), &hw_decrypted));

    println!("done");

    loop {}
}
fn eq(slice1: &[u8; 16], slice2: &[u8; 16]) -> bool {
    slice1.iter().zip(slice2.iter()).all(|(a, b)| a == b)
}
