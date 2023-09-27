//! Encrypt/Decrypt a message using AES

#![no_std]
#![no_main]
use aes::{
    cipher::{generic_array::GenericArray, BlockDecrypt, BlockEncrypt, KeyInit},
    Aes128 as Aes128SW,
};
use esp32c3_hal::{
    aes::{
        dma::{CipherMode, WithDmaAes},
        Aes,
        Mode,
    },
    clock::ClockControl,
    dma::DmaPriority,
    gdma::Gdma,
    peripherals::Peripherals,
    prelude::*,
    systimer::SystemTimer,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let dma = Gdma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let mut descriptors = [0u32; 8 * 3];
    let mut rx_descriptors = [0u32; 8 * 3];

    let aes = Aes::new(peripherals.AES).with_dma(dma_channel.configure(
        false,
        &mut descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    let keytext = buffer1();
    let plaintext = buffer2();
    plaintext.copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
    keytext.copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);

    // create an array with aes128 key size
    let mut keybuf = [0_u8; 16];
    keybuf[..keytext.len()].copy_from_slice(keytext);

    // create an array with aes block size
    let mut block_buf = [0_u8; 16];
    block_buf[..plaintext.len()].copy_from_slice(plaintext);

    let hw_encrypted = buffer3();
    let pre_hw_encrypt = SystemTimer::now();
    let transfer = aes
        .dma_transfer(
            plaintext,
            hw_encrypted,
            Mode::Encryption128,
            CipherMode::Ecb,
            keybuf,
        )
        .unwrap();
    let (hw_encrypted, plaintext, aes) = transfer.wait().unwrap();
    let post_hw_encrypt = SystemTimer::now();
    println!(
        "it took {} cycles for hw encrypt",
        post_hw_encrypt - pre_hw_encrypt
    );

    let keytext = buffer4();
    plaintext.copy_from_slice(hw_encrypted);
    keytext.copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);

    let mut keybuf = [0_u8; 16];
    keybuf[..keytext.len()].copy_from_slice(keytext);

    let hw_decrypted = buffer5();
    let pre_hw_decrypt = SystemTimer::now();
    let transfer = aes
        .dma_transfer(
            plaintext,
            hw_decrypted,
            Mode::Decryption128,
            CipherMode::Ecb,
            keybuf,
        )
        .unwrap();
    let (hw_decrypted, _, _) = transfer.wait().unwrap();
    let post_hw_decrypt = SystemTimer::now();
    println!(
        "it took {} cycles for hw decrypt",
        post_hw_decrypt - pre_hw_decrypt
    );

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
    let sw_decrypted = block.clone();

    assert!(eq(&sw_encrypted.into(), &hw_encrypted));
    assert!(eq(&sw_decrypted.into(), &hw_decrypted));

    println!("done");
    loop {}
}

fn buffer1() -> &'static mut [u8; 16] {
    static mut BUFFER: [u8; 16] = [0u8; 16];
    unsafe { &mut BUFFER }
}

fn buffer2() -> &'static mut [u8; 16] {
    static mut BUFFER: [u8; 16] = [0u8; 16];
    unsafe { &mut BUFFER }
}

fn buffer3() -> &'static mut [u8; 16] {
    static mut BUFFER: [u8; 16] = [0u8; 16];
    unsafe { &mut BUFFER }
}

fn buffer4() -> &'static mut [u8; 16] {
    static mut BUFFER: [u8; 16] = [0u8; 16];
    unsafe { &mut BUFFER }
}

fn buffer5() -> &'static mut [u8; 16] {
    static mut BUFFER: [u8; 16] = [0u8; 16];
    unsafe { &mut BUFFER }
}

fn eq(slice1: &[u8; 16], slice2: &[u8; 16]) -> bool {
    slice1.iter().zip(slice2.iter()).all(|(a, b)| a == b)
}
