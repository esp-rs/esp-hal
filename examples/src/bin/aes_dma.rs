//! Encrypt/Decrypt a message using AES

//% CHIPS: esp32c3 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use aes::{
    cipher::{generic_array::GenericArray, BlockDecrypt, BlockEncrypt, KeyInit},
    Aes128 as Aes128SW,
};
use esp_backtrace as _;
use esp_hal::{
    aes::{
        dma::{CipherMode, WithDmaAes},
        Aes,
        Mode,
    },
    dma::{Dma, DmaPriority},
    dma_buffers,
    peripherals::Peripherals,
    prelude::*,
};
use esp_println::println;
use examples::cycles;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    let (input, mut tx_descriptors, mut output, mut rx_descriptors) = dma_buffers!(16, 16);

    let mut aes = Aes::new(peripherals.AES).with_dma(dma_channel.configure(
        false,
        &mut tx_descriptors,
        &mut rx_descriptors,
        DmaPriority::Priority0,
    ));

    let keytext = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
    input.copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);

    let pre_hw_encrypt = cycles();
    let transfer = aes
        .process(
            &input,
            &mut output,
            Mode::Encryption128,
            CipherMode::Ecb,
            keytext,
        )
        .unwrap();
    transfer.wait().unwrap();
    let post_hw_encrypt = cycles();
    println!(
        "it took {} cycles for hw encrypt",
        post_hw_encrypt - pre_hw_encrypt
    );

    let mut hw_encrypted = [0u8; 16];
    (&mut hw_encrypted[..]).copy_from_slice(output);

    input.copy_from_slice(output);

    let pre_hw_decrypt = cycles();
    let transfer = aes
        .process(
            &input,
            &mut output,
            Mode::Decryption128,
            CipherMode::Ecb,
            keytext,
        )
        .unwrap();
    transfer.wait().unwrap();
    let post_hw_decrypt = cycles();
    println!(
        "it took {} cycles for hw decrypt",
        post_hw_decrypt - pre_hw_decrypt
    );

    let mut hw_decrypted = [0u8; 16];
    (&mut hw_decrypted[..]).copy_from_slice(output);

    // create an array with aes block size
    let mut block_buf = [0_u8; 16];
    block_buf[..].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);

    let key = GenericArray::from(keytext);
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
    let sw_decrypted = block.clone();

    assert!(eq(&sw_encrypted.into(), &hw_encrypted));
    assert!(eq(&sw_decrypted.into(), &hw_decrypted));

    println!("done");
    loop {}
}

fn eq(slice1: &[u8; 16], slice2: &[u8; 16]) -> bool {
    slice1.iter().zip(slice2.iter()).all(|(a, b)| a == b)
}
