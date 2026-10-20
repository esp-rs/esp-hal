use embassy_executor::SendSpawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
#[cfg(aes_supports_dma)]
use esp_hal::aes::dma::AesDmaBackend;
use esp_hal::{
    Config,
    aes::{
        Aes,
        AesBackend,
        AesContext,
        CipherState,
        Key,
        Operation,
        cipher_modes::{Cbc, Cfb8, Cfb128, Ctr, Ecb, Ofb},
    },
    clock::CpuClock,
};
use hil_test::mk_static;

const KEY: &[u8] = b"SUp4SeCp@sSw0rd";
const KEY_128: [u8; 16] = pad_to::<16>(KEY);
const KEY_256: [u8; 32] = pad_to::<32>(KEY);

const IV: [u8; 16] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];

const PLAINTEXT: &[u8] = b"Lorem Ipsum is simply dummy text of the printing and typesetting industry. Lorem Ipsum has been the industry's standard dummy text ever since the 1500s, when an unknown printer took a galley of type and scrambled it to make a type specimen book.";
const PLAINTEXT_BUF_SIZE: usize = const { PLAINTEXT.len().next_multiple_of(16) };

// DMA source data must be in RAM
fn fill_with_plaintext(buf: &mut [u8]) {
    let len = buf.len().min(PLAINTEXT.len());
    buf[..len].copy_from_slice(&PLAINTEXT[..len]);
}

const CIPHERTEXT_ECB_128: [u8; PLAINTEXT_BUF_SIZE] = [
    0x4b, 0xfc, 0xd9, 0xd5, 0xc9, 0x5e, 0x65, 0x7b, 0xea, 0x8f, 0x37, 0x32, 0x45, 0xbe, 0x77, 0x73,
    0xdf, 0x85, 0xc6, 0x84, 0xb6, 0xf1, 0xab, 0x6c, 0xd1, 0x51, 0x7a, 0xdf, 0x7b, 0x8e, 0x76, 0x89,
    0xf5, 0x9d, 0xe, 0xc5, 0xe2, 0xb, 0x66, 0x7d, 0xf5, 0xdb, 0xc8, 0x66, 0xc5, 0x7, 0xa0, 0x58,
    0x9, 0x50, 0x9e, 0xe2, 0x69, 0x1c, 0x48, 0x56, 0xd1, 0x2c, 0x5b, 0x13, 0x45, 0x58, 0xf3, 0xf6,
    0x22, 0xc8, 0xc9, 0x29, 0x48, 0xcf, 0xfa, 0x32, 0x9a, 0x15, 0x2b, 0x41, 0x1a, 0xed, 0xef, 0xb4,
    0x30, 0x84, 0x58, 0x77, 0xc4, 0x80, 0xc9, 0xbb, 0x93, 0x57, 0xe8, 0xa3, 0xe9, 0x5f, 0x26, 0x3e,
    0xe2, 0xe4, 0x66, 0x26, 0x57, 0xcd, 0x59, 0x42, 0x2e, 0x93, 0xe4, 0x55, 0xe6, 0x48, 0x46, 0x2f,
    0x6e, 0xee, 0xbf, 0x98, 0xd8, 0x2, 0xf5, 0x1a, 0xef, 0x2e, 0x62, 0x4f, 0x3f, 0x25, 0x21, 0x4f,
    0x5d, 0x46, 0x41, 0xb2, 0xc1, 0x71, 0x69, 0xfe, 0x60, 0xa7, 0x1b, 0xe3, 0xed, 0xb3, 0x4e, 0xde,
    0x70, 0x60, 0x81, 0x8d, 0xaf, 0xdd, 0xb8, 0x77, 0x53, 0x8d, 0x21, 0x8e, 0x24, 0x3e, 0xf0, 0x55,
    0xe0, 0x8, 0xa6, 0xcc, 0xd2, 0xc1, 0xb3, 0xd6, 0x33, 0x6e, 0x7a, 0xa4, 0x6e, 0x3e, 0x5a, 0xe,
    0x19, 0xca, 0xd2, 0x9, 0x12, 0xbe, 0xfb, 0x99, 0xf1, 0x46, 0xa1, 0x83, 0xb8, 0x78, 0xcf, 0xe7,
    0x1, 0xdc, 0x2, 0x45, 0x58, 0xa4, 0x6b, 0x96, 0xa2, 0x2b, 0xe, 0x2a, 0x96, 0x2e, 0x1c, 0x1a,
    0xfa, 0xe0, 0x59, 0xeb, 0xca, 0x27, 0x28, 0xd1, 0x3f, 0x2b, 0xb8, 0xf5, 0x8e, 0xd2, 0x89, 0x4b,
    0x60, 0xcb, 0x18, 0x2e, 0xa7, 0xce, 0xb6, 0xa1, 0x7a, 0x14, 0xa2, 0x2f, 0x99, 0xfe, 0xcc, 0x5e,
    0x60, 0x6c, 0xba, 0x7b, 0x9d, 0x50, 0xf6, 0xd, 0x84, 0x8c, 0x77, 0x7a, 0x50, 0x98, 0xf5, 0x86,
];
#[cfg(aes_key_length_192)]
const CIPHERTEXT_ECB_192: [u8; PLAINTEXT_BUF_SIZE] = [
    0x4c, 0x56, 0x58, 0xf5, 0x63, 0x62, 0xa1, 0xf7, 0x15, 0x60, 0xe3, 0xdb, 0x67, 0x99, 0x0, 0x6b,
    0xeb, 0xc5, 0x8f, 0xc, 0x65, 0x56, 0x43, 0x5d, 0xd7, 0x85, 0x94, 0x63, 0x0, 0x6, 0xfd, 0xd5,
    0xb4, 0x8f, 0x9b, 0x14, 0xe4, 0x9a, 0x64, 0x2, 0xdb, 0x74, 0x29, 0xc7, 0x81, 0x3b, 0x9c, 0xca,
    0xb0, 0xf1, 0xf5, 0xd4, 0xd1, 0xb8, 0xd4, 0x6e, 0xcb, 0x61, 0xaa, 0x5d, 0xc5, 0x8d, 0x57, 0xa2,
    0x6b, 0x6e, 0x4f, 0x3f, 0x6b, 0x58, 0xfb, 0x74, 0x6f, 0x4d, 0x78, 0x8f, 0x73, 0xdf, 0x66, 0x1,
    0xc9, 0x51, 0xb0, 0xd7, 0x6d, 0x6a, 0xa4, 0xe6, 0x22, 0xa5, 0x27, 0xca, 0x8a, 0xe6, 0x62, 0x1d,
    0x2, 0x59, 0xf4, 0xe7, 0xff, 0xc4, 0xb9, 0x17, 0x3b, 0x8d, 0xf9, 0x93, 0xc4, 0xd7, 0x3e, 0xe6,
    0xa2, 0xac, 0xde, 0x4b, 0x7b, 0x83, 0xef, 0xa6, 0x37, 0x98, 0xea, 0xbd, 0x1e, 0x31, 0x76, 0xb2,
    0x1d, 0x64, 0x1e, 0x3, 0x5e, 0xd2, 0xd1, 0xe8, 0x97, 0xfb, 0xcc, 0x46, 0x50, 0x61, 0xc5, 0x2,
    0x43, 0x10, 0x1d, 0x33, 0xd5, 0xd2, 0xc8, 0x8f, 0x58, 0x5f, 0xbf, 0xd8, 0x2f, 0x67, 0xca, 0xcc,
    0xa1, 0xc6, 0x83, 0x7f, 0xd6, 0xdf, 0x93, 0x7e, 0x11, 0x76, 0xf9, 0x11, 0xa0, 0xac, 0x2d, 0x26,
    0x3b, 0xa9, 0x6c, 0xbb, 0x91, 0xc9, 0x9, 0x82, 0x3d, 0x6, 0x70, 0xde, 0x62, 0x2b, 0x76, 0xc1,
    0xd5, 0x4e, 0x1b, 0x7d, 0xb2, 0xb7, 0xc8, 0xc8, 0x7b, 0x1f, 0x23, 0x64, 0x42, 0xe0, 0x52, 0x45,
    0x7c, 0xcf, 0x5b, 0xe4, 0x90, 0xc5, 0x46, 0x8e, 0xf1, 0x46, 0xf2, 0xfe, 0x6c, 0x4d, 0x76, 0xac,
    0xa4, 0x74, 0xaa, 0xf1, 0x6f, 0x70, 0xcf, 0x5f, 0x31, 0xed, 0x56, 0xfd, 0x1d, 0x2e, 0x42, 0x4f,
    0xed, 0x6b, 0x9f, 0xd3, 0x2c, 0x30, 0xcf, 0xfb, 0xa7, 0x78, 0xd0, 0xf9, 0xc7, 0xa7, 0x6c, 0x85,
];
const CIPHERTEXT_ECB_256: [u8; PLAINTEXT_BUF_SIZE] = [
    0x98, 0xf1, 0xd8, 0xfc, 0x4e, 0x3b, 0x4, 0xd3, 0xfa, 0xe5, 0xdf, 0x7a, 0x50, 0x10, 0x5f, 0xd7,
    0xfc, 0xbf, 0xd1, 0x46, 0xcf, 0xa1, 0x35, 0x2e, 0xf0, 0xb0, 0xc8, 0x3a, 0xc0, 0x3e, 0xc5, 0xc0,
    0xf8, 0x32, 0x31, 0xaa, 0xf2, 0xd6, 0x15, 0x78, 0x3d, 0xc9, 0x50, 0x77, 0x7b, 0x53, 0x39, 0x46,
    0x51, 0xb0, 0x61, 0x8b, 0xd1, 0xe3, 0x89, 0x49, 0x74, 0xf8, 0x49, 0x9c, 0xf, 0xf9, 0xc0, 0xe,
    0x82, 0xcc, 0xa6, 0xf3, 0xae, 0xbe, 0x89, 0x83, 0x7, 0xd5, 0xa2, 0xbf, 0x3e, 0xef, 0x2c, 0xc3,
    0xf6, 0x30, 0xa, 0x92, 0x95, 0x1d, 0xe4, 0xf8, 0xc3, 0x56, 0xea, 0xc0, 0xfb, 0xcb, 0x83, 0x5a,
    0xec, 0x8, 0x3c, 0x6f, 0x30, 0x66, 0x3b, 0xb2, 0xf6, 0x2a, 0x75, 0xf3, 0x72, 0x81, 0x69, 0x79,
    0x74, 0x9d, 0x4, 0x48, 0xb1, 0x60, 0x25, 0xc9, 0xb6, 0x7e, 0x57, 0xec, 0x64, 0xbf, 0x78, 0x7b,
    0xeb, 0x54, 0xea, 0x41, 0xd9, 0xa9, 0x68, 0x57, 0xc4, 0x7e, 0xdb, 0xe7, 0x7, 0xcf, 0xc8, 0xdc,
    0x97, 0x8a, 0x71, 0x6a, 0xa5, 0x72, 0x65, 0x43, 0x3e, 0xad, 0xd5, 0xb6, 0x78, 0x74, 0x53, 0xe1,
    0x2e, 0x52, 0x1b, 0x57, 0x3e, 0x1d, 0xc5, 0x75, 0x5d, 0x64, 0xc8, 0x46, 0xd3, 0xd4, 0xba, 0x5f,
    0x2a, 0x84, 0x9e, 0x9a, 0xb8, 0xd3, 0x5c, 0xfc, 0xbb, 0x65, 0xc, 0xb0, 0x90, 0x7c, 0xeb, 0x2a,
    0x40, 0xa5, 0x9e, 0x2d, 0x4f, 0xc4, 0x2e, 0x87, 0xcc, 0xa6, 0x63, 0xd8, 0x88, 0x50, 0x6, 0x4d,
    0xeb, 0xf6, 0x5a, 0x30, 0x1b, 0xa5, 0x90, 0x3b, 0x4a, 0xd2, 0x8b, 0x88, 0x10, 0xd6, 0xce, 0x4e,
    0xec, 0xd7, 0x28, 0x86, 0xee, 0x79, 0x76, 0xc2, 0xf3, 0x60, 0xad, 0x4, 0x99, 0x37, 0xf3, 0xa5,
    0xd4, 0x67, 0xa5, 0xcc, 0x80, 0x1e, 0x75, 0x85, 0x35, 0x2e, 0x39, 0xb5, 0xf2, 0x89, 0xde, 0xa1,
];

const CIPHERTEXT_CBC_128: [u8; PLAINTEXT_BUF_SIZE] = [
    0x2e, 0xd1, 0x6a, 0xc7, 0x92, 0x2, 0xb9, 0x88, 0x31, 0xd7, 0xf8, 0xce, 0x9f, 0xf2, 0x24, 0xf7,
    0x1c, 0xaa, 0x4a, 0x43, 0xd6, 0xa7, 0xfd, 0x70, 0x13, 0x93, 0x7a, 0x54, 0xf7, 0xa9, 0xb0, 0xc3,
    0x44, 0xed, 0xe9, 0x19, 0x4b, 0x67, 0x74, 0x8d, 0x1, 0xf3, 0xc8, 0x21, 0x2c, 0xeb, 0xf4, 0x81,
    0x8b, 0x42, 0x32, 0x73, 0x1b, 0x3a, 0xcb, 0x1b, 0x9d, 0x1e, 0xdd, 0x81, 0xd3, 0xd7, 0xec, 0xc5,
    0x6b, 0x83, 0xda, 0xa, 0xc5, 0xe7, 0xc5, 0x31, 0xe, 0x6d, 0x44, 0xb3, 0xd9, 0x4d, 0x8a, 0x6d,
    0x8e, 0x6e, 0x84, 0x99, 0x6b, 0xbe, 0x0, 0x65, 0xce, 0xd1, 0x14, 0x4d, 0xe6, 0xc, 0x75, 0x91,
    0x1d, 0xd9, 0x9d, 0x47, 0xf3, 0x69, 0x9a, 0x6e, 0x2a, 0x63, 0xb2, 0xce, 0x4, 0x24, 0x2f, 0xb1,
    0xd3, 0x39, 0x0, 0xa6, 0x78, 0x51, 0xf2, 0xcc, 0x73, 0x79, 0xca, 0xef, 0x8a, 0x67, 0xb5, 0x1e,
    0xb0, 0x64, 0xfc, 0xed, 0x19, 0x3e, 0x8, 0xe, 0x33, 0x10, 0xae, 0x25, 0x6b, 0x5e, 0xa3, 0x3f,
    0xa4, 0x2, 0x72, 0x5d, 0x6b, 0xce, 0xec, 0xe9, 0xf3, 0x9e, 0xeb, 0x85, 0x50, 0x4e, 0x6d, 0x76,
    0x79, 0xb6, 0x75, 0x6a, 0xec, 0x1b, 0x15, 0x6b, 0x60, 0xb6, 0x76, 0x96, 0x14, 0xf1, 0x79, 0xf2,
    0x1f, 0x96, 0x91, 0xac, 0xaf, 0xdb, 0x26, 0xea, 0xc, 0x79, 0xc2, 0x4f, 0x9c, 0x22, 0xfb, 0x16,
    0x11, 0x8, 0x49, 0x56, 0x6c, 0x1c, 0x81, 0xff, 0x64, 0xb, 0x65, 0xd4, 0x5d, 0xe0, 0x81, 0x11,
    0x74, 0x1c, 0x60, 0xad, 0xfc, 0x80, 0x7a, 0xaf, 0x23, 0x8, 0x36, 0x8c, 0x57, 0xec, 0x2d, 0xe1,
    0xbc, 0xfa, 0x56, 0xe, 0xa7, 0x1, 0xa1, 0x63, 0x1f, 0xbf, 0xf6, 0xb9, 0x46, 0xa5, 0x5, 0x29,
    0x84, 0xe1, 0x23, 0xab, 0x8f, 0x3a, 0x1f, 0x89, 0xa1, 0x8e, 0x23, 0xce, 0x74, 0x31, 0x1a, 0x36,
];
#[cfg(aes_key_length_192)]
const CIPHERTEXT_CBC_192: [u8; PLAINTEXT_BUF_SIZE] = [
    0xfe, 0x14, 0x13, 0xbf, 0x53, 0xe5, 0xbb, 0x1c, 0x66, 0x5b, 0x32, 0x2e, 0x9b, 0xb9, 0x35, 0xcb,
    0xf6, 0x6c, 0xb8, 0x26, 0x22, 0xca, 0x57, 0x9a, 0xe9, 0x98, 0x8f, 0x2f, 0xd1, 0xb1, 0xff, 0xa7,
    0xd, 0xd8, 0x92, 0x62, 0x47, 0x2d, 0x6b, 0xfc, 0x51, 0xaf, 0x84, 0x4d, 0xb4, 0x10, 0x19, 0xdf,
    0x5c, 0xb1, 0xa4, 0x5d, 0x11, 0xcf, 0x20, 0xb2, 0x2a, 0xd1, 0x43, 0x13, 0x27, 0x94, 0xd8, 0x50,
    0xfa, 0xd3, 0xb6, 0x79, 0xdb, 0xdd, 0x2f, 0x12, 0x14, 0xaa, 0x13, 0x5c, 0x9b, 0xc9, 0x7f, 0xf6,
    0xde, 0x3d, 0x63, 0x4, 0x5a, 0x0, 0x19, 0xeb, 0xf3, 0x8, 0x67, 0xda, 0x1b, 0xce, 0x28, 0xf5,
    0x1d, 0xb0, 0x6c, 0x12, 0xad, 0xfc, 0x61, 0xa9, 0xa5, 0x39, 0x7e, 0x85, 0xa9, 0xb7, 0xd5, 0xad,
    0x32, 0x1, 0x42, 0x7f, 0xd5, 0xda, 0xb0, 0x97, 0x31, 0xf8, 0xc5, 0x19, 0x44, 0xe7, 0x5, 0x79,
    0xe7, 0xd2, 0x96, 0xd0, 0x26, 0xc5, 0xf2, 0xc7, 0x23, 0x2f, 0x75, 0xde, 0x81, 0x45, 0x91, 0x5a,
    0x12, 0x7e, 0xfb, 0x5b, 0x4a, 0x3, 0xec, 0x46, 0x87, 0x34, 0x8f, 0xba, 0xf4, 0x5b, 0x29, 0x72,
    0xa7, 0x8f, 0x64, 0x2e, 0xfb, 0xb7, 0xc3, 0x9a, 0x17, 0x3, 0xae, 0xf5, 0x7e, 0xca, 0xc0, 0x1c,
    0xa3, 0x44, 0xb3, 0x84, 0x2e, 0x16, 0x77, 0xd2, 0xe5, 0x38, 0x49, 0x56, 0x9b, 0xe, 0xe8, 0xfd,
    0xfa, 0x30, 0x39, 0x0, 0xb, 0xa1, 0xaa, 0x58, 0x80, 0xfb, 0x92, 0xbb, 0xe0, 0xba, 0x5e, 0xdf,
    0xe6, 0x48, 0xf6, 0x94, 0xa8, 0xd0, 0x85, 0x63, 0xd8, 0xd3, 0x76, 0x58, 0x77, 0x2a, 0x34, 0x10,
    0x48, 0x30, 0x1f, 0x3d, 0xb0, 0x5e, 0xa, 0x3a, 0x12, 0x3d, 0x68, 0x83, 0xc9, 0x68, 0xa9, 0x44,
    0x49, 0xb2, 0x6, 0xe, 0x31, 0x3e, 0x5d, 0xe7, 0x8a, 0xea, 0xfe, 0x42, 0x95, 0xac, 0x53, 0x9a,
];
const CIPHERTEXT_CBC_256: [u8; PLAINTEXT_BUF_SIZE] = [
    0x29, 0xd4, 0xa0, 0x2e, 0x5c, 0x1d, 0xe, 0x75, 0x2, 0x2d, 0x2e, 0xdc, 0xf9, 0x56, 0x3d, 0x47,
    0xba, 0x86, 0xa2, 0xd1, 0x3b, 0xd1, 0x1d, 0xf9, 0xf2, 0x85, 0xa5, 0xa6, 0x7b, 0x8e, 0x88, 0x51,
    0x95, 0xa2, 0x3f, 0x2a, 0x61, 0x5e, 0x9b, 0x69, 0x9d, 0x9b, 0x32, 0xef, 0xbc, 0xab, 0x8a, 0x63,
    0x5a, 0x9e, 0x5c, 0xcb, 0xe6, 0xd2, 0xd0, 0xce, 0x30, 0xa8, 0x8f, 0xf8, 0x82, 0x32, 0xcf, 0xad,
    0xb2, 0xd2, 0x7d, 0xbd, 0x6c, 0xe0, 0xcf, 0xc9, 0xf3, 0xce, 0x5c, 0x6f, 0x9, 0x46, 0x34, 0x79,
    0x31, 0x83, 0xc8, 0x4c, 0x90, 0x7a, 0x6d, 0x8f, 0xb8, 0x8d, 0xa6, 0x9a, 0x30, 0x37, 0xc6, 0xe8,
    0x3f, 0xf5, 0x2f, 0xef, 0xd3, 0xf7, 0xd3, 0x77, 0xfb, 0x34, 0x44, 0x5e, 0xc, 0xfb, 0xa2, 0x9d,
    0x1a, 0x49, 0x6f, 0x10, 0xb6, 0x31, 0x76, 0xb0, 0x2a, 0xdf, 0x12, 0xaf, 0x1f, 0xdf, 0xfe, 0x57,
    0xd5, 0x24, 0x70, 0x89, 0x25, 0xa8, 0x38, 0x2b, 0xfa, 0xb9, 0x1e, 0x81, 0x74, 0xeb, 0x9b, 0x73,
    0x36, 0xa2, 0x70, 0x2b, 0x87, 0x24, 0xdb, 0xe0, 0x66, 0x2e, 0xd7, 0xea, 0xaf, 0x76, 0xe9, 0x15,
    0xa8, 0xc7, 0x78, 0xc0, 0x3b, 0xb7, 0x98, 0xbf, 0x60, 0xfe, 0x5d, 0x9f, 0x15, 0xb8, 0xb6, 0xee,
    0xb1, 0x7d, 0xf0, 0xc5, 0xcc, 0xfc, 0x6b, 0x9f, 0x94, 0xb4, 0x18, 0xa1, 0xe4, 0x8d, 0x39, 0xc8,
    0x3e, 0x92, 0x2c, 0x18, 0x26, 0xd5, 0x93, 0xc3, 0x69, 0x5c, 0x1, 0x53, 0x37, 0xa4, 0x94, 0xb5,
    0xbf, 0x87, 0xe9, 0x76, 0x70, 0x50, 0xcc, 0x8, 0x9c, 0xb9, 0x8d, 0x3a, 0x75, 0xae, 0xdc, 0x56,
    0x45, 0xd8, 0xa6, 0xd4, 0x54, 0x45, 0x75, 0x6d, 0x21, 0xcf, 0xe, 0x56, 0xd3, 0x2a, 0xe5, 0x78,
    0xc0, 0xf9, 0x75, 0x2c, 0x79, 0xcf, 0x24, 0x2e, 0x7f, 0xe0, 0xfc, 0x47, 0x16, 0x6f, 0xb8, 0x16,
];
// For the rest, we assume the other key sizes work, just to save some time on tests
const CIPHERTEXT_OFB: [u8; PLAINTEXT_BUF_SIZE] = [
    0x5, 0xcb, 0x8b, 0x29, 0x1, 0x13, 0x9e, 0xe3, 0x4f, 0x10, 0x8d, 0xdd, 0x2d, 0x58, 0x6b, 0xc7,
    0xe8, 0x8f, 0x20, 0x27, 0xbd, 0x81, 0x51, 0xa7, 0xe2, 0x5e, 0xe8, 0xa3, 0x50, 0xb1, 0xcc, 0x62,
    0x96, 0x50, 0x8a, 0x4, 0x6, 0x80, 0xd2, 0x1b, 0x9b, 0x77, 0xdc, 0x25, 0xb3, 0x1f, 0x30, 0xd0,
    0x83, 0x94, 0x5c, 0x9, 0x39, 0x10, 0x57, 0x9c, 0x3, 0xc2, 0x7e, 0x3a, 0x3a, 0x5a, 0x6c, 0xb7,
    0xc4, 0xe8, 0xc4, 0x3c, 0xfb, 0xab, 0xd8, 0xf0, 0x92, 0x79, 0xac, 0xd3, 0xd4, 0xcb, 0xab, 0xdd,
    0x40, 0x5, 0x98, 0x65, 0xc6, 0x9d, 0x0, 0x81, 0xe5, 0xd5, 0x13, 0x34, 0x80, 0xc3, 0xf6, 0xc6,
    0x4d, 0xc9, 0x52, 0x5e, 0xb2, 0xdf, 0x24, 0x4a, 0xeb, 0x5, 0x68, 0xf0, 0x63, 0xca, 0x4, 0xa0,
    0xb8, 0xc3, 0xa, 0x93, 0x74, 0x8f, 0x19, 0xec, 0x42, 0x79, 0xae, 0x3b, 0x50, 0x82, 0xc5, 0x64,
    0x70, 0x5, 0xe, 0x46, 0x36, 0xb2, 0xd1, 0x44, 0x49, 0xd0, 0xb8, 0xa1, 0x43, 0xd1, 0x65, 0x59,
    0xa9, 0xa0, 0x31, 0x68, 0x61, 0x90, 0x95, 0xda, 0xd1, 0x87, 0xeb, 0xc9, 0x8, 0x72, 0x54, 0x13,
    0x3, 0x82, 0x46, 0x98, 0xcb, 0x6e, 0x9b, 0x4c, 0xa4, 0x1d, 0xc6, 0x5b, 0xa0, 0xef, 0x84, 0xaf,
    0x3a, 0xed, 0x2e, 0x1f, 0x66, 0xad, 0xca, 0xc1, 0xf8, 0x48, 0x53, 0xda, 0xf4, 0x5c, 0xca, 0xf3,
    0xcc, 0xc1, 0x32, 0xf8, 0x76, 0x6f, 0x89, 0xa1, 0xfd, 0xfb, 0xf6, 0x21, 0xc2, 0x55, 0xf9, 0x4f,
    0x6e, 0x9e, 0x86, 0x41, 0xc6, 0xc4, 0x95, 0xc5, 0x5, 0xeb, 0xe3, 0xd4, 0xae, 0x15, 0xa3, 0x18,
    0x9c, 0xbc, 0x27, 0xd5, 0xfa, 0xe3, 0xdf, 0xed, 0xde, 0x4f, 0xa1, 0x92, 0x6d, 0x11, 0x72, 0xd6,
    0x22, 0x25, 0xe8, 0x83, 0x11, 0x55, 0x1d, 0x94, 0xa1, 0x8, 0x2c, 0x78, 0xc7, 0xf2, 0xa7, 0xae,
];
const CIPHERTEXT_CTR: [u8; PLAINTEXT_BUF_SIZE] = [
    0x5, 0xcb, 0x8b, 0x29, 0x1, 0x13, 0x9e, 0xe3, 0x4f, 0x10, 0x8d, 0xdd, 0x2d, 0x58, 0x6b, 0xc7,
    0xc4, 0x8f, 0x9a, 0x2c, 0xd8, 0x22, 0x27, 0xba, 0xda, 0x92, 0xc0, 0x36, 0xe3, 0x5, 0xb, 0x52,
    0xdb, 0xe2, 0xe5, 0x42, 0x34, 0x66, 0xfc, 0x71, 0x54, 0xfe, 0x19, 0x1e, 0xad, 0x8e, 0x12, 0x3f,
    0x72, 0x44, 0xc2, 0xdd, 0xd7, 0xb3, 0x9f, 0x6e, 0x3d, 0xff, 0xf4, 0x81, 0x49, 0x33, 0x24, 0x45,
    0x9d, 0x1b, 0xd1, 0xd5, 0x97, 0x9, 0xb1, 0x81, 0x4f, 0xec, 0x7c, 0xdb, 0x75, 0xc, 0x7c, 0x2b,
    0x27, 0x40, 0xf9, 0x9e, 0x97, 0x30, 0x53, 0x9f, 0x26, 0x75, 0xb4, 0xd1, 0x1f, 0xea, 0xdd, 0xb,
    0xa5, 0x7c, 0xc0, 0x36, 0x58, 0x47, 0x37, 0xf2, 0x41, 0x47, 0x6d, 0x44, 0x12, 0xd8, 0x10, 0x6e,
    0x86, 0xa5, 0x15, 0x1, 0x39, 0x21, 0x75, 0xd9, 0x4f, 0x2c, 0x64, 0x60, 0xad, 0xe9, 0x26, 0xde,
    0x3e, 0x3a, 0x15, 0x12, 0x19, 0x4d, 0xb4, 0x9f, 0xe2, 0x29, 0xf9, 0x23, 0xbb, 0xb0, 0xb3, 0xab,
    0x30, 0x9b, 0xe, 0xd, 0x1b, 0xe0, 0xd2, 0x2b, 0x19, 0x73, 0x13, 0xfa, 0xf1, 0x6b, 0x64, 0x9,
    0xe6, 0x7e, 0x2c, 0x0, 0x42, 0x65, 0x69, 0x25, 0x57, 0xfd, 0x8c, 0x5, 0x38, 0x4d, 0xa5, 0xcd,
    0xeb, 0xa6, 0xaf, 0x5c, 0x52, 0xbc, 0x89, 0x34, 0xe6, 0xc2, 0x1e, 0xe3, 0x1d, 0x8d, 0x75, 0x98,
    0x54, 0x5a, 0xad, 0xb0, 0x61, 0xf9, 0x96, 0x5, 0xf8, 0xde, 0x4f, 0xbe, 0xad, 0xd8, 0x9b, 0xb2,
    0xfa, 0x46, 0xe8, 0xeb, 0xf4, 0x45, 0x71, 0x39, 0xaa, 0x4d, 0x2a, 0xd2, 0xc7, 0x6e, 0x67, 0x7a,
    0x21, 0x9d, 0x58, 0x1e, 0x8b, 0xec, 0x4, 0xac, 0x75, 0x47, 0x44, 0x17, 0x2b, 0x26, 0x14, 0xca,
    0x7b, 0xe0, 0xe6, 0xa6, 0x27, 0x90, 0xca, 0x30, 0x41, 0xd2, 0xe3, 0x80, 0xe8, 0x43, 0x7, 0x86,
];
const CIPHERTEXT_CFB8: [u8; PLAINTEXT_BUF_SIZE] = [
    0x5, 0x31, 0xde, 0x6a, 0xd7, 0x13, 0x10, 0x22, 0xcb, 0x16, 0x53, 0x27, 0x9b, 0xc7, 0x2d, 0xd2,
    0x47, 0x89, 0xa4, 0x72, 0xb3, 0x67, 0x68, 0xb1, 0xad, 0xd8, 0x3f, 0x7d, 0xb3, 0xee, 0x19, 0xf6,
    0x1, 0x79, 0x3d, 0x35, 0x19, 0x68, 0xc6, 0xf1, 0x53, 0xf2, 0x62, 0x29, 0xdf, 0xd2, 0xd8, 0x7f,
    0x1, 0xaf, 0x76, 0x38, 0x27, 0x7a, 0xec, 0xd6, 0x30, 0x6d, 0x1, 0xcd, 0xcb, 0xd4, 0xbd, 0x39,
    0x3a, 0x75, 0x46, 0x6c, 0xa4, 0x28, 0xf2, 0x90, 0x3c, 0x6, 0xe2, 0x2f, 0x7e, 0x47, 0xb8, 0x97,
    0xcc, 0xd3, 0x69, 0xdd, 0x58, 0xff, 0x6f, 0x8e, 0x46, 0xf0, 0x9a, 0xb8, 0xa7, 0x50, 0x5b, 0xc9,
    0xe0, 0xac, 0x25, 0xb5, 0x70, 0x59, 0x33, 0x22, 0x97, 0x37, 0xe3, 0x74, 0x5d, 0x3a, 0x7c, 0x5f,
    0x27, 0x81, 0xba, 0xee, 0xd1, 0x22, 0x2c, 0x1b, 0xa5, 0xf3, 0xd2, 0xc7, 0x43, 0xf2, 0xd, 0xa4,
    0x4c, 0xd5, 0xd, 0xf3, 0xe5, 0x81, 0x47, 0x62, 0xa6, 0x53, 0x9e, 0x95, 0xb9, 0xd5, 0x6f, 0xd7,
    0x4c, 0x6, 0xef, 0x12, 0xe9, 0x90, 0x39, 0xaa, 0x52, 0xab, 0xc8, 0x21, 0xf8, 0x5e, 0x3b, 0x6a,
    0x4, 0x63, 0x5c, 0x94, 0x8a, 0xb3, 0xe2, 0x85, 0xf3, 0xd4, 0x35, 0xb2, 0xb4, 0x3c, 0x50, 0x89,
    0xe5, 0xc3, 0x2, 0x1a, 0x7f, 0xc7, 0x34, 0x65, 0x62, 0xc6, 0xb9, 0xe9, 0x4a, 0xa7, 0xf3, 0xd1,
    0x91, 0x57, 0x8, 0x2f, 0xd7, 0x8b, 0x4a, 0xd9, 0xa, 0xd9, 0x14, 0xfe, 0xfc, 0xab, 0x44, 0x73,
    0xb2, 0x95, 0xb, 0x7, 0x69, 0xc4, 0xdb, 0x4d, 0xcb, 0x35, 0x98, 0xd8, 0x10, 0x6, 0xcd, 0x67,
    0x3, 0xa3, 0x1b, 0x24, 0xef, 0xab, 0x11, 0xbd, 0x42, 0x94, 0x71, 0xd, 0xb8, 0xb0, 0xed, 0x6c,
    0xd8, 0x39, 0xec, 0x8c, 0x3f, 0x29, 0xbc, 0x19, 0x81, 0x3b, 0x89, 0x25, 0x3c, 0x36, 0x59, 0x7a,
];
const CIPHERTEXT_CFB128: [u8; PLAINTEXT_BUF_SIZE] = [
    0x5, 0xcb, 0x8b, 0x29, 0x1, 0x13, 0x9e, 0xe3, 0x4f, 0x10, 0x8d, 0xdd, 0x2d, 0x58, 0x6b, 0xc7,
    0xfc, 0x87, 0x8a, 0xb4, 0x87, 0xa2, 0x4, 0x40, 0xf5, 0x4e, 0x70, 0x53, 0x10, 0x89, 0x79, 0xab,
    0x83, 0x67, 0x12, 0xbb, 0x7d, 0x12, 0xd0, 0xf4, 0xcc, 0x89, 0x24, 0x8e, 0x51, 0x5a, 0x35, 0x17,
    0x51, 0xe3, 0x71, 0xd0, 0xcf, 0x67, 0xd2, 0x81, 0x8f, 0x2e, 0x69, 0x70, 0xe2, 0xc8, 0x8c, 0x52,
    0x62, 0x82, 0xa6, 0xea, 0x35, 0xbc, 0xe2, 0x50, 0xc7, 0x99, 0xe2, 0xb9, 0x66, 0x38, 0x76, 0x7c,
    0xcd, 0x95, 0xe8, 0x1c, 0xd2, 0x50, 0x98, 0x2c, 0x32, 0xe8, 0x1a, 0x88, 0xd1, 0xd2, 0xd0, 0xb4,
    0xa4, 0xa8, 0x9a, 0x86, 0x6a, 0x75, 0x9f, 0x20, 0xb6, 0xac, 0x24, 0x53, 0x41, 0x2d, 0xa4, 0x1a,
    0xa9, 0xd6, 0xde, 0x25, 0x56, 0x6f, 0xfb, 0xeb, 0xae, 0x27, 0xeb, 0xe0, 0x25, 0xd2, 0x27, 0xe6,
    0x20, 0xc1, 0x8a, 0x50, 0x52, 0x5, 0x32, 0xc8, 0x16, 0x75, 0x62, 0xec, 0xe3, 0xa, 0x5, 0xac,
    0x4e, 0x2c, 0x53, 0xfe, 0x23, 0x34, 0x33, 0xe2, 0x30, 0xcf, 0xd, 0x5d, 0x1e, 0x73, 0x35, 0xa7,
    0xb1, 0xef, 0x58, 0xea, 0x99, 0x3b, 0xe8, 0x70, 0x6a, 0x4d, 0xb2, 0xb5, 0x5e, 0x13, 0x39, 0x91,
    0x9, 0xa7, 0x60, 0x65, 0x9b, 0x58, 0x9b, 0xa5, 0x35, 0xcc, 0x3e, 0xc1, 0x12, 0x15, 0xc6, 0x6c,
    0xf7, 0xba, 0x1f, 0x9a, 0x75, 0x77, 0xe5, 0x38, 0x20, 0xe5, 0x4b, 0x60, 0x48, 0x58, 0x23, 0x50,
    0x86, 0xc9, 0x70, 0x3f, 0xe9, 0x2f, 0xeb, 0x73, 0xa4, 0x43, 0x77, 0xcf, 0xde, 0xee, 0x77, 0x2,
    0x27, 0xd9, 0xc5, 0xbb, 0xac, 0x7c, 0xdc, 0x1b, 0xfa, 0xb, 0x72, 0xd2, 0xd7, 0x1e, 0xeb, 0x68,
    0xa5, 0x1b, 0xae, 0xdc, 0x78, 0x4f, 0xcf, 0xcf, 0x31, 0xee, 0xb6, 0xc5, 0x7c, 0x2d, 0x81, 0x37,
];

#[cfg(aes_supports_dma)]
extern crate alloc;

const fn pad_to<const K: usize>(input: &[u8]) -> [u8; K] {
    let mut out = [0; K];

    let in_bytes = input.len();
    assert!(in_bytes <= K);

    let mut i = 0;
    while i < in_bytes {
        out[i] = input[i];
        i += 1;
    }

    out
}
fn aes_ll_roundtrip<const K: usize>(aes: &mut Aes<'_>, plaintext: [u8; 16], ciphertext: [u8; 16])
where
    Key: From<[u8; K]>,
{
    let mut block_buf = plaintext;
    aes.encrypt(&mut block_buf, pad_to::<K>(KEY));
    hil_test::assert_eq!(&block_buf[..ciphertext.len()], ciphertext);

    let mut block_buf = ciphertext;
    aes.decrypt(&mut block_buf, pad_to::<K>(KEY));
    hil_test::assert_eq!(&block_buf[..plaintext.len()], plaintext);
}

fn aes_roundtrip<const K: usize>(
    tag: &'static str,
    block_ctx: impl Into<CipherState>,
    plaintext: &[u8],
    ciphertext: &[u8],
    buffer: &mut [u8],
) where
    Key: From<[u8; K]>,
{
    let block_ctx = block_ctx.into();
    defmt::info!("{}: buffer @ {:x}", tag, buffer.as_ptr() as usize);
    let mut ecb_encrypt = AesContext::new(block_ctx.clone(), Operation::Encrypt, pad_to::<K>(KEY));
    let handle = ecb_encrypt.process(plaintext, buffer).unwrap();
    handle.wait_blocking();
    hil_test::assert_eq!(
        buffer,
        ciphertext,
        "Failed to encrypt with {}, K={}",
        tag,
        K
    );

    let mut ecb_decrypt = AesContext::new(block_ctx, Operation::Decrypt, pad_to::<K>(KEY));
    let handle = ecb_decrypt.process_in_place(buffer).unwrap();
    handle.wait_blocking();
    hil_test::assert_eq!(buffer, plaintext, "Failed to encrypt with {}, K={}", tag, K);
}

fn run_cipher_tests(buffer: &mut [u8]) {
    let mut plaintext = [0; PLAINTEXT_BUF_SIZE];
    fill_with_plaintext(&mut plaintext);

    // Let's use ECB as a test case for short unaligned DMA transfers.
    let short_len = 16;
    aes_roundtrip::<16>(
        "Short ECB",
        Ecb,
        &plaintext[0..short_len],
        &CIPHERTEXT_ECB_128[0..short_len],
        &mut buffer[0..short_len],
    );

    aes_roundtrip::<16>("ECB", Ecb, &plaintext, &CIPHERTEXT_ECB_128, buffer);
    #[cfg(aes_key_length_192)]
    aes_roundtrip::<24>("ECB", Ecb, &plaintext, &CIPHERTEXT_ECB_192, buffer);
    aes_roundtrip::<32>("ECB", Ecb, &plaintext, &CIPHERTEXT_ECB_256, buffer);

    aes_roundtrip::<16>("CBC", Cbc::new(IV), &plaintext, &CIPHERTEXT_CBC_128, buffer);
    #[cfg(aes_key_length_192)]
    aes_roundtrip::<24>("CBC", Cbc::new(IV), &plaintext, &CIPHERTEXT_CBC_192, buffer);
    aes_roundtrip::<32>("CBC", Cbc::new(IV), &plaintext, &CIPHERTEXT_CBC_256, buffer);

    aes_roundtrip::<16>("OFB", Ofb::new(IV), &plaintext, &CIPHERTEXT_OFB, buffer);
    aes_roundtrip::<16>("CTR", Ctr::new(IV), &plaintext, &CIPHERTEXT_CTR, buffer);
    aes_roundtrip::<16>("CFB-8", Cfb8::new(IV), &plaintext, &CIPHERTEXT_CFB8, buffer);
    aes_roundtrip::<16>(
        "CFB-128",
        Cfb128::new(IV),
        &plaintext,
        &CIPHERTEXT_CFB128,
        buffer,
    );
}

#[cfg(aes_supports_dma)]
fn run_unaligned_dma_tests<const MAX_SHIFT: usize>(memory: &mut [u8]) {
    let zeros = [0; MAX_SHIFT];

    // Different alignments in internal memory
    for shift in 0..MAX_SHIFT {
        memory.fill(0);
        let buffer = &mut memory[shift..][..PLAINTEXT_BUF_SIZE];
        run_cipher_tests(buffer);
        hil_test::assert_eq!(
            memory[..shift],
            zeros[..shift],
            "Out of bounds write with shift {}",
            shift
        );
    }
}

#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())] // defmt slows the tests down a bit
mod tests {
    use super::*;

    #[test]
    fn test_aes_block_operation() {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));
        let mut aes = Aes::new(p.AES);

        aes_ll_roundtrip::<16>(
            &mut aes,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_128[0..16].try_into().unwrap(),
        );

        #[cfg(aes_key_length_192)]
        aes_ll_roundtrip::<24>(
            &mut aes,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_192[0..16].try_into().unwrap(),
        );

        aes_ll_roundtrip::<32>(
            &mut aes,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_256[0..16].try_into().unwrap(),
        );
    }

    /// The crypto accelerators and the MCPWM peripherals share a clock, and its reference count.
    #[test]
    #[cfg(soc_has_clock_node_crypto_pwm_clk)]
    fn test_crypto_clock_shared_with_mcpwm() {
        use esp_hal::{
            clock::ll::{ClockTree, CryptoPwmClkConfig, McpwmInstance, crypto_pwm_clk_frequency},
            mcpwm::{McPwm, PeripheralClockConfig},
            time::Rate,
        };

        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));
        let mut aes = Aes::new(p.AES);

        let clock_config = PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
        let mcpwm = McPwm::new(p.MCPWM0, clock_config);

        hil_test::assert!(
            ClockTree::with(|clocks| clocks.crypto_pwm_clk()) == Some(CryptoPwmClkConfig::Pll)
        );
        hil_test::assert_eq!(
            McpwmInstance::Mcpwm0.function_clock_frequency(),
            crypto_pwm_clk_frequency()
        );
        aes_ll_roundtrip::<16>(
            &mut aes,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_128[0..16].try_into().unwrap(),
        );

        core::mem::drop(mcpwm);

        aes_ll_roundtrip::<16>(
            &mut aes,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_128[0..16].try_into().unwrap(),
        );
    }

    #[test]
    #[cfg(aes_supports_dma)]
    fn test_aes_dma_ecb() {
        use esp_hal::{
            aes::dma::AesDma,
            dma::{DmaRxBuf, DmaTxBuf},
            dma_rx_buffer,
            dma_tx_buffer,
        };

        struct Context<'a> {
            aes: AesDma<'a>,
            output: DmaRxBuf,
            input: DmaTxBuf,
        }

        fn test_aes_ecb<const K: usize>(
            mut ctx: Context<'_>,
            plaintext: [u8; 16],
            ciphertext: [u8; 16],
        ) -> Context<'_>
        where
            Key: From<[u8; K]>,
        {
            use esp_hal::aes::dma::DmaCipherState;

            // Encrypt
            ctx.input.as_mut_slice().copy_from_slice(&plaintext);
            let transfer = ctx
                .aes
                .process(
                    1,
                    ctx.output,
                    ctx.input,
                    Operation::Encrypt,
                    &DmaCipherState::from(Ecb),
                    pad_to::<K>(KEY),
                )
                .map_err(|e| e.0)
                .unwrap();
            let (aes, output, mut input) = transfer.wait();
            hil_test::assert_eq!(output.as_slice(), ciphertext);

            // Decrypt
            input.as_mut_slice().copy_from_slice(&ciphertext);
            let transfer = aes
                .process(
                    1,
                    output,
                    input,
                    Operation::Decrypt,
                    &DmaCipherState::from(Ecb),
                    pad_to::<K>(KEY),
                )
                .map_err(|e| e.0)
                .unwrap();
            let (aes, output, input) = transfer.wait();
            hil_test::assert_eq!(output.as_slice(), plaintext);

            Context { aes, output, input }
        }

        const DMA_BUFFER_SIZE: usize = 16;

        let input = dma_tx_buffer!(DMA_BUFFER_SIZE).unwrap();
        let output = dma_rx_buffer!(DMA_BUFFER_SIZE).unwrap();

        let peripherals = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        let dma_channel = cfg_select! {
            aes_dma_engine = "CRYPTO_DMA" => peripherals.DMA_CRYPTO,
            aes_dma_engine = "AXI_GDMA" => peripherals.DMA_AXI_CH0,
            _ => peripherals.DMA_CH0,
        };

        let aes = Aes::new(peripherals.AES).with_dma(dma_channel);
        let ctx = Context { aes, output, input };

        let ctx = test_aes_ecb::<16>(
            ctx,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_128[0..16].try_into().unwrap(),
        );
        #[cfg(aes_key_length_192)]
        let ctx = test_aes_ecb::<24>(
            ctx,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_192[0..16].try_into().unwrap(),
        );
        let _ = test_aes_ecb::<32>(
            ctx,
            PLAINTEXT[0..16].try_into().unwrap(),
            CIPHERTEXT_ECB_256[0..16].try_into().unwrap(),
        );
    }
}

#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod work_queue_tests {
    use super::*;

    struct Context {
        aes: AesBackend<'static>,
    }

    #[init]
    fn init() -> Context {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));
        Context {
            aes: AesBackend::new(p.AES),
        }
    }

    #[test]
    fn test_aes_work_queue_cpu(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let mut buffer = [0; PLAINTEXT_BUF_SIZE];
        run_cipher_tests(&mut buffer);
    }

    #[test]
    fn test_aes_work_queue_work_posted_before_queue_started(mut ctx: Context) {
        let mut output = [0; PLAINTEXT_BUF_SIZE];

        let mut plaintext = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut plaintext);

        let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        let handle = ecb_encrypt.process(&plaintext, &mut output).unwrap();

        let _backend = ctx.aes.start();

        handle.wait_blocking();

        hil_test::assert_eq!(output, CIPHERTEXT_ECB_128);
    }

    #[test]
    async fn test_aes_work_queue_work_posted_before_queue_started_async(mut ctx: Context) {
        #[embassy_executor::task]
        async fn aes_task(signal: &'static Signal<CriticalSectionRawMutex, ()>) {
            let mut output = [0; PLAINTEXT_BUF_SIZE];
            let mut plaintext = [0; PLAINTEXT_BUF_SIZE];
            fill_with_plaintext(&mut plaintext);

            let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
            let mut handle = ecb_encrypt.process(&plaintext, &mut output).unwrap();

            // Backend can start now
            signal.signal(());

            handle.wait().await;
            core::mem::drop(handle);

            hil_test::assert_eq!(output, CIPHERTEXT_ECB_128);

            // Test can end now
            signal.signal(());
        }

        let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());

        // Start task before we'd start the AES operation
        let spawner = SendSpawner::for_current_executor().await;
        spawner.spawn(aes_task(signal).unwrap());

        signal.wait().await;

        let _backend = ctx.aes.start();

        signal.wait().await;
    }

    #[test]
    fn test_aes_work_queue_in_place(mut ctx: Context) {
        let mut buffer = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut buffer);

        let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        let handle = ecb_encrypt.process_in_place(&mut buffer).unwrap();

        let _backend = ctx.aes.start();

        handle.wait_blocking();

        hil_test::assert_eq!(buffer, CIPHERTEXT_ECB_128);
    }

    #[test]
    fn test_aes_cancelling_work(mut ctx: Context) {
        // In this test, we post two work items, and cancel the first one before starting the
        // backend. We will assert that, when the second item has finished correctly, the first did
        // not modify its output buffer.
        // Note that this result is not guaranteed. The cancellation can come later than the work
        // item has finished processing. We can only reliably test it because we start the backend
        // after cancelling the operation.
        let mut buffer1 = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut buffer1);
        let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        let handle1 = ecb_encrypt.process_in_place(&mut buffer1).unwrap();

        let mut buffer2 = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut buffer2);
        let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, KEY_256);
        let handle2 = ecb_encrypt.process_in_place(&mut buffer2).unwrap();

        let mut buffer3 = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut buffer3);
        let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        let handle3 = ecb_encrypt.process_in_place(&mut buffer3).unwrap();

        core::mem::drop(handle1);

        let _backend = ctx.aes.start();

        handle3.wait_blocking();
        handle2.wait_blocking();

        hil_test::assert_eq!(&buffer1[..PLAINTEXT.len()], PLAINTEXT);
        hil_test::assert_eq!(buffer2, CIPHERTEXT_ECB_256);
        hil_test::assert_eq!(buffer3, CIPHERTEXT_ECB_128);
    }
}

#[cfg(aes_supports_dma)]
#[embedded_test::tests(default_timeout = 10)]
mod work_queue_dma_tests {
    use allocator_api2::vec::Vec;

    use super::*;

    struct Context {
        aes: AesDmaBackend<'static>,
    }

    #[init]
    fn init() -> Context {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        esp_alloc::heap_allocator!(size: 32 * 1024);

        #[cfg(all(soc_has_psram, dma_can_access_psram))]
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);

        let dma = cfg_select! {
            aes_dma_engine = "CRYPTO_DMA" => p.DMA_CRYPTO,
            aes_dma_engine = "AXI_GDMA" => p.DMA_AXI_CH0,
            _ => p.DMA_CH0,
        };

        Context {
            aes: AesDmaBackend::new(p.AES, dma),
        }
    }

    #[test]
    fn test_aes_dma_work_queue(mut ctx: Context) {
        let _backend = ctx.aes.start();

        const MAX_SHIFT: usize = 15;

        let mut internal_memory =
            Vec::with_capacity_in(PLAINTEXT_BUF_SIZE + MAX_SHIFT, esp_alloc::InternalMemory);
        internal_memory.resize(PLAINTEXT_BUF_SIZE + MAX_SHIFT, 0);

        // Different alignments in internal memory
        run_unaligned_dma_tests::<MAX_SHIFT>(&mut internal_memory);
    }

    #[test]
    #[cfg(all(soc_has_psram, dma_can_access_psram))]
    fn test_aes_dma_work_queue_psram(mut ctx: Context) {
        let _backend = ctx.aes.start();

        const MAX_SHIFT: usize = 15;

        let mut external_memory =
            Vec::with_capacity_in(PLAINTEXT_BUF_SIZE + MAX_SHIFT, esp_alloc::ExternalMemory);
        external_memory.resize(PLAINTEXT_BUF_SIZE + MAX_SHIFT, 0);

        // Different alignments in external memory
        run_unaligned_dma_tests::<MAX_SHIFT>(&mut external_memory);
    }

    // 128 bytes is the longest data cache line in front of memory that the DMA writes.
    const SHARED_LINE: usize = 128;
    // The output is long, so that its last line stays unwritten while the CPU reads the bytes
    // beside it.
    const SHARED_LINE_OUTPUT_LEN: usize = 4_000;
    const SHARED_LINE_MEMORY_LEN: usize = SHARED_LINE + 16 + SHARED_LINE_OUTPUT_LEN + 16;

    /// Encrypts into an output that shares its first and last cache lines with other bytes. The
    /// CPU reads and writes those bytes while the DMA runs. The output must hold the DMA's data,
    /// and the other bytes must keep the CPU's writes.
    ///
    /// The output starts 16 bytes into a cache line, which meets the DMA's block alignment, and
    /// ends partway into another line.
    fn run_output_shares_cache_lines(memory: &mut [u8]) {
        let addr = memory.as_ptr().addr();
        let line_start = addr.next_multiple_of(SHARED_LINE) - addr;
        let (before, rest) = memory[line_start..].split_at_mut(16);
        let (output, rest) = rest.split_at_mut(SHARED_LINE_OUTPUT_LEN);
        let after = &mut rest[..16];

        // ECB encrypts block by block, so a repeated plaintext has a repeated ciphertext.
        let mut block = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut block);
        let mut plaintext =
            Vec::with_capacity_in(SHARED_LINE_OUTPUT_LEN, esp_alloc::InternalMemory);
        plaintext.extend((0..SHARED_LINE_OUTPUT_LEN).map(|i| block[i % PLAINTEXT_BUF_SIZE]));
        let mut expected = Vec::with_capacity_in(SHARED_LINE_OUTPUT_LEN, esp_alloc::InternalMemory);
        expected.extend(
            (0..SHARED_LINE_OUTPUT_LEN).map(|i| CIPHERTEXT_ECB_128[i % PLAINTEXT_BUF_SIZE]),
        );

        let mut ecb_encrypt = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        let mut handle = ecb_encrypt.process(&plaintext, output).unwrap();

        let mut last = 0xa5;
        for byte in before.iter_mut().chain(after.iter_mut()) {
            unsafe { core::ptr::write_volatile(byte, last) };
        }
        let mut count: u32 = 0;
        while !handle.poll() {
            count = count.wrapping_add(1);
            last = count as u8;
            for byte in before.iter_mut().chain(after.iter_mut()) {
                // The read pulls the shared line into the cache, and the write dirties it.
                let _ = unsafe { core::ptr::read_volatile(byte) };
                unsafe { core::ptr::write_volatile(byte, last) };
            }
        }
        core::mem::drop(handle);

        let expected = &expected[..];
        hil_test::assert_eq!(&output[..], expected, "output, {} polls", count);
        hil_test::assert!(
            before.iter().all(|&b| b == last),
            "bytes before the output, {} polls",
            count
        );
        hil_test::assert!(
            after.iter().all(|&b| b == last),
            "bytes after the output, {} polls",
            count
        );
    }

    #[test]
    fn test_aes_dma_output_shares_cache_lines(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let mut internal_memory =
            Vec::with_capacity_in(SHARED_LINE_MEMORY_LEN, esp_alloc::InternalMemory);
        internal_memory.resize(SHARED_LINE_MEMORY_LEN, 0);

        run_output_shares_cache_lines(&mut internal_memory);
    }

    #[test]
    #[cfg(all(soc_has_psram, dma_can_access_psram, not(esp32s2)))]
    fn test_aes_dma_psram_output_shares_cache_lines(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let mut external_memory =
            Vec::with_capacity_in(SHARED_LINE_MEMORY_LEN, esp_alloc::ExternalMemory);
        external_memory.resize(SHARED_LINE_MEMORY_LEN, 0);

        run_output_shares_cache_lines(&mut external_memory);
    }
}

#[cfg(aes_dma_mode_gcm)]
mod gcm {
    use aes_gcm::{
        AeadInOut,
        AesGcm,
        KeyInit,
        aead::consts::U12,
        aes::{Aes128, Aes256},
    };
    use allocator_api2::vec::Vec;
    use esp_hal::{
        aes::{
            Error,
            Key,
            dma::{AesGcmContext, GcmError},
        },
        rng::Rng,
    };
    use hex_literal::hex;

    pub const MAX_TEXT: usize = 20_400;
    pub const MAX_AAD: usize = 4_080;

    /// Adds the GCM tests' heap. The test modules share one region, because each test boots on
    /// its own and the ESP32-S2 cannot fit two.
    pub fn gcm_heap() {
        esp_alloc::heap_allocator!(size: 96 * 1024);
    }

    /// One NIST CAVP vector from `gcmtestvectors.zip` (files dated 2012-08-31), with a 96-bit IV
    /// and a 128-bit tag. `plaintext` is `None` for a vector the file marks FAIL.
    pub struct Vector {
        pub key: &'static [u8],
        pub nonce: [u8; 12],
        pub aad: &'static [u8],
        pub plaintext: Option<&'static [u8]>,
        pub ciphertext: &'static [u8],
        pub tag: [u8; 16],
    }

    // gcmEncryptExtIV128.rsp, [PTlen = 104] [AADlen = 160], Count = 0
    // gcmEncryptExtIV128.rsp, [PTlen = 408] [AADlen = 720], Count = 0
    pub const ENCRYPT_128: [Vector; 2] = [
        Vector {
            key: &hex!("fe0121f42e599f88ff02a985403e19bb"),
            nonce: hex!("3bb9eb7724cbe1943d43de21"),
            aad: &hex!("2662d895035b6519f3510eae0faa3900ad23cfdf"),
            plaintext: Some(&hex!("fd331ca8646091c29f21e5f0a1")),
            ciphertext: &hex!("59fe29b07b0de8d869efbbd9b4"),
            tag: hex!("d24c3e9c1c73c0af1097e26061c857de"),
        },
        Vector {
            key: &hex!("2c1f21cf0f6fb3661943155c3e3d8492"),
            nonce: hex!("23cb5ff362e22426984d1907"),
            aad: &hex!(
                "5d3624879d35e46849953e45a32a624d6a6c536ed9857c613b572b0333e701557a713e3f010ecdf9a6bd6c9e3e44b065208645aff4aabee611b391528514170084ccf587177f4488f33cfb5e979e42b6e1cfc0a60238982a7aec"
            ),
            plaintext: Some(&hex!(
                "42f758836986954db44bf37c6ef5e4ac0adaf38f27252a1b82d02ea949c8a1a2dbc0d68b5615ba7c1220ff6510e259f06655d8"
            )),
            ciphertext: &hex!(
                "81824f0e0d523db30d3da369fdc0d60894c7a0a20646dd015073ad2732bd989b14a222b6ad57af43e1895df9dca2a5344a62cc"
            ),
            tag: hex!("57a3ee28136e94c74838997ae9823f3a"),
        },
    ];

    // gcmDecrypt128.rsp, [PTlen = 408] [AADlen = 160], Count = 0
    // gcmDecrypt128.rsp, [PTlen = 408] [AADlen = 160], Count = 2 (FAIL)
    pub const DECRYPT_128: [Vector; 2] = [
        Vector {
            key: &hex!("af57f42c60c0fc5a09adb81ab86ca1c3"),
            nonce: hex!("a2dc01871f37025dc0fc9a79"),
            aad: &hex!("41dc38988945fcb44faf2ef72d0061289ef8efd8"),
            plaintext: Some(&hex!(
                "3803a0727eeb0ade441e0ec107161ded2d425ec0d102f21f51bf2cf9947c7ec4aa72795b2f69b041596e8817d0a3c16f8fadeb"
            )),
            ciphertext: &hex!(
                "b9a535864f48ea7b6b1367914978f9bfa087d854bb0e269bed8d279d2eea1210e48947338b22f9bad09093276a331e9c79c7f4"
            ),
            tag: hex!("4f71e72bde0018f555c5adcce062e005"),
        },
        Vector {
            key: &hex!("52bdbbf9cf477f187ec010589cb39d58"),
            nonce: hex!("d3be36d3393134951d324b31"),
            aad: &hex!("93c4fc6a4135f54d640b0c976bf755a06a292c33"),
            plaintext: None,
            ciphertext: &hex!(
                "700188da144fa692cf46e4a8499510a53d90903c967f7f13e8a1bd8151a74adc4fe63e32b992760b3a5f99e9a47838867000a9"
            ),
            tag: hex!("8ca4e38aa3dfa6b1d0297021ccf3ea5f"),
        },
    ];

    // gcmEncryptExtIV192.rsp, [PTlen = 104] [AADlen = 160], Count = 0
    // gcmEncryptExtIV192.rsp, [PTlen = 408] [AADlen = 720], Count = 0
    #[cfg(aes_key_length_192)]
    pub const ENCRYPT_192: [Vector; 2] = [
        Vector {
            key: &hex!("4bea36ca9c7a5cf6df8f8bb5be1e97647d014bd181d50e28"),
            nonce: hex!("bff3d447b453eeb78f3a5650"),
            aad: &hex!("33b7e18c7d5fd133faadd0ab524b86526e398203"),
            plaintext: Some(&hex!("d4f1053348696176de90c538c6")),
            ciphertext: &hex!("0d9f7cde2614b71ed9c4b93503"),
            tag: hex!("e88484488bd6470aa5e79b898c93dd28"),
        },
        Vector {
            key: &hex!("50f5cc5e33774f383235c64e77c502ca6c5d06ab0af6be04"),
            nonce: hex!("9d4c766490224946d91a99a7"),
            aad: &hex!(
                "4feca4174889298e992b765dfa67d33219f9b23898f8e869b7f1fbf5d71d87b44edd963d1055618b9db7bffe96b7856f3197868d6ce5e755a6c751661773e1def403a4cf036151ecae67b35f2891285420956604ef31c9aab341"
            ),
            plaintext: Some(&hex!(
                "35df59329eb5fc30c28791164bf9b374df228d2be467414727f457a9dd0f079f7342e23adb06929ce0ad607116ee11718ce320"
            )),
            ciphertext: &hex!(
                "77670f6520a34dfc05819a10f322b30e397473fe51f8274bfb105c2eec37e24951abaef11e73c3c425a8c4b2d8c3ffeb8ed924"
            ),
            tag: hex!("5349ef7a70499f2760123c0d5dcf5a6d"),
        },
    ];

    // gcmDecrypt192.rsp, [PTlen = 408] [AADlen = 160], Count = 2
    // gcmDecrypt192.rsp, [PTlen = 408] [AADlen = 160], Count = 0 (FAIL)
    #[cfg(aes_key_length_192)]
    pub const DECRYPT_192: [Vector; 2] = [
        Vector {
            key: &hex!("beb5a7901bbce4d5b9a9c7f19d31138a00b0c121d31faace"),
            nonce: hex!("bb6c4ebe43472f592aad175e"),
            aad: &hex!("fa66b0e1198022e461984e292f35ea347ad7cabc"),
            plaintext: Some(&hex!(
                "de8ca2991cff1f55111b135b817ef9f8b04a00db2776edbf8e414219c2cbfe21dd45d77759c50f1991e6ef27b9076f57e0e515"
            )),
            ciphertext: &hex!(
                "696b6ddcdb6c294caa50e1ae7d40aaef76faa36a93a8203ee9286f7922283bd5640142cb269e84f1239fd26ed0d573a1cd4470"
            ),
            tag: hex!("6c221a469347c2ff03ab6281c10af6b2"),
        },
        Vector {
            key: &hex!("39ee8afdc515cd2fed1e016df1d08c48d01b5ce7c799b32a"),
            nonce: hex!("72c9f8b19dca43db33a46cba"),
            aad: &hex!("e3158d92ef610031a9eae01742345dd890cdb65d"),
            plaintext: None,
            ciphertext: &hex!(
                "faa0f2639391ae8de58b74d1c4ecb85d8c3a254ac11795101b5a888db5f203e6551675a75d958b995c585694cb5fbd548ed142"
            ),
            tag: hex!("6f2351bfc1f2a16f54481a01eb9d37ba"),
        },
    ];

    // gcmEncryptExtIV256.rsp, [PTlen = 104] [AADlen = 160], Count = 0
    // gcmEncryptExtIV256.rsp, [PTlen = 408] [AADlen = 720], Count = 0
    pub const ENCRYPT_256: [Vector; 2] = [
        Vector {
            key: &hex!("69b458f2644af9020463b40ee503cdf083d693815e2659051ae0d039e606a970"),
            nonce: hex!("8d1da8ab5f91ccd09205944b"),
            aad: &hex!("036ad5e5494ef817a8af2f5828784a4bfedd1653"),
            plaintext: Some(&hex!("f3e0e09224256bf21a83a5de8d")),
            ciphertext: &hex!("c0a62d77e6031bfdc6b13ae217"),
            tag: hex!("a794a9aaee48cd92e47761bf1baff0af"),
        },
        Vector {
            key: &hex!("148579a3cbca86d5520d66c0ec71ca5f7e41ba78e56dc6eebd566fed547fe691"),
            nonce: hex!("b08a5ea1927499c6ecbfd4e0"),
            aad: &hex!(
                "e4f963f015ffbb99ee3349bbaf7e8e8e6c2a71c230a48f9d59860a29091d2747e01a5ca572347e247d25f56ba7ae8e05cde2be3c97931292c02370208ecd097ef692687fecf2f419d3200162a6480a57dad408a0dfeb492e2c5d"
            ),
            plaintext: Some(&hex!(
                "9d0b15fdf1bd595f91f8b3abc0f7dec927dfd4799935a1795d9ce00c9b879434420fe42c275a7cd7b39d638fb81ca52b49dc41"
            )),
            ciphertext: &hex!(
                "2097e372950a5e9383c675e89eea1c314f999159f5611344b298cda45e62843716f215f82ee663919c64002a5c198d7878fd3f"
            ),
            tag: hex!("adbecdb0d5c2224d804d2886ff9a5760"),
        },
    ];

    // gcmDecrypt256.rsp, [PTlen = 408] [AADlen = 160], Count = 2
    // gcmDecrypt256.rsp, [PTlen = 408] [AADlen = 160], Count = 0 (FAIL)
    pub const DECRYPT_256: [Vector; 2] = [
        Vector {
            key: &hex!("6450d4501b1e6cfbe172c4c8570363e96b496591b842661c28c2f6c908379cad"),
            nonce: hex!("7e4262035e0bf3d60e91668a"),
            aad: &hex!("f1c522f026e4c5d43851da516a1b78768ab18171"),
            plaintext: Some(&hex!(
                "17449e236ef5858f6d891412495ead4607bfae2a2d735182a2a0242f9d52fc5345ef912dbe16f3bb4576fe3bcafe336dee6085"
            )),
            ciphertext: &hex!(
                "5a99b336fd3cfd82f10fb08f7045012415f0d9a06bb92dcf59c6f0dbe62d433671aacb8a1c52ce7bbf6aea372bf51e2ba79406"
            ),
            tag: hex!("fe93b01636f7bb0458041f213e98de65"),
        },
        Vector {
            key: &hex!("e9d381a9c413bee66175d5586a189836e5c20f5583535ab4d3f3e612dc21700e"),
            nonce: hex!("23e81571da1c7821c681c7ca"),
            aad: &hex!("6f39c9ae7b8e8a58a95f0dd8ea6a9087cbccdfd6"),
            plaintext: None,
            ciphertext: &hex!(
                "a25f3f580306cd5065d22a6b7e9660110af7204bb77d370f7f34bee547feeff7b32a596fce29c9040e68b1589aad48da881990"
            ),
            tag: hex!("5b6dcd70eefb0892fab1539298b92a4b"),
        },
    ];

    pub fn hw_key(key: &[u8]) -> Key {
        match key.len() {
            16 => Key::from(<[u8; 16]>::try_from(key).unwrap()),
            #[cfg(aes_key_length_192)]
            24 => Key::from(<[u8; 24]>::try_from(key).unwrap()),
            32 => Key::from(<[u8; 32]>::try_from(key).unwrap()),
            _ => panic!("unsupported key length {}", key.len()),
        }
    }

    /// Encrypts `buffer` in place with RustCrypto's software AES-GCM, and returns the tag.
    pub fn sw_encrypt(key: &[u8], nonce: &[u8; 12], aad: &[u8], buffer: &mut [u8]) -> [u8; 16] {
        let tag = match key.len() {
            16 => AesGcm::<Aes128, U12>::new_from_slice(key)
                .unwrap()
                .encrypt_inout_detached(nonce.into(), aad, buffer.into()),
            #[cfg(aes_key_length_192)]
            24 => AesGcm::<aes_gcm::aes::Aes192, U12>::new_from_slice(key)
                .unwrap()
                .encrypt_inout_detached(nonce.into(), aad, buffer.into()),
            32 => AesGcm::<Aes256, U12>::new_from_slice(key)
                .unwrap()
                .encrypt_inout_detached(nonce.into(), aad, buffer.into()),
            _ => panic!("unsupported key length {}", key.len()),
        };
        tag.unwrap().into()
    }

    pub fn random<const N: usize>() -> [u8; N] {
        let mut bytes = [0; N];
        Rng::new().read(&mut bytes);
        bytes
    }

    pub fn random_vec(len: usize) -> Vec<u8, esp_alloc::InternalMemory> {
        let mut bytes = zeroed_vec(len);
        Rng::new().read(&mut bytes);
        bytes
    }

    pub fn zeroed_vec(len: usize) -> Vec<u8, esp_alloc::InternalMemory> {
        let mut bytes = Vec::with_capacity_in(len, esp_alloc::InternalMemory);
        bytes.resize(len, 0);
        bytes
    }

    pub fn copy_vec(source: &[u8]) -> Vec<u8, esp_alloc::InternalMemory> {
        let mut bytes = zeroed_vec(source.len());
        bytes.copy_from_slice(source);
        bytes
    }

    /// As [`compare_with_software`], waiting with `wait` instead of `wait_blocking`.
    pub async fn compare_with_software_async(key: &[u8], len: usize, aad_len: usize) {
        let nonce = random::<12>();
        let aad = random_vec(aad_len);
        let plaintext = random_vec(len);

        let mut expected = copy_vec(&plaintext);
        let expected_tag = sw_encrypt(key, &nonce, &aad, &mut expected);

        let mut gcm = AesGcmContext::new(hw_key(key));
        let mut ciphertext = zeroed_vec(len);
        let mut tag = [0; 16];
        let mut handle = gcm
            .encrypt(&nonce, &aad, &plaintext, &mut ciphertext, &mut tag)
            .unwrap();
        handle.wait().await.unwrap();
        core::mem::drop(handle);
        hil_test::assert!(
            ciphertext[..] == expected[..],
            "len {} aad {}",
            len,
            aad_len
        );
        hil_test::assert_eq!(tag, expected_tag, "len {} aad {}", len, aad_len);

        let mut handle = gcm
            .decrypt_in_place(&nonce, &aad, &mut ciphertext, &tag)
            .unwrap();
        handle.wait().await.unwrap();
        core::mem::drop(handle);
        hil_test::assert!(
            ciphertext[..] == plaintext[..],
            "len {} aad {}",
            len,
            aad_len
        );
    }

    /// Encrypts and decrypts `len` random bytes with `aad_len` random bytes of additional data on
    /// the hardware, and compares both with RustCrypto.
    pub fn compare_with_software(key: &[u8], len: usize, aad_len: usize) {
        let nonce = random::<12>();
        let aad = random_vec(aad_len);
        let plaintext = random_vec(len);

        let mut expected = copy_vec(&plaintext);
        let expected_tag = sw_encrypt(key, &nonce, &aad, &mut expected);

        let mut gcm = AesGcmContext::new(hw_key(key));
        let mut ciphertext = zeroed_vec(len);
        let mut tag = [0; 16];
        gcm.encrypt(&nonce, &aad, &plaintext, &mut ciphertext, &mut tag)
            .unwrap()
            .wait_blocking()
            .unwrap();
        hil_test::assert_eq!(
            &ciphertext[..],
            &expected[..],
            "len {} aad {}",
            len,
            aad_len
        );
        hil_test::assert_eq!(tag, expected_tag, "len {} aad {}", len, aad_len);

        gcm.decrypt_in_place(&nonce, &aad, &mut ciphertext, &tag)
            .unwrap()
            .wait_blocking()
            .unwrap();
        hil_test::assert_eq!(
            &ciphertext[..],
            &plaintext[..],
            "len {} aad {}",
            len,
            aad_len
        );
    }

    /// Decrypts `ciphertext` with `tag` and expects `TagMismatch` and an all-zero output.
    pub fn expect_mismatch(
        key: &[u8],
        nonce: &[u8; 12],
        aad: &[u8],
        ciphertext: &[u8],
        tag: &[u8; 16],
    ) {
        let mut gcm = AesGcmContext::new(hw_key(key));
        let mut output = zeroed_vec(ciphertext.len());
        output.fill(0xa5);
        let result = gcm
            .decrypt(nonce, aad, ciphertext, &mut output, tag)
            .unwrap()
            .wait_blocking();
        hil_test::assert_eq!(result, Err(GcmError::TagMismatch));
        hil_test::assert!(output.iter().all(|&b| b == 0), "output not zeroed");
    }

    pub const POISON: u32 = 0xa55a_5aa5;

    /// Writes a known word to the first key register, and asserts that it reads back.
    ///
    /// A register that does not read back what was written could read zero whatever it holds, so
    /// a zero read from it would prove nothing. The AES clock must be on, or every register reads
    /// zero.
    pub fn poison_key() {
        let regs = esp_hal::peripherals::AES::regs();
        regs.key(0).write(|w| unsafe { w.bits(POISON) });
        hil_test::assert_eq!(
            regs.key(0).read().bits(),
            POISON,
            "the key registers do not read back"
        );
    }

    /// Asserts that the key and H registers hold zero after an operation. The AES clock must be
    /// on.
    ///
    /// These are the secrets: H is the key's encryption of a zero block. J0 comes from the nonce
    /// and the tag goes out with the ciphertext. Software cannot write H between operations, so
    /// no test can first show that H reads back.
    pub fn assert_secrets_clear() {
        let regs = esp_hal::peripherals::AES::regs();
        for i in 0..8 {
            hil_test::assert_eq!(regs.key(i).read().bits(), 0, "key word {}", i);
        }
        for i in 0..4 {
            hil_test::assert_eq!(regs.h_mem(i).read().bits(), 0, "H word {}", i);
        }
    }

    /// Keeps the AES clock on while the test reads registers, beside the backend's own driver.
    ///
    /// Call it before the backend starts: `Aes::new` turns DMA mode off, which would stop an
    /// operation in progress.
    pub fn hold_aes_clock() -> esp_hal::aes::Aes<'static> {
        esp_hal::aes::Aes::new(unsafe { esp_hal::peripherals::AES::steal() })
    }

    pub fn expect_refused(
        gcm: &mut AesGcmContext,
        aad: &[u8],
        input: &[u8],
        output: &mut [u8],
        expected: Error,
    ) {
        let nonce = [0x11; 12];
        let mut tag = [0x5a; 16];
        output.fill(0xa5);
        let result = gcm
            .encrypt(&nonce, aad, input, output, &mut tag)
            .map(|_| ());
        hil_test::assert_eq!(result, Err(expected));
        hil_test::assert_eq!(tag, [0x5a; 16]);
        hil_test::assert!(output.iter().all(|&b| b == 0xa5), "output changed");
    }
}

#[cfg(aes_dma_mode_gcm)]
#[embedded_test::tests(default_timeout = 30, executor = hil_test::Executor::new())]
mod gcm_tests {
    use allocator_api2::vec::Vec;
    use esp_hal::{
        aes::{
            Error,
            dma::{AesDmaBackend, AesGcmContext, GcmError},
        },
        rng::Rng,
    };

    use super::{gcm::*, *};

    struct Context {
        aes: AesDmaBackend<'static>,
    }

    #[init]
    fn init() -> Context {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        gcm_heap();

        #[cfg(all(soc_has_psram, dma_can_access_psram))]
        esp_alloc::psram_allocator!(p.PSRAM, esp_hal::psram);

        let dma = cfg_select! {
            aes_dma_engine = "CRYPTO_DMA" => p.DMA_CRYPTO,
            aes_dma_engine = "AXI_GDMA" => p.DMA_AXI_CH0,
            _ => p.DMA_CH0,
        };

        Context {
            aes: AesDmaBackend::new(p.AES, dma),
        }
    }

    fn run_vectors(encrypt: &[Vector], decrypt: &[Vector]) {
        // The vectors are in flash, and the DMA reads the additional data where it is.
        for v in encrypt {
            let aad = v.aad.to_vec();
            let mut gcm = AesGcmContext::new(hw_key(v.key));
            let plaintext = v.plaintext.unwrap();
            let mut output = zeroed_vec(plaintext.len());
            let mut tag = [0; 16];
            gcm.encrypt(&v.nonce, &aad, plaintext, &mut output, &mut tag)
                .unwrap()
                .wait_blocking()
                .unwrap();
            hil_test::assert_eq!(&output[..], v.ciphertext);
            hil_test::assert_eq!(tag, v.tag);
        }
        for v in decrypt {
            let aad = v.aad.to_vec();
            match v.plaintext {
                Some(plaintext) => {
                    let mut gcm = AesGcmContext::new(hw_key(v.key));
                    let mut output = zeroed_vec(v.ciphertext.len());
                    gcm.decrypt(&v.nonce, &aad, v.ciphertext, &mut output, &v.tag)
                        .unwrap()
                        .wait_blocking()
                        .unwrap();
                    hil_test::assert_eq!(&output[..], plaintext);
                }
                None => expect_mismatch(v.key, &v.nonce, &aad, v.ciphertext, &v.tag),
            }
        }
    }

    /// NIST CAVP vectors, for correctness against a published source.
    #[test]
    fn test_gcm_nist_vectors(mut ctx: Context) {
        let _backend = ctx.aes.start();

        run_vectors(&ENCRYPT_128, &DECRYPT_128);
        #[cfg(aes_key_length_192)]
        run_vectors(&ENCRYPT_192, &DECRYPT_192);
        run_vectors(&ENCRYPT_256, &DECRYPT_256);
    }

    /// Text and additional-data lengths on both sides of one block, one DMA descriptor and two
    /// descriptors, against RustCrypto.
    #[test]
    fn test_gcm_lengths(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let key_128 = random::<16>();
        let key_256 = random::<32>();
        for len in [1, 15, 16, 17, 4_079, 4_080, 4_081, 8_161, 16_385] {
            for aad_len in [0, 5, 16, 17, MAX_AAD] {
                compare_with_software(&key_128, len, aad_len);
                compare_with_software(&key_256, len, aad_len);
            }
        }
        #[cfg(aes_key_length_192)]
        compare_with_software(&random::<24>(), 4_081, 17);
    }

    /// The longest text the driver takes.
    #[test]
    fn test_gcm_max_text(mut ctx: Context) {
        let _backend = ctx.aes.start();

        compare_with_software(&random::<16>(), MAX_TEXT, 5);
    }

    /// The longest text and additional data in PSRAM, where the larger alignment fills every
    /// descriptor the driver owns.
    #[test]
    #[cfg(all(soc_has_psram, dma_can_access_psram))]
    fn test_gcm_max_lengths_psram(mut ctx: Context) {
        let _backend = ctx.aes.start();

        fn psram_vec(len: usize) -> Vec<u8, esp_alloc::ExternalMemory> {
            let mut bytes = Vec::with_capacity_in(len, esp_alloc::ExternalMemory);
            bytes.resize(len, 0);
            Rng::new().read(&mut bytes);
            bytes
        }

        let key = random::<32>();
        let nonce = random::<12>();
        let aad = psram_vec(MAX_AAD);
        let plaintext = psram_vec(MAX_TEXT);
        let mut expected = copy_vec(&plaintext);
        let expected_tag = sw_encrypt(&key, &nonce, &aad, &mut expected);

        let mut gcm = AesGcmContext::new(hw_key(&key));
        let mut ciphertext = psram_vec(MAX_TEXT);
        let mut tag = [0; 16];
        gcm.encrypt(&nonce, &aad, &plaintext, &mut ciphertext, &mut tag)
            .unwrap()
            .wait_blocking()
            .unwrap();
        hil_test::assert!(ciphertext[..] == expected[..], "ciphertext differs");
        hil_test::assert_eq!(tag, expected_tag);

        gcm.decrypt_in_place(&nonce, &aad, &mut ciphertext, &tag)
            .unwrap()
            .wait_blocking()
            .unwrap();
        hil_test::assert!(ciphertext[..] == plaintext[..], "plaintext differs");
    }

    fn run_shifts(memory: &mut [u8]) {
        const LEN: usize = 100;
        let key = random::<16>();
        let nonce = random::<12>();
        let aad = random::<5>();
        let mut plaintext = [0; LEN];
        Rng::new().read(&mut plaintext);
        let mut expected = plaintext;
        let expected_tag = sw_encrypt(&key, &nonce, &aad, &mut expected);
        let mut gcm = AesGcmContext::new(hw_key(&key));

        for shift in 0..16 {
            memory.fill(0);
            let buffer = &mut memory[shift..][..LEN];
            buffer.copy_from_slice(&plaintext);
            let mut tag = [0; 16];
            gcm.encrypt_in_place(&nonce, &aad, buffer, &mut tag)
                .unwrap()
                .wait_blocking()
                .unwrap();
            hil_test::assert_eq!(buffer, &expected[..], "in place, shift {}", shift);
            hil_test::assert_eq!(tag, expected_tag, "in place, shift {}", shift);

            gcm.decrypt_in_place(&nonce, &aad, buffer, &tag)
                .unwrap()
                .wait_blocking()
                .unwrap();
            hil_test::assert_eq!(buffer, &plaintext[..], "in place, shift {}", shift);
            hil_test::assert!(
                memory[..shift].iter().all(|&b| b == 0),
                "write before, shift {}",
                shift
            );
            hil_test::assert!(
                memory[shift + LEN..].iter().all(|&b| b == 0),
                "write after, shift {}",
                shift
            );

            memory.fill(0);
            let buffer = &mut memory[shift..][..LEN];
            gcm.encrypt(&nonce, &aad, &plaintext, buffer, &mut tag)
                .unwrap()
                .wait_blocking()
                .unwrap();
            hil_test::assert_eq!(buffer, &expected[..], "separate, shift {}", shift);
            hil_test::assert_eq!(tag, expected_tag, "separate, shift {}", shift);
        }
    }

    /// Alignment in internal memory, in place and to a separate buffer.
    #[test]
    fn test_gcm_shifts(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let mut memory = zeroed_vec(100 + 16 + 64);
        run_shifts(&mut memory);
    }

    /// Alignment in PSRAM, in place and to a separate buffer.
    #[test]
    #[cfg(all(soc_has_psram, dma_can_access_psram))]
    fn test_gcm_shifts_psram(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let mut memory = Vec::with_capacity_in(100 + 16 + 64, esp_alloc::ExternalMemory);
        memory.resize(100 + 16 + 64, 0);
        run_shifts(&mut memory);
    }

    fn run_guard_byte(memory: &mut [u8]) {
        let key = random::<16>();
        let nonce = random::<12>();
        let plaintext = random::<17>();
        let mut expected = plaintext;
        let expected_tag = sw_encrypt(&key, &nonce, &[], &mut expected);

        memory.fill(0xa5);
        let (output, guard) = memory.split_at_mut(17);
        let mut tag = [0; 16];
        AesGcmContext::new(hw_key(&key))
            .encrypt(&nonce, &[], &plaintext, output, &mut tag)
            .unwrap()
            .wait_blocking()
            .unwrap();
        hil_test::assert_eq!(output, &expected[..]);
        hil_test::assert_eq!(tag, expected_tag);
        hil_test::assert!(guard.iter().all(|&b| b == 0xa5), "guard bytes changed");
    }

    /// A text one byte longer than a block does not overwrite the memory after the buffer.
    #[test]
    fn test_gcm_guard_byte(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let mut memory = zeroed_vec(17 + 64);
        run_guard_byte(&mut memory);
    }

    /// A text one byte longer than a block does not overwrite the memory after the buffer, in
    /// PSRAM.
    #[test]
    #[cfg(all(soc_has_psram, dma_can_access_psram))]
    fn test_gcm_guard_byte_psram(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let mut memory = Vec::with_capacity_in(17 + 64, esp_alloc::ExternalMemory);
        memory.resize(17 + 64, 0);
        run_guard_byte(&mut memory);
    }

    /// A text in flash, which the backend copies to the output before the DMA reads it.
    #[test]
    fn test_gcm_text_in_flash(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let v = &ENCRYPT_128[1];
        let mut output = zeroed_vec(v.ciphertext.len());
        let mut tag = [0; 16];
        let aad = v.aad.to_vec();
        AesGcmContext::new(hw_key(v.key))
            .encrypt(&v.nonce, &aad, v.plaintext.unwrap(), &mut output, &mut tag)
            .unwrap()
            .wait_blocking()
            .unwrap();
        hil_test::assert_eq!(&output[..], v.ciphertext);
        hil_test::assert_eq!(tag, v.tag);
    }

    /// A changed tag, changed additional data and a changed last partial block each end in
    /// `TagMismatch` with the output zeroed.
    #[test]
    fn test_gcm_tag_mismatch(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let key = random::<16>();
        let nonce = random::<12>();
        let aad = random::<16>();
        let mut ciphertext = random::<33>();
        let tag = sw_encrypt(&key, &nonce, &aad, &mut ciphertext);

        let mut first = tag;
        first[0] ^= 1;
        expect_mismatch(&key, &nonce, &aad, &ciphertext, &first);

        let mut last = tag;
        last[15] ^= 0x80;
        expect_mismatch(&key, &nonce, &aad, &ciphertext, &last);

        let mut changed_aad = aad;
        changed_aad[9] ^= 1;
        expect_mismatch(&key, &nonce, &changed_aad, &ciphertext, &tag);

        let mut changed_text = ciphertext;
        changed_text[32] ^= 1;
        expect_mismatch(&key, &nonce, &aad, &changed_text, &tag);
    }

    /// Inputs the hardware path does not take are refused before anything is posted.
    #[test]
    fn test_gcm_refusals(mut ctx: Context) {
        let _backend = ctx.aes.start();

        static AAD_IN_FLASH: [u8; 5] = [1, 2, 3, 4, 5];

        let mut gcm = AesGcmContext::new(hw_key(&random::<16>()));
        let aad = zeroed_vec(MAX_AAD + 1);
        let input = zeroed_vec(MAX_TEXT + 1);
        let mut output = zeroed_vec(MAX_TEXT + 1);

        expect_refused(&mut gcm, &[], &[], &mut [], Error::IncorrectBufferLength);
        expect_refused(
            &mut gcm,
            &aad,
            &input[..16],
            &mut output[..16],
            Error::IncorrectBufferLength,
        );
        expect_refused(
            &mut gcm,
            &[],
            &input,
            &mut output,
            Error::IncorrectBufferLength,
        );
        expect_refused(
            &mut gcm,
            &AAD_IN_FLASH,
            &input[..16],
            &mut output[..16],
            Error::AadNotInRam,
        );
        expect_refused(
            &mut gcm,
            &[],
            &input[..16],
            &mut output[..17],
            Error::BuffersNotEqual,
        );

        // A refused operation leaves nothing in the queue.
        compare_with_software(&random::<16>(), 17, 5);
    }

    /// The block reset at the end of each operation leaves no key and no H in the registers: after
    /// an encryption and a decryption, and after a mismatch.
    #[test]
    fn test_gcm_clears_secrets(mut ctx: Context) {
        let _clock = hold_aes_clock();
        let _backend = ctx.aes.start();

        poison_key();
        compare_with_software(&random::<32>(), 33, 5);
        assert_secrets_clear();

        let key = random::<32>();
        let nonce = random::<12>();
        let mut ciphertext = random::<33>();
        let mut tag = sw_encrypt(&key, &nonce, &[], &mut ciphertext);
        tag[0] ^= 1;
        poison_key();
        expect_mismatch(&key, &nonce, &[], &ciphertext, &tag);
        assert_secrets_clear();
    }

    /// A decryption cancelled before it starts ends as `Cancelled`, with its output zeroed, and an
    /// encryption cancelled the same way leaves its tag untouched.
    #[test]
    async fn test_gcm_cancel_before_start(_ctx: Context) {
        let key = random::<16>();
        let nonce = random::<12>();
        let mut ciphertext = random::<33>();
        let tag = sw_encrypt(&key, &nonce, &[], &mut ciphertext);

        let mut gcm = AesGcmContext::new(hw_key(&key));
        let mut output = [0xa5; 33];
        // No backend runs, so the operation stays in the queue until it is cancelled.
        let mut handle = gcm
            .decrypt(&nonce, &[], &ciphertext, &mut output, &tag)
            .unwrap();
        handle.cancel().await;
        hil_test::assert_eq!(handle.wait().await, Err(GcmError::Cancelled));
        core::mem::drop(handle);
        hil_test::assert_eq!(output, [0; 33]);

        let mut tag = [0x5a; 16];
        let mut handle = gcm
            .encrypt(&nonce, &[], &ciphertext, &mut output, &mut tag)
            .unwrap();
        handle.cancel().await;
        hil_test::assert_eq!(handle.wait().await, Err(GcmError::Cancelled));
        core::mem::drop(handle);
        hil_test::assert_eq!(tag, [0x5a; 16]);
    }

    /// Dropping the backend during an operation waits for the operation to complete, with its tag
    /// checked.
    #[test]
    fn test_gcm_backend_dropped_during_operation(mut ctx: Context) {
        let key = random::<16>();
        let nonce = random::<12>();
        let plaintext = random_vec(MAX_TEXT);
        let mut ciphertext = copy_vec(&plaintext);
        let tag = sw_encrypt(&key, &nonce, &[], &mut ciphertext);

        let mut gcm = AesGcmContext::new(hw_key(&key));
        let mut output = zeroed_vec(MAX_TEXT);
        let _clock = hold_aes_clock();
        poison_key();
        let backend = ctx.aes.start();
        let mut handle = gcm
            .decrypt(&nonce, &[], &ciphertext, &mut output, &tag)
            .unwrap();
        // The first poll hands the operation to the backend, which starts the DMA.
        hil_test::assert!(
            !handle.poll(),
            "operation finished before the backend was dropped"
        );
        core::mem::drop(backend);

        hil_test::assert_eq!(handle.wait_blocking(), Ok(()));
        hil_test::assert_eq!(&output[..], &plaintext[..]);
        assert_secrets_clear();
    }

    /// Three operations posted before the backend starts. The last is waited first, so the earlier
    /// results must survive until they are asked for.
    #[test]
    fn test_gcm_posted_before_start(mut ctx: Context) {
        let (key0, key1, key2) = (random::<16>(), random::<32>(), random::<16>());
        let keys: [&[u8]; 3] = [&key0, &key1, &key2];
        let nonce = random::<12>();
        let aad = random::<5>();
        let plaintext = random::<40>();

        let mut expected = [plaintext; 3];
        let mut expected_tags = [[0; 16]; 3];
        for i in 0..3 {
            expected_tags[i] = sw_encrypt(&keys[i], &nonce, &aad, &mut expected[i]);
        }

        let [mut gcm0, mut gcm1, mut gcm2] = keys.map(|k| AesGcmContext::new(hw_key(k)));
        let [mut out0, mut out1, mut out2] = [[0; 40]; 3];
        let [mut tag0, mut tag1, mut tag2] = [[0; 16]; 3];
        let h0 = gcm0
            .encrypt(&nonce, &aad, &plaintext, &mut out0, &mut tag0)
            .unwrap();
        let h1 = gcm1
            .encrypt(&nonce, &aad, &plaintext, &mut out1, &mut tag1)
            .unwrap();
        let h2 = gcm2
            .encrypt(&nonce, &aad, &plaintext, &mut out2, &mut tag2)
            .unwrap();

        let _backend = ctx.aes.start();

        h2.wait_blocking().unwrap();
        h1.wait_blocking().unwrap();
        h0.wait_blocking().unwrap();

        hil_test::assert_eq!([out0, out1, out2], expected);
        hil_test::assert_eq!([tag0, tag1, tag2], expected_tags);
    }

    /// An async wait.
    #[test]
    async fn test_gcm_async(mut ctx: Context) {
        let _backend = ctx.aes.start();

        let key = random::<16>();
        let nonce = random::<12>();
        let aad = random::<5>();
        let plaintext = random::<100>();
        let mut expected = plaintext;
        let expected_tag = sw_encrypt(&key, &nonce, &aad, &mut expected);

        let mut gcm = AesGcmContext::new(hw_key(&key));
        let mut output = [0; 100];
        let mut tag = [0; 16];
        let mut handle = gcm
            .encrypt(&nonce, &aad, &plaintext, &mut output, &mut tag)
            .unwrap();
        handle.wait().await.unwrap();
        core::mem::drop(handle);

        hil_test::assert_eq!(output, expected);
        hil_test::assert_eq!(tag, expected_tag);
    }

    /// GCM and ECB operations queued together. The last is waited first, so the earlier results
    /// must survive until they are asked for. Each GCM operation sets its own block mode, key and
    /// lengths, and leaves nothing that changes the next operation.
    #[test]
    fn test_gcm_mixed_with_ecb(mut ctx: Context) {
        let mut plaintext = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut plaintext);
        let mut ecb_128 = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        let mut ecb_256 = AesContext::new(Ecb, Operation::Encrypt, KEY_256);
        let mut ecb_128_out = [0; PLAINTEXT_BUF_SIZE];
        let mut ecb_256_out = [0; PLAINTEXT_BUF_SIZE];

        let (key_a, key_b) = (random::<16>(), random::<32>());
        let (nonce_a, nonce_b) = (random::<12>(), random::<12>());
        let aad_a = random::<16>();
        let text_a = random::<17>();
        let text_b = random::<16>();
        let mut expected_a = text_a;
        let expected_tag_a = sw_encrypt(&key_a, &nonce_a, &aad_a, &mut expected_a);
        let mut expected_b = text_b;
        let expected_tag_b = sw_encrypt(&key_b, &nonce_b, &[], &mut expected_b);

        let mut gcm_a = AesGcmContext::new(hw_key(&key_a));
        let mut gcm_b = AesGcmContext::new(hw_key(&key_b));
        let (mut out_a, mut out_b) = ([0; 17], [0; 16]);
        let (mut tag_a, mut tag_b) = ([0; 16], [0; 16]);

        let ecb_128_handle = ecb_128.process(&plaintext, &mut ecb_128_out).unwrap();
        let gcm_a_handle = gcm_a
            .encrypt(&nonce_a, &aad_a, &text_a, &mut out_a, &mut tag_a)
            .unwrap();
        let ecb_256_handle = ecb_256.process(&plaintext, &mut ecb_256_out).unwrap();
        let gcm_b_handle = gcm_b
            .encrypt(&nonce_b, &[], &text_b, &mut out_b, &mut tag_b)
            .unwrap();

        let _backend = ctx.aes.start();

        gcm_b_handle.wait_blocking().unwrap();
        ecb_256_handle.wait_blocking();
        gcm_a_handle.wait_blocking().unwrap();
        ecb_128_handle.wait_blocking();

        hil_test::assert_eq!(ecb_128_out, CIPHERTEXT_ECB_128);
        hil_test::assert_eq!(ecb_256_out, CIPHERTEXT_ECB_256);
        hil_test::assert_eq!(out_a, expected_a);
        hil_test::assert_eq!(tag_a, expected_tag_a);
        hil_test::assert_eq!(out_b, expected_b);
        hil_test::assert_eq!(tag_b, expected_tag_b);
    }

    /// Two workers, each with its own key, interleave GCM operations through the queue, and each
    /// gets its own results.
    #[test]
    async fn test_gcm_two_workers(mut ctx: Context) {
        async fn worker() {
            let key = random::<32>();
            for _ in 0..20 {
                let [len, aad_len] = random::<2>();
                compare_with_software_async(&key, 1 + len as usize * 4, aad_len as usize % 64)
                    .await;
            }
        }

        let _backend = ctx.aes.start();

        embassy_futures::join::join(worker(), worker()).await;
    }
}

#[cfg(aes_dma_mode_gcm)]
#[embedded_test::tests(default_timeout = 10, executor = hil_test::Executor::new())]
mod gcm_cpu_backend_tests {
    use esp_hal::aes::dma::{AesGcmContext, GcmError};

    use super::{gcm::*, *};

    /// A GCM item on the CPU backend ends as `Unsupported`, and leaves the output, the tag and the
    /// key registers untouched.
    #[test]
    fn test_gcm_unsupported_on_cpu_backend() {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));
        let _clock = hold_aes_clock();
        poison_key();
        let mut aes = AesBackend::new(p.AES);
        let _backend = aes.start();

        let mut gcm = AesGcmContext::new(hw_key(&random::<16>()));
        let plaintext = random::<17>();
        let mut output = [0xa5; 17];
        let mut tag = [0x5a; 16];
        let result = gcm
            .encrypt(&[0x11; 12], &[], &plaintext, &mut output, &mut tag)
            .unwrap()
            .wait_blocking();

        hil_test::assert_eq!(result, Err(GcmError::Unsupported));
        hil_test::assert_eq!(output, [0xa5; 17]);
        hil_test::assert_eq!(tag, [0x5a; 16]);
        // The backend wrote no key: the word written before the operation is still there.
        hil_test::assert_eq!(
            esp_hal::peripherals::AES::regs().key(0).read().bits(),
            POISON
        );
    }
}

/// Tests of the AES work queue under contention: a waiter at a higher interrupt priority, and a
/// second core. GCM and ECB operations are the load.
#[cfg(aes_dma_mode_gcm)]
#[embedded_test::tests(default_timeout = 30, executor = hil_test::Executor::new())]
mod work_queue_concurrency_tests {
    use esp_hal::{
        aes::dma::{AesDmaBackend, AesGcmContext},
        interrupt::{InterruptHandler, Priority, software::SoftwareInterrupt},
        peripherals::{FROM_CPU_INTR3, Peripherals},
        time::{Duration, Instant},
    };
    use portable_atomic::{AtomicU8, Ordering};

    use super::{gcm::*, *};

    const NOT_DONE: u8 = 0;
    const PASSED: u8 = 1;
    const FAILED: u8 = 2;
    const TIMED_OUT: u8 = 3;

    #[init]
    fn init() -> Peripherals {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));
        gcm_heap();
        p
    }

    /// Encrypts 100 random bytes with GCM and compares the result with RustCrypto, using only the
    /// stack.
    #[cfg(all(multi_core, soc_has_cpu_ctrl))]
    fn gcm_round() -> bool {
        let key = random::<16>();
        let nonce = random::<12>();
        let aad = random::<16>();
        let plaintext = random::<100>();
        let mut expected = plaintext;
        let expected_tag = sw_encrypt(&key, &nonce, &aad, &mut expected);

        let mut gcm = AesGcmContext::new(hw_key(&key));
        let mut output = [0; 100];
        let mut tag = [0; 16];
        let result = gcm
            .encrypt(&nonce, &aad, &plaintext, &mut output, &mut tag)
            .unwrap()
            .wait_blocking();
        result.is_ok() && output == expected && tag == expected_tag
    }

    /// Encrypts the known plaintext with ECB and compares it with the known ciphertext, using only
    /// the stack.
    #[cfg(all(multi_core, soc_has_cpu_ctrl))]
    fn ecb_round() -> bool {
        let mut plaintext = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut plaintext);
        let mut output = [0; PLAINTEXT_BUF_SIZE];
        let mut ecb = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        ecb.process(&plaintext, &mut output)
            .unwrap()
            .wait_blocking();
        output == CIPHERTEXT_ECB_128
    }

    /// As [`ecb_round`], but polls for at most `limit`, so a queue that stops making progress
    /// fails the test instead of hanging it. Returns `None` when the operation has not completed
    /// by then.
    fn ecb_round_within(limit: Duration) -> Option<bool> {
        let mut plaintext = [0; PLAINTEXT_BUF_SIZE];
        fill_with_plaintext(&mut plaintext);
        let mut output = [0; PLAINTEXT_BUF_SIZE];
        let mut ecb = AesContext::new(Ecb, Operation::Encrypt, KEY_128);
        let mut handle = ecb.process(&plaintext, &mut output).unwrap();
        let start = Instant::now();
        while !handle.poll() {
            if start.elapsed() > limit {
                // Dropping the handle removes the operation at once while it waits in the queue.
                // When the backend has started it, the drop waits for the hardware to finish it.
                return None;
            }
        }
        core::mem::drop(handle);
        Some(output == CIPHERTEXT_ECB_128)
    }

    /// A handler at a higher priority than the AES interrupt posts an ECB operation and waits for
    /// it while a GCM decryption from thread mode is in progress. The handler's wait processes
    /// the queue, because the AES interrupt cannot run until the handler returns.
    ///
    /// The handler runs at `Priority::max()`, the highest level a handler can have. That level is
    /// still below what the queue's critical section masks.
    #[test]
    fn test_wait_at_higher_priority_during_gcm(p: Peripherals) {
        static HANDLER_RESULT: AtomicU8 = AtomicU8::new(NOT_DONE);

        extern "C" fn high_priority() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).reset();
            let result = match ecb_round_within(Duration::from_secs(1)) {
                Some(true) => PASSED,
                Some(false) => FAILED,
                None => TIMED_OUT,
            };
            HANDLER_RESULT.store(result, Ordering::Release);
        }

        let dma = cfg_select! {
            aes_dma_engine = "CRYPTO_DMA" => p.DMA_CRYPTO,
            aes_dma_engine = "AXI_GDMA" => p.DMA_AXI_CH0,
            _ => p.DMA_CH0,
        };
        let mut aes = AesDmaBackend::new(p.AES, dma);
        let _backend = aes.start();

        let key = random::<16>();
        let nonce = random::<12>();
        let plaintext = random_vec(MAX_TEXT);
        let mut ciphertext = copy_vec(&plaintext);
        let tag = sw_encrypt(&key, &nonce, &[], &mut ciphertext);

        let mut gcm = AesGcmContext::new(hw_key(&key));
        let mut output = zeroed_vec(MAX_TEXT);
        let mut handle = gcm
            .decrypt(&nonce, &[], &ciphertext, &mut output, &tag)
            .unwrap();
        // The first poll hands the decryption to the backend, which starts the DMA.
        hil_test::assert!(!handle.poll(), "decryption finished before the handler ran");

        let mut interrupt = SoftwareInterrupt::new(p.FROM_CPU_INTR3);
        interrupt.set_interrupt_handler(InterruptHandler::new(high_priority, Priority::max()));
        interrupt.raise();

        // The core can take the interrupt a few instructions after the raise.
        let start = Instant::now();
        while HANDLER_RESULT.load(Ordering::Acquire) == NOT_DONE {
            hil_test::assert!(
                start.elapsed() < Duration::from_secs(1),
                "the handler did not run"
            );
        }
        hil_test::assert_eq!(HANDLER_RESULT.load(Ordering::Acquire), PASSED);
        hil_test::assert_eq!(handle.wait_blocking(), Ok(()));
        hil_test::assert!(output[..] == plaintext[..], "decryption differs");
    }

    /// Both cores post GCM and ECB operations to the one queue at the same time, and each gets
    /// its own results. Core 1 uses only its stack and two atomics that core 0 owns.
    #[test]
    #[cfg(all(multi_core, soc_has_cpu_ctrl))]
    fn test_two_cores(p: Peripherals) {
        use esp_hal::system::{CpuControl, Stack};
        use portable_atomic::AtomicBool;

        const ROUNDS: usize = 20;
        let start_signal = AtomicBool::new(false);
        let core_1_result = AtomicU8::new(NOT_DONE);

        let dma = cfg_select! {
            aes_dma_engine = "CRYPTO_DMA" => p.DMA_CRYPTO,
            aes_dma_engine = "AXI_GDMA" => p.DMA_AXI_CH0,
            _ => p.DMA_CH0,
        };
        let mut aes = AesDmaBackend::new(p.AES, dma);
        let _backend = aes.start();

        let core_1 = || {
            while !start_signal.load(Ordering::Acquire) {}
            let mut passed = true;
            for _ in 0..ROUNDS {
                passed &= gcm_round();
            }
            core_1_result.store(if passed { PASSED } else { FAILED }, Ordering::Release);
            loop {}
        };

        let mut cpu_control = CpuControl::new(p.CPU_CTRL);
        let stack = mk_static!(Stack<16384>, Stack::new());
        let _guard = cpu_control.start_app_core(stack, core_1).unwrap();

        start_signal.store(true, Ordering::Release);
        for round in 0..ROUNDS {
            hil_test::assert!(ecb_round(), "core 0 ECB round {}", round);
            hil_test::assert!(gcm_round(), "core 0 GCM round {}", round);
        }

        let start = Instant::now();
        while core_1_result.load(Ordering::Acquire) == NOT_DONE {
            hil_test::assert!(
                start.elapsed() < Duration::from_secs(10),
                "core 1 did not finish"
            );
        }
        hil_test::assert_eq!(core_1_result.load(Ordering::Acquire), PASSED);
    }
}
