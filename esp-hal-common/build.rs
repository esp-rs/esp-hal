fn main() {
    let chip_features = [
        cfg!(feature = "esp32"),
        cfg!(feature = "esp32c3"),
        cfg!(feature = "esp32s2"),
        cfg!(feature = "esp32s3"),
    ];

    if chip_features.iter().filter(|&&f| f).count() != 1 {
        panic!("Exactly one chip must be enabled via its cargo feature");
    }
}
