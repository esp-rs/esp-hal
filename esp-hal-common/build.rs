fn main() {
    let chip_features = [
        cfg!(feature = "enable-esp32"),
        cfg!(feature = "enable-esp32c3"),
        cfg!(feature = "enable-esp32s2"),
    ];

    if chip_features.iter().filter(|&&f| f).count() != 1 {
        panic!("Exactly one chip must be selected via its cargo feature");
    }
}
