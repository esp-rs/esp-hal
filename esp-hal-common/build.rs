fn main() {
    let chip_features = [
        cfg!(feature = "32"),
        cfg!(feature = "32c3"),
        cfg!(feature = "32s2"),
        cfg!(feature = "32s3"),
    ];

    if chip_features.iter().filter(|&&f| f).count() != 1 {
        panic!("Exactly one chip must be enabled via its cargo feature");
    }
}
