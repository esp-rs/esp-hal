# Migration Guide from 0.10.x to v0.11.x

## No need to add `rom_functions.x`

Don't add `rom_functions.x` manually in `build.rs` or `config.toml`

`build.rs`
```diff
- println!("cargo::rustc-link-arg=-Trom_functions.x");
```

`config.toml`
```diff
-     "-C", "link-arg=-Trom_functions.x",
```

