# Migration Guide from 0.17.0 to {{currentVersion}}

## The `serde` feature has been removed

You will have to provide your own datatypes that you wish to serialize/deserialize. For this, you have two options:

1. Implement the `Serialize` and `Deserialize` traits [manually](https://serde.rs/impl-serialize.html) for your custom types.
2. Implement the types and conversions to/from esp-radio types.

For example, you may want to mirror `ScanMethod`, so that you can store it in flash as a configuraton option. In this case, you could do the following:

```rust
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Serialize, Deserialize)] // and possibly more
pub enum ScanMethod {
    Fast,
    AllChannels,
}

impl From<ScanMethod> for esp_radio::wifi::ScanMethod {
    fn from(scan_method: ScanMethod) -> Self {
        match scan_method {
            ScanMethod::Fast => Self::Fast,
            ScanMethod::AllChannels => Self::AllChannels,
        }
    }
}
```
