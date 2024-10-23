# Migration Guide from 0.10.x to v0.11.x

## No need to include `rom_functions.x` manually

Don't include `rom_functions.x` from esp-wifi

```diff
rustflags = [
    "-C", "link-arg=-Tlinkall.x",
-    "-C", "link-arg=-Trom_functions.x",
]
```

## ESP-NOW: Use `get_data` to access the received payload

Previously `data` and `len` were public - use the previously already existing `get_data` function.

Accessing `data` or `len` was never encouraged.
