# Migration Guide from 1.1.0 to {{currentVersion}}

## `Clocks` struct removed

The `Clocks` struct and `Clocks::get()` have been removed. The clock frequencies
they provided are now available as lockless free functions from `esp_hal::clock`
and (for chip-specific clocks) `esp_hal::clock::ll`.

| Old | New |
|---|---|
| `Clocks::get().cpu_clock` | `clock::cpu_clock()` |
| `Clocks::get().apb_clock` | `Rate::from_hz(clock::ll::apb_clk_frequency())` |
| `Clocks::get().xtal_clock` | `clock::xtal_clock()` |

```diff
-use esp_hal::clock::Clocks;
+use esp_hal::clock;

-let cpu_freq = Clocks::get().cpu_clock;
-let apb_freq = Clocks::get().apb_clock;
-let xtal_freq = Clocks::get().xtal_clock;
+let cpu_freq = clock::cpu_clock();
+let apb_freq = Rate::from_hz(clock::ll::apb_clk_frequency());
+let xtal_freq = clock::xtal_clock();
```
