# Compile Tests

The projects here are using fixed versions and are meant to make sure that an upcoming release won't break existing code.

The `hal` project's pins track the current lines and are rewritten when those
crates are released. Frozen `ble_*` / `wifi_*` projects stay on the minor they
were snapshotted at; bump those by hand when you want a new frozen line.

Make sure to never check-in `Cargo.lock` files here.
