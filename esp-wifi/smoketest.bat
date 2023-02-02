REM A simple script to help in smoke-testing.
REM Not really useful to users.

@echo off
echo Make sure to have the env-vars SSID, PASSWORD, STATIC_IP and GATEWAY_IP set.
echo Use CTRL-C to exit an example and start the next one.

set CARGO_PROFILE_RELEASE_OPT_LEVEL=3
echo.
echo Connect ESP32-C3
pause
cargo "+nightly" run --example ble --release --target riscv32imc-unknown-none-elf --features "esp32c3,ble"
cargo "+nightly" run --example dhcp --release --target riscv32imc-unknown-none-elf --features "esp32c3,embedded-svc,wifi"
cargo "+nightly" run --example static_ip --release --target riscv32imc-unknown-none-elf --features "esp32c3,embedded-svc,wifi"
cargo "+nightly" run --example coex --release --target riscv32imc-unknown-none-elf --features "esp32c3,embedded-svc,wifi,ble"

set CARGO_PROFILE_RELEASE_OPT_LEVEL=3
echo.
echo Connect ESP32
pause
cargo "+esp" run --example ble --release --target xtensa-esp32-none-elf --features "esp32,ble"
cargo "+esp" run --example dhcp --release --target xtensa-esp32-none-elf --features "esp32,embedded-svc,wifi"
cargo "+esp" run --example static_ip --release --target xtensa-esp32-none-elf --features "esp32,embedded-svc,wifi"

set CARGO_PROFILE_RELEASE_OPT_LEVEL=3
echo.
echo Connect ESP32-S3
pause
cargo "+esp" run --example ble --release --target xtensa-esp32s3-none-elf --features "esp32s3,ble"
cargo "+esp" run --example dhcp --release --target xtensa-esp32s3-none-elf --features "esp32s3,embedded-svc,wifi"
cargo "+esp" run --example static_ip --release --target xtensa-esp32s3-none-elf --features "esp32s3,embedded-svc,wifi"
cargo "+esp" run --example coex --release --target xtensa-esp32s3-none-elf --features "esp32s3,embedded-svc,wifi,ble"

set CARGO_PROFILE_RELEASE_OPT_LEVEL=2
echo.
echo Connect ESP32-S2
pause
cargo "+esp" run --example dhcp --release --target xtensa-esp32s2-none-elf --features "esp32s2,embedded-svc,wifi"
cargo "+esp" run --example static_ip --release --target xtensa-esp32s2-none-elf --features "esp32s2,embedded-svc,wifi"

set CARGO_PROFILE_RELEASE_OPT_LEVEL=3
echo.
echo Connect ESP32-C2 and modify the 'target.riscv32imc-unknown-none-elf.dev-dependencies' section
pause
cargo "+nightly" run --example ble --release --target riscv32imc-unknown-none-elf --features "esp32c2,ble"
cargo "+nightly" run --example dhcp --release --target riscv32imc-unknown-none-elf --features "esp32c2,embedded-svc,wifi"
cargo "+nightly" run --example static_ip --release --target riscv32imc-unknown-none-elf --features "esp32c2,embedded-svc,wifi"
