$builds = @(
  @{ Chip = "esp32";   Target = "xtensa-esp32-none-elf" },
  @{ Chip = "esp32s2"; Target = "xtensa-esp32s2-none-elf" },
  @{ Chip = "esp32s3"; Target = "xtensa-esp32s3-none-elf" },
  @{ Chip = "esp32c2"; Target = "riscv32imc-unknown-none-elf" },
  @{ Chip = "esp32c3"; Target = "riscv32imc-unknown-none-elf" },
  @{ Chip = "esp32c5"; Target = "riscv32imac-unknown-none-elf" },
  @{ Chip = "esp32c6"; Target = "riscv32imac-unknown-none-elf" },
  @{ Chip = "esp32h2"; Target = "riscv32imac-unknown-none-elf" }
)

foreach ($b in $builds) {
  cargo +esp build `
    --manifest-path esp-hal/Cargo.toml `
    --features "$($b.Chip),unstable" `
    --target $b.Target `
    -Zbuild-std

  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}