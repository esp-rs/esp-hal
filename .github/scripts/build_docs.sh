#!/usr/bin/env bash

set -euo pipefail

# Extract the package version from `esp-hal` (using arcane methods):
PKG_VERSION=$(
  cargo metadata --format-version=1 --no-deps --manifest-path=esp-hal/Cargo.toml \
  | jq -r '.packages[] | select(.name=="esp-hal") | .version'
)

# Build the documentation for each supported cheap, namespacing by
# package version and chip:
CHIPS=("esp32" "esp32c2" "esp32c3" "esp32c6" "esp32h2" "esp32p4" "esp32s2" "esp32s3")

for CHIP in "${CHIPS[@]}"; do
  cargo xtask build-documentation \
    --output-path="docs/esp-hal/$PKG_VERSION"/"$CHIP"/ \
    esp-hal \
    "$CHIP"
done

# Copy any additional resources (such as the index and our logo)
# to the location of the built documentation as well:
cp resources/esp-rs.svg docs/
cp resources/index.html docs/
