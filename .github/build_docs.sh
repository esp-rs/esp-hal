#!/bin/bash

WORKSPACE=${GITHUB_WORKSPACE:-$(pwd)}

CHIPS=("esp32" "esp32c2" "esp32c3" "esp32c6" "esp32h2" "esp32p4" "esp32s2" "esp32s3")
for CHIP in "${CHIPS[@]}"; do
  if [ "$CHIP" == "esp32" ] || [ "$CHIP" == "esp32s2" ] || [ "$CHIP" == "esp32s3" ]; then
    cargo +esp xtask build-documentation --output-path=docs/"$CHIP"/ esp-hal "$CHIP"
  else
    cargo +nightly xtask build-documentation --output-path=docs/"$CHIP"/ esp-hal "$CHIP"
  fi
done

cp resources/esp-rs.svg "${WORKSPACE}"
cp resources/index.html "${WORKSPACE}"

find . -maxdepth 1 -mindepth 1 ! -name 'docs' ! -name '.git' ! -name 'index.html' ! -name 'esp-rs.svg' ! -name '.' -exec rm -rf {} +