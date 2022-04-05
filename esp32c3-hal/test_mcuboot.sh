#!/usr/bin/env bash
#
#  Copyright (c) 2022 Espressif Systems (Shanghai) Co., Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#

SCRIPT_NAME=$(basename "${BASH_SOURCE[0]}")

set -eo pipefail

usage() {
  echo ""
  echo "USAGE: ${SCRIPT_NAME} <ELF>"
  echo ""
}

if [ -z "${1}" ]; then
  echo "ERROR: Missing application ELF file."
  usage
  exit 1
fi

# MCUboot's tool for image signing and key management
if ! command -v imgtool &> /dev/null; then
  echo ""
  echo "imgtool not found. Please run: \"pip install imgtool\""
  echo ""
  exit 1
fi

elf_file=${1}

objcopy -O ihex "${elf_file}" app.hex
imgtool sign --pad --align 4 -v 0 -s auto -H 32 --pad-header -S 0x100000 app.hex app.bin
esptool.py -c esp32c3 -p /dev/ttyUSB0 -b 921600 --after no_reset write_flash -fs 4MB -fm dio -ff 40m 0x0 ./mcuboot-esp32c3.bin 0x110000 ./app.bin
picocom -b 115200 /dev/ttyUSB0 --imap lfcrlf
