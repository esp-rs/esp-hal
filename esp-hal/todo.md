## SDIO device driver

- reference esp-idf SDIO "slave" component driver:
  - <esp-idf/components/esp_driver_sdio/src/sdio_slave.c>

- apply remaining fixes from review
  - re-read review for any additional fixes
  - convert `OutputPin` + `Pin` into `PeripheralOutput`
    - replace `Output` + `Flex` with `OutputSignal` (covers input + output)
