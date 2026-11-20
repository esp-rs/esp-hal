# SDIO host and slave

These two examples exchange one packet between an ESP32-P4 host and an ESP32-C6
slave. They demonstrate the existing SDMMC host API and the native SDIO slave
DMA and async API. The C6 receives 64 bytes and echoes the same bytes to the P4.
The P4 checks the complete payload.

The reference wiring is the **Espressif ESP32-P4X-Function-EV-Board**, with its
on-board ESP32-C6-MINI-1. The **Waveshare ESP32-P4-WIFI6-POE-ETH** uses the same
SDIO and C6-enable GPIO mapping. Both use the code below without a board-name
feature or a separate driver implementation. Check the schematic for your board
revision before using the examples with another board. The default P4 build
requires chip revision v3.0 or later; the older P4-Function-EV-Board with a
v1.x chip is not the default target of this example. Verify the chip revision on
either board.

These examples have been compiled but have not been run on either board.

## Wiring

The following connections are already on the two boards listed above:

| Signal | P4 GPIO | C6 GPIO/pin |
| --- | --- | --- |
| CLK | 18 | 19 |
| CMD | 19 | 18 |
| DATA0 | 14 | 20 |
| DATA1 | 15 | 21 |
| DATA2 | 16 | 22 |
| DATA3 | 17 | 23 |
| Slave enable/reset | 54 | EN |

The host uses **slot 1**, routed through the GPIO matrix, at 400 kHz in 1-bit
mode. This is the C6 connection, not the board's microSD-card slot. The C6 SDIO
pads are fixed by its IO MUX. For another layout, change the P4 pin bindings and
the enable GPIO in `sdio_host/src/main.rs`. Use 3.3 V signalling and the required
external pull-ups on CMD and DATA0..3.

Wiring references:

- [Espressif ESP32-P4X-Function-EV-Board user guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4x-function-ev-board/user_guide.html).
- [Espressif reference design](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32p4/_downloads/3ccd3fcf4b943035f2313b42bcff07e7/ESP32-P4X-Function-EV-Board-EN.zip), schematic `SCH_ESP32-P4X_FUNCTION_EV_BOARD_V1.7_20260203.pdf`, P4 connections and C6 module sheets.
- [Waveshare ESP32-P4-WIFI6-POE-ETH schematic](https://files.waveshare.com/wiki/ESP32-P4-WIFI6-POE-ETH/ESP32-P4-WIFI6-POE-ETH-Schematic.pdf), including the GPIO54-to-EN link.

## Build

From the repository root:

```sh
cargo xtask build sdio_host esp32p4
cargo xtask build sdio_slave esp32c6
```

Each directory is a standalone Cargo project, with a feature for its supported
chip. These build commands do not flash either MCU.

## Tests

The host tests exercise descriptor ownership, state transitions, completion
events, and CMD5 voltage selection without hardware:

```sh
cargo xtask host-tests esp-hal-host-tests
```

The `sdio_slave` HIL test runs on one ESP32-C6 through the repository's normal
debug-probe runner. It needs no SDIO host, second MCU, or additional wiring.
It checks bus configuration, high speed advertisement, and storage recovery
after DMA shutdown. Keep an external SDIO host disconnected or held in reset.

```sh
cargo xtask build tests sdio_slave esp32c6
cargo xtask test esp32c6 sdio_slave
```

The second command programs and runs the HIL test. It does not exercise packet
exchange with a peer; use the paired examples for that.

## Use

Both MCUs need their companion Rust firmware. The factory ESP-Hosted C6 image
does not implement this echo example. This example exchanges SDIO data; it does
not provide a remote Wi-Fi service.

The C6 programming interface and the console connectors depend on the board
revision. Use each board's documentation to program the two MCUs. Keep the P4
in reset while programming the C6 so its GPIO54 does not drive the C6's EN pin.

The P4 uses `esp-println`'s automatic console selection. It also initializes
UART0 TX on GPIO37 at 115200 baud for the UART fallback. The C6 prints through
UART0 TX on GPIO16 at 115200 baud. A UART adapter can read the C6 console through
its programming header. The C6 console is not forwarded through the P4.

Start the P4 host after both images are installed. It holds C6 EN low while
configuring the host, then releases it and waits for the slave firmware to boot.
It sends the 80 initialization clocks, enumerates the C6, enables function 1, and
waits for readiness and a receive credit. It writes one packet with CMD53, waits
for IOR1, and polls the cumulative packet-length register until the echo is
available. A FUNCTION_NUMBER refusal while polling means the slave has not
declared function 1 ready again yet. It reads the echo with CMD53 and checks all
64 bytes.

The C6 waits for the packet with `wait_received()`, validates it and explicitly
recovers the descriptor and buffer with `finish_receive()`. It hands those same
allocations to `arm_transmit()` and waits with `wait_transmitted()`. It then calls
`finish_transmit()`. This shows a direction change without disconnecting the
bus pads. The C6 leaves the pads routed after normal completion so the SDIO card
can finish bus signalling.

Expected output includes:

```text
P4: Sent 64 bytes; waiting for the C6 echo
C6: C6 received and checked 64 bytes
C6: C6 completed 64 outbound DMA bytes; check the P4 payload result
P4: P4 received and checked all 64 echoed bytes
```

The `P4:` and `C6:` prefixes above identify the two consoles; they are not part
of the firmware's printed lines. The C6 DMA result alone does not prove that P4
validated the payload.

Transfer polling and async completion waits have bounds. If a C6 async wait times
out, the firmware explicitly requests DMA shutdown; cancelling a future alone only masks
notifications. A shutdown timeout retains the storage. Neither MCU retries a
failed packet. Restart the session to retry: both cumulative counters start from
the fresh C6 reset, and this example does not implement reconnect recovery.

Stopping either direction before validated completion puts the driver in
`SessionInvalidated`. Storage can be reclaimed after confirmed stop, but
`enable_bus()` refuses to reopen that instance. Drop it, reset controller and
host accounting, and create a new session. This also applies to an unused receive
credit; shutdown does not retract it. Cancelling only an async wait does not
invalidate the session: the same armed transfer can still be awaited again.
