# Migration Guide from v1.0.0-beta.0 to ?

## GPIO changes

### Interconnect types now have a lifetime

The GPIO interconnect types now have a lifetime. This lifetime prevents them to be live for longer
than the GPIO that was used to create them.

The affected types in the `gpio::interconnect` module are:

- `InputSignal`
- `OutputSignal`
- `InputConnection`
- `OutputConnection`

### Pin drivers no longer implement `Peripheral`

Peripheral drivers now take `impl PeripheralInput` and `impl PeripheralOutput` directly. These
traits are now implemented for GPIO pins (stably) and driver structs (unstably) alike.

This change means it's no longer possible to pass a reference to a GPIO driver to a peripheral
driver. For example, it's no longer possible to pass an `&mut Input` to `Spi::with_miso`.
