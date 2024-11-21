A small benchmark running on ESP32-S3, measuring the performance of embassy-executor and our time driver.

Connect an ESP32-S3, and run `cargo run --release`

The result is printed as follows:

> INFO - Test OK, count=53495, cycles=22432/100

`count` means the number of times the test task was scheduled. `cycles` is the number of CPU cycles per executor loop.