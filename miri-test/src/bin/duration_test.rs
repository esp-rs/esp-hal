#![no_std]
#![no_main]

use miri_test as _;

miri_test::miri_init!();

fn main() {
    let one_hour = esp_hal::time::Duration::from_hours(1);

    assert_eq!(one_hour.as_minutes(), 60);
    assert_eq!(one_hour.as_secs(), 3600);
}
