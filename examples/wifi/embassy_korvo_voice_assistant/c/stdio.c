// The standard streams of picolibc, which an application defines. ESP-SR only writes to them, and
// `src/idf.rs` prints what it writes.
#include <stdio.h>

int esp_sr_putc(char c, FILE *stream);

static FILE stream = FDEV_SETUP_STREAM(esp_sr_putc, NULL, NULL, _FDEV_SETUP_WRITE);

FILE *stdin = &stream;
FILE *stdout = &stream;
FILE *stderr = &stream;
