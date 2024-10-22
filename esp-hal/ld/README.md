# ROM functions

Files in the `rom` subdirectories are taken from esp-idf

- DON'T include any `*newlib*` functions
- systimer, wdt and mbedtls shouldn't be included
- make sure to align the version you take the files from with esp-wifi-sys - NEVER randomly sync the files with other versions
- some additional functions are needed from ROM - see `additional.ld` (these are usually defined in the `*newlib*` files)
