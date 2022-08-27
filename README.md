# `adafruit-clue`
Board support for Adafruit CLUE:

https://www.adafruit.com/product/4500

Still very much a work in progress, but there's enough up and running
to be useful. Comments and pull requests welcome.

## Examples
A few simple examples are included as a starting point.

### blink.rs
Alternates blinking the white and red LEDs.

## Building and Flashing an Example

I've been using the Microsoft UF2 tools:

https://github.com/Microsoft/uf2

to generate a file compatible with the UF2 bootloader.
Steps for building/flashing the ```blink``` example:

1. ```cargo build --release --example blink```
2. ```cargo objcopy --release --example blink -- -O binary blink.bin```
3. ```uf2conv.py blink.bin -c -f 0xADA52840 -b 0x26000 -o blink.uf2```
4. ```cp blink.uf2 <path where your Clue is mounted>```

I've also included a script (```exampletouf2.sh```) that performs steps 2
and 3 on a named example:

```
exampletouf2.sh blink
```