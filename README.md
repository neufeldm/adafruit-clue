# `adafruit-clue`
Board support for Adafruit CLUE:

https://www.adafruit.com/product/4500

Still very much a work in progress, but there's enough up and running
to be useful. Comments and pull requests welcome.

## Initial Setup
Since the s140 v6.x and v7.x SoftDevices require different memory layouts,
you'll need to specify either nrf-s140-v6 or nrf-s140-v7 features in your
Cargo.toml.

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

In step 3 use ```0x26000``` if your Clue has version 6.X.X of the NRF
SoftDevice, or ```0x27000``` if your Clue has version 7.X.X of the NRF
SoftDevice. If you're using the stock Adafruit Clue then you've got 6.X.X
and should use 0x26000. You'll only have 7.X.X if you've explicitly built
and flashed a custom bootloader containing it.

3. ```uf2conv.py blink.bin -c -f 0xADA52840 -b <0x26000 | 0x27000> -o blink.uf2```
4. ```cp blink.uf2 <path where your Clue is mounted>```

I've also included scripts (```exampletouf2-s140_v6.sh```) and 
(```exampletouf2-s140_v7.sh```) that performs steps 2
and 3 on a named example. Use whichever script matches your SoftDevice version.

```
exampletouf2-s140-v6.sh blink
```