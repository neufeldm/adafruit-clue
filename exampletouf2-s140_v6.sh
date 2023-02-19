
#!/bin/sh
if [[ $# -ne 1 ]]
then
  echo "Usage:"
  echo "exampletouf2-s140_v6.sh <name of example to convert to uf2>"
  echo
  echo "Note that you must have 'uf2conv.py' executable in your path."
  exit -1
fi
cargo objcopy --release --example "$1" -- -O binary "$1".bin
uf2conv.py "$1".bin -c -f 0xADA52840 -b 0x26000 -o "$1"-s140_v6.uf2
