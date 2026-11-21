# bme280-probe

Host-side check for the driver in this repository. It opens `/dev/i2c-N`,
verifies the chip id, performs a single forced measurement and prints the
compensated values as JSON:

```
$ ./bme280_probe.py --bus /dev/i2c-1 --addr 0x76
{"humidity_mrh": 41367, "press_pa": 98140, "temp_mdeg_c": 25080}
```

The compensation is a line-by-line port of `bme280.c`, so a reading from this
tool and a reading from firmware on the same board and calibration block must
agree. Use it before flashing a build that touches the compensation code.

## Requirements

Python 3.8+, no packages. The user needs read/write on the bus:

```
sudo usermod -aG i2c $USER   # then log out and back in
```

## Simulation

`--simulate` skips the bus and runs the datasheet example vector
(adc_T 519888, adc_P 415148, adc_H 27554). Expected output is 25080 mdeg C,
98140 Pa and 41367 mRH; any drift means a formula or scaling change broke the
driver:

```
$ ./bme280_probe.py --simulate
```

## Notes

- `--addr` accepts decimal or hex; 0x76 is the SDO-low address.
- Wire CS to 3V3 first, the part answers on I2C only in that state.
- JSON keys are integers: milli degrees Celsius, pascals, milli percent RH.
