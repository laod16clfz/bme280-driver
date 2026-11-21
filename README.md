# bme280-driver

Interrupt-free I2C driver for the Bosch BME280 temperature, pressure and
humidity sensor, written for Cortex-M targets. It is a single translation
unit with no dynamic allocation, no globals and no RTOS dependency: the
caller hands in bus and delay callbacks, so the same code runs on bare
metal, Zephyr or a Linux `i2c-dev` shim.

Why another driver: most public BME280 code pulls in a framework or assumes a
particular HAL. This one only depends on three function pointers, which makes
it easy to audit and easy to port.

## Wiring

| Board pin | Sensor pin | Note |
|-----------|------------|------|
| 3V3       | VIN        | 1.71 V - 3.6 V, do not use 5 V |
| GND       | GND        | |
| P0.26     | SCL        | 400 kHz max, 4.7 kOhm pull-up to 3V3 |
| P0.27     | SDA        | 4.7 kOhm pull-up to 3V3 |
| 3V3       | CS         | tie high to select I2C |
| 3V3       | SDO        | optional: tie low for address 0x76, high for 0x77 |

## Power

From the datasheet (table 1): 350 uA typical while measuring all three
channels at x1 oversampling, 0.1 uA in sleep. The driver leaves the part in
sleep and uses forced mode, so the average draw is set by the sample period,
not by the chip.

## Install

Vendor the two files:

```
cp bme280.c bme280.h path/to/project/drivers/
```

Then implement the callbacks:

```c
static int my_read(void *ctx, uint8_t reg, uint8_t *buf, size_t len);
static int my_write(void *ctx, uint8_t reg, const uint8_t *buf, size_t len);
static void my_delay(void *ctx, uint32_t ms);

struct bme280 dev;
err = bme280_init(&dev, BME280_I2C_ADDR_PRIMARY, ctx,
                  my_read, my_write, my_delay);
```

`examples/main.c` shows the nRF52 TWI0 bring-up, including the register-write
frame the sensor expects.

## Usage

```c
struct bme280_sample s;
if (bme280_read(&dev, &s) == BME280_OK) {
        /* s.temp_mdeg_c, s.press_pa, s.humidity_mrh */
}
```

`bme280_read()` blocks for the worst-case conversion time (about 10 ms at x1
oversampling) and is therefore safe to call from a thread but not from an ISR.
For DMA users, `bme280_compensate()` takes an already-read 0xF7..0xFE burst
and performs the same integer compensation.

Units are integers on purpose: milli degrees Celsius, pascals and milli
percent RH. No `float` is used, so the driver links without a soft-float ABI.

## Layout

```
bme280.h             API, register map, error enum
bme280.c             init, forced-mode read, datasheet 4.2.3 compensation
examples/main.c      nRF52 DK example on TWI0
tools/bme280-probe/  host-side i2c-dev probe (feature branch)
```

## Errata and limits

- Chip id 0x58 is a BMP280. It has no humidity channel; `bme280_init()` will
  reject it with `BME280_ECHIP` rather than return bogus RH.
- Compensated pressure is only valid between 300 and 1100 hPa (datasheet 1.1).
- The humidity channel has hysteresis after long exposure above 80 %RH;
  recondition per datasheet 1.2 before trusting a dry reading after that.
- Oversampling above x2 heats the die; keep x1 unless the noise budget really
  needs more, and calibrate the offset in the enclosure.
- A soft reset is issued on every `bme280_init()`; wait the 3 ms startup time
  before the next bus access, as the driver does.

## License

MIT, see LICENSE.
