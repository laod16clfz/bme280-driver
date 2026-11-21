/*
 * bme280.h - I2C driver for the Bosch BME280 environmental sensor.
 *
 * The driver owns no bus: the caller supplies read/write/delay callbacks so
 * the same code runs on bare metal, Zephyr, or a Linux i2c-dev shim.
 *
 * Datasheet: BME280 datasheet rev. 1.6, sections 5 (registers), 4.2.3
 * (compensation) and appendix A (calibration layout).
 */
#ifndef BME280_H
#define BME280_H

#include <stddef.h>
#include <stdint.h>

#define BME280_I2C_ADDR_PRIMARY   0x76u
#define BME280_I2C_ADDR_SECONDARY 0x77u

#define BME280_REG_CALIB0  0x88u
#define BME280_REG_CHIP_ID 0xD0u
#define BME280_REG_RESET   0xE0u
#define BME280_REG_CALIB26 0xE1u
#define BME280_REG_CTRL_HUM 0xF2u
#define BME280_REG_STATUS  0xF3u
#define BME280_REG_CTRL_MEAS 0xF4u
#define BME280_REG_CONFIG  0xF5u
#define BME280_REG_DATA    0xF7u

#define BME280_CHIP_ID     0x60u
#define BME280_RESET_CMD   0xB6u
#define BME280_STATUS_MEASURING 0x08u

enum bme280_err {
	BME280_OK     =  0,
	BME280_EIO    = -1, /* callback reported a bus error */
	BME280_ECHIP  = -2, /* chip id mismatch */
	BME280_ECALIB = -3, /* calibration block is all zeroes */
	BME280_EPARAM = -4, /* NULL callback or sample pointer */
};

struct bme280_sample {
	int32_t  temp_mdeg_c;   /* milli degrees Celsius */
	uint32_t press_pa;      /* pascals */
	uint32_t humidity_mrh;  /* milli percent relative humidity */
};

struct bme280_calib {
	uint16_t t1;
	int16_t  t2, t3;
	uint16_t p1;
	int16_t  p2, p3, p4, p5, p6, p7, p8, p9;
	uint8_t  h1;
	int16_t  h2;
	uint8_t  h3;
	int16_t  h4, h5;
	int8_t   h6;
};

typedef int (*bme280_read_fn)(void *ctx, uint8_t reg, uint8_t *buf, size_t len);
typedef int (*bme280_write_fn)(void *ctx, uint8_t reg, const uint8_t *buf, size_t len);
typedef void (*bme280_delay_fn)(void *ctx, uint32_t ms);

struct bme280 {
	uint8_t addr;             /* 7-bit I2C address */
	struct bme280_calib calib;
	int32_t t_fine;
	void *ctx;
	bme280_read_fn read;
	bme280_write_fn write;
	bme280_delay_fn delay;
};

/*
 * Reset the part, verify the chip id and load factory calibration.
 * Returns BME280_OK, or a negative enum bme280_err. Safe to call again.
 */
int bme280_init(struct bme280 *dev, uint8_t addr, void *ctx,
		bme280_read_fn read, bme280_write_fn write,
		bme280_delay_fn delay);

/*
 * Trigger one forced measurement and compensate it.
 * Blocking: waits for the datasheet worst-case conversion time.
 */
int bme280_read(struct bme280 *dev, struct bme280_sample *out);

/* Compensate a raw burst from 0xF7..0xFE. Exposed for DMA-based users. */
int bme280_compensate(struct bme280 *dev, const uint8_t raw[8],
		struct bme280_sample *out);

#endif /* BME280_H */
