/*
 * bme280.c - see bme280.h for the contract.
 *
 * Register sequence follows datasheet 5.4.1 (reset), 5.4.6 (forced mode) and
 * 4.2.3 (integer compensation). No dynamic allocation, no globals.
 */
#include "bme280.h"

#define BME280_CALIB0_LEN  26u
#define BME280_CALIB26_LEN 7u
#define BME280_STARTUP_MS  3u  /* t_startup, datasheet table 1 */
#define BME280_CONV_MS     10u /* worst case, osrs x1 on all channels */
#define BME280_POLL_MS     2u
#define BME280_POLL_TRIES  50u

static uint16_t le16(const uint8_t *p)
{
	return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static int dev_read(struct bme280 *dev, uint8_t reg, uint8_t *buf, size_t len)
{
	return dev->read(dev->ctx, reg, buf, len) == 0 ? BME280_OK : BME280_EIO;
}

static int dev_write_byte(struct bme280 *dev, uint8_t reg, uint8_t value)
{
	return dev->write(dev->ctx, reg, &value, 1) == 0 ? BME280_OK : BME280_EIO;
}

static void dev_delay(struct bme280 *dev, uint32_t ms)
{
	if (dev->delay)
		dev->delay(dev->ctx, ms);
}

static int load_calib(struct bme280 *dev)
{
	uint8_t b0[BME280_CALIB0_LEN];
	uint8_t b1[BME280_CALIB26_LEN];
	uint32_t sum = 0;
	int err;

	err = dev_read(dev, BME280_REG_CALIB0, b0, sizeof b0);
	if (err)
		return err;
	err = dev_read(dev, BME280_REG_CALIB26, b1, sizeof b1);
	if (err)
		return err;

	for (size_t i = 0; i < sizeof b0; i++)
		sum += b0[i];
	for (size_t i = 0; i < sizeof b1; i++)
		sum += b1[i];
	if (sum == 0u)
		return BME280_ECALIB;

	dev->calib.t1 = (uint16_t)(b0[0] | ((uint16_t)b0[1] << 8));
	dev->calib.t2 = (int16_t)(b0[2] | ((uint16_t)b0[3] << 8));
	dev->calib.t3 = (int16_t)(b0[4] | ((uint16_t)b0[5] << 8));
	dev->calib.p1 = le16(&b0[6]);
	dev->calib.p2 = (int16_t)le16(&b0[8]);
	dev->calib.p3 = (int16_t)le16(&b0[10]);
	dev->calib.p4 = (int16_t)le16(&b0[12]);
	dev->calib.p5 = (int16_t)le16(&b0[14]);
	dev->calib.p6 = (int16_t)le16(&b0[16]);
	dev->calib.p7 = (int16_t)le16(&b0[18]);
	dev->calib.p8 = (int16_t)le16(&b0[20]);
	dev->calib.p9 = (int16_t)le16(&b0[22]);
	dev->calib.h1 = b0[25];

	dev->calib.h2 = (int16_t)le16(&b1[0]);
	dev->calib.h3 = b1[2];
	/* H4/H5 are 12-bit, big-endian across 0xE4/0xE5 and 0xE5/0xE6. */
	dev->calib.h4 = (int16_t)((b1[3] << 4) | (b1[4] & 0x0Fu));
	if (dev->calib.h4 & 0x800)
		dev->calib.h4 -= 0x1000;
	dev->calib.h5 = (int16_t)((b1[5] << 4) | (b1[4] >> 4));
	if (dev->calib.h5 & 0x800)
		dev->calib.h5 -= 0x1000;
	dev->calib.h6 = (int8_t)b1[6];

	if (dev->calib.p1 == 0u || dev->calib.h1 == 0u)
		return BME280_ECALIB;
	return BME280_OK;
}

int bme280_init(struct bme280 *dev, uint8_t addr, void *ctx,
		bme280_read_fn read, bme280_write_fn write,
		bme280_delay_fn delay)
{
	uint8_t chip_id = 0;
	int err;

	if (!dev || !read || !write)
		return BME280_EPARAM;

	dev->addr = addr;
	dev->ctx = ctx;
	dev->read = read;
	dev->write = write;
	dev->delay = delay;
	dev->t_fine = 0;

	err = dev_read(dev, BME280_REG_CHIP_ID, &chip_id, 1);
	if (err)
		return err;
	if (chip_id != BME280_CHIP_ID)
		return BME280_ECHIP;

	err = dev_write_byte(dev, BME280_REG_RESET, BME280_RESET_CMD);
	if (err)
		return err;
	dev_delay(dev, BME280_STARTUP_MS);

	err = load_calib(dev);
	if (err)
		return err;

	/* Filter off, 1000 ms standby; irrelevant in forced mode. */
	err = dev_write_byte(dev, BME280_REG_CONFIG, 0x00u);
	if (err)
		return err;
	err = dev_write_byte(dev, BME280_REG_CTRL_HUM, 0x01u);
	if (err)
		return err;

	return dev_write_byte(dev, BME280_REG_CTRL_MEAS, 0x00u);
}

static int32_t compensate_temp(struct bme280 *dev, int32_t adc_t)
{
	int32_t var1, var2;

	var1 = ((((adc_t >> 3) - ((int32_t)dev->calib.t1 << 1))) *
		((int32_t)dev->calib.t2)) >> 11;
	var2 = (((((adc_t >> 4) - (int32_t)dev->calib.t1) *
		  ((adc_t >> 4) - (int32_t)dev->calib.t1)) >> 12) *
		((int32_t)dev->calib.t3)) >> 14;
	dev->t_fine = var1 + var2;
	return (dev->t_fine * 5 + 128) >> 8; /* 0.01 degC */
}

static uint32_t compensate_press(struct bme280 *dev, int32_t adc_p)
{
	int64_t var1, var2, p;

	var1 = (int64_t)dev->t_fine - 128000;
	var2 = var1 * var1 * (int64_t)dev->calib.p6;
	var2 = var2 + ((var1 * (int64_t)dev->calib.p5) << 17);
	var2 = var2 + ((int64_t)dev->calib.p4 << 35);
	var1 = ((var1 * var1 * (int64_t)dev->calib.p3) >> 8) +
	       (var1 * (int64_t)dev->calib.p2 << 12);
	var1 = ((((int64_t)1 << 47) + var1) * (int64_t)dev->calib.p1) >> 33;
	if (var1 == 0)
		return 0u;

	p = 1048576 - adc_p;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t)dev->calib.p9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t)dev->calib.p8 * p) >> 19;
	p = ((p + var1 + var2) >> 8) + ((int64_t)dev->calib.p7 << 4);

	return (uint32_t)(p >> 8); /* Q24.8 -> Pa */
}

static uint32_t compensate_hum(struct bme280 *dev, int32_t adc_h)
{
	int32_t v;

	v = dev->t_fine - 76800;
	v = (((((adc_h << 14) - ((int32_t)dev->calib.h4 << 20) -
		((int32_t)dev->calib.h5 * v)) + 16384) >> 15) *
	     (((((((v * (int32_t)dev->calib.h6) >> 10) *
		  (((v * (int32_t)dev->calib.h3) >> 11) + 32768)) >> 10) +
		2097152) * (int32_t)dev->calib.h2 + 8192) >> 14));
	v -= ((((v >> 15) * (v >> 15)) >> 7) * (int32_t)dev->calib.h1) >> 4;
	v = v < 0 ? 0 : v;
	v = v > 419430400 ? 419430400 : v;

	return (uint32_t)(((int64_t)v * 1000) >> 22); /* Q22.10 -> milli %RH */
}

int bme280_compensate(struct bme280 *dev, const uint8_t raw[8],
		struct bme280_sample *out)
{
	int32_t adc_p, adc_t, adc_h;

	if (!dev || !raw || !out)
		return BME280_EPARAM;

	adc_p = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) |
			  (raw[2] >> 4));
	adc_t = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) |
			  (raw[5] >> 4));
	adc_h = (int32_t)(((uint32_t)raw[6] << 8) | raw[7]);

	out->temp_mdeg_c = compensate_temp(dev, adc_t) * 10;
	out->press_pa = compensate_press(dev, adc_p);
	out->humidity_mrh = compensate_hum(dev, adc_h);
	return BME280_OK;
}

int bme280_read(struct bme280 *dev, struct bme280_sample *out)
{
	uint8_t raw[8];
	uint8_t status = 0;
	int err;

	if (!dev || !out)
		return BME280_EPARAM;

	/* Forced mode, osrs_t = osrs_p = osrs_h = x1 (datasheet 5.4.6). */
	err = dev_write_byte(dev, BME280_REG_CTRL_HUM, 0x01u);
	if (err)
		return err;
	err = dev_write_byte(dev, BME280_REG_CTRL_MEAS, 0x25u);
	if (err)
		return err;

	dev_delay(dev, BME280_CONV_MS);
	for (unsigned i = 0; i < BME280_POLL_TRIES; i++) {
		err = dev_read(dev, BME280_REG_STATUS, &status, 1);
		if (err)
			return err;
		if (!(status & BME280_STATUS_MEASURING))
			break;
		dev_delay(dev, BME280_POLL_MS);
	}
	if (status & BME280_STATUS_MEASURING)
		return BME280_EIO;

	err = dev_read(dev, BME280_REG_DATA, raw, sizeof raw);
	if (err)
		return err;

	return bme280_compensate(dev, raw, out);
}
