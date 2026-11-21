/*
 * main.c - nRF52 DK bring-up sketch for the bme280 driver.
 *
 * Uses the TWI0 peripheral at 400 kHz with P0.26/P0.27 as SCL/SDA. On other
 * boards only the bme280_bus_* callbacks need replacing.
 */
#include <stdint.h>
#include <stdio.h>

#include "bme280.h"

#define TWI_SCL 26u
#define TWI_SDA 27u
#define SENSOR_ADDR BME280_I2C_ADDR_PRIMARY

static void bus_init(void)
{
	/* twi_init(TWI0, TWI_SCL, TWI_SDA, 400000); */
}

static int bus_read(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
	(void)ctx;
	return twi_write_read(TWI0, SENSOR_ADDR, &reg, 1, buf, len);
}

static int bus_write(void *ctx, uint8_t reg, const uint8_t *buf, size_t len)
{
	uint8_t frame[2] = { reg, buf[0] };
	(void)ctx;
	return len == 1 ? twi_write(TWI0, SENSOR_ADDR, frame, 2) : -1;
}

static void bus_delay(void *ctx, uint32_t ms)
{
	(void)ctx;
	k_sleep(K_MSEC(ms));
}

int main(void)
{
	struct bme280 dev;
	struct bme280_sample s;
	int err;

	bus_init();
	err = bme280_init(&dev, SENSOR_ADDR, NULL, bus_read, bus_write, bus_delay);
	if (err) {
		printf("bme280 init failed: %d\n", err);
		return err;
	}

	for (;;) {
		err = bme280_read(&dev, &s);
		if (err) {
			printf("read failed: %d\n", err);
		} else {
			printf("%d.%02u C  %u Pa  %u.%02u %%RH\n",
			       s.temp_mdeg_c / 1000, (unsigned)(s.temp_mdeg_c % 1000) / 10,
			       s.press_pa,
			       s.humidity_mrh / 1000, (unsigned)(s.humidity_mrh % 1000) / 10);
		}
		k_sleep(K_SECONDS(10));
	}
}
