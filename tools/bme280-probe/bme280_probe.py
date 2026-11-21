#!/usr/bin/env python3
"""bme280-probe: read one compensated sample from a BME280 on Linux i2c-dev.

The tool mirrors the integer compensation in bme280.c so a new build can be
compared against a known-good host reading before firmware is flashed.
Standard library only; needs access to /dev/i2c-N.
"""

import argparse
import fcntl
import json
import os
import sys
import time

I2C_SLAVE = 0x0703
REG_CALIB0 = 0x88
REG_CHIP_ID = 0xD0
REG_RESET = 0xE0
REG_CALIB26 = 0xE1
REG_CTRL_HUM = 0xF2
REG_STATUS = 0xF3
REG_CTRL_MEAS = 0xF4
REG_CONFIG = 0xF5
REG_DATA = 0xF7
CHIP_ID = 0x60
STATUS_MEASURING = 0x08


def open_bus(path, addr):
    fd = os.open(path, os.O_RDWR)
    fcntl.ioctl(fd, I2C_SLAVE, addr)
    return fd


def read_regs(fd, reg, length):
    os.write(fd, bytes([reg]))
    data = os.read(fd, length)
    if len(data) != length:
        raise OSError(f"short read: {reg:#04x}")
    return data


def write_reg(fd, reg, value):
    os.write(fd, bytes([reg, value]))


def load_calib(fd):
    low = read_regs(fd, REG_CALIB0, 26)
    high = read_regs(fd, REG_CALIB26, 7)
    s16 = lambda b, o: int.from_bytes(b[o:o + 2], "little", signed=True)
    u16 = lambda b, o: int.from_bytes(b[o:o + 2], "little")
    h4 = (high[3] << 4) | (high[4] & 0x0F)
    h5 = (high[5] << 4) | (high[4] >> 4)
    return {
        "t1": u16(low, 0), "t2": s16(low, 2), "t3": s16(low, 4),
        "p1": u16(low, 6), "p2": s16(low, 8), "p3": s16(low, 10),
        "p4": s16(low, 12), "p5": s16(low, 14), "p6": s16(low, 16),
        "p7": s16(low, 18), "p8": s16(low, 20), "p9": s16(low, 22),
        "h1": low[25], "h2": s16(high, 0), "h3": high[2],
        "h4": h4 - 0x1000 if h4 & 0x800 else h4,
        "h5": h5 - 0x1000 if h5 & 0x800 else h5,
        "h6": int.from_bytes(high[6:7], "little", signed=True),
    }


def compensate(c, raw):
    adc_p = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4)
    adc_t = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4)
    adc_h = (raw[6] << 8) | raw[7]

    var1 = ((adc_t >> 3) - (c["t1"] << 1)) * c["t2"] >> 11
    var2 = (((adc_t >> 4) - c["t1"]) ** 2 >> 12) * c["t3"] >> 14
    t_fine = var1 + var2
    temp_mdeg_c = ((t_fine * 5 + 128) >> 8) * 10

    var1 = t_fine - 128000
    var2 = var1 * var1 * c["p6"] + (var1 * c["p5"] << 17) + (c["p4"] << 35)
    var1 = ((var1 * var1 * c["p3"] >> 8) + (var1 * c["p2"] << 12))
    var1 = ((1 << 47) + var1) * c["p1"] >> 33
    if var1 == 0:
        raise ValueError("pressure calibration gives zero divisor")
    p = 1048576 - adc_p
    p = (((p << 31) - var2) * 3125) // var1
    var1 = c["p9"] * (p >> 13) * (p >> 13) >> 25
    var2 = c["p8"] * p >> 19
    p = ((p + var1 + var2) >> 8) + (c["p7"] << 4)
    press_pa = p >> 8

    v = t_fine - 76800
    v = (((adc_h << 14) - (c["h4"] << 20) - c["h5"] * v + 16384) >> 15) * (
        ((((v * c["h6"] >> 10) * ((v * c["h3"] >> 11) + 32768)) >> 10)
         + 2097152) * c["h2"] + 8192 >> 14)
    v -= (((v >> 15) * (v >> 15)) >> 7) * c["h1"] >> 4
    v = max(0, min(v, 419430400))
    humidity_mrh = v * 1000 >> 22

    return {"temp_mdeg_c": temp_mdeg_c, "press_pa": press_pa,
            "humidity_mrh": humidity_mrh}


SIM_CALIB = {
    "t1": 27504, "t2": 26435, "t3": -1000, "p1": 37404, "p2": -10685,
    "p3": 3024, "p4": 2855, "p5": 140, "p6": -7, "p7": 15500,
    "p8": -14600, "p9": 6000, "h1": 75, "h2": 362, "h3": 0, "h4": 313,
    "h5": 50, "h6": 30,
}
SIM_RAW = bytes([0x65, 0x5A, 0xCC, 0x7E, 0xED, 0xD0, 0x6B, 0xA2])


def sample(fd, addr):
    chip = read_regs(fd, REG_CHIP_ID, 1)[0]
    if chip != CHIP_ID:
        raise ValueError(f"chip id {chip:#04x} is not a BME280")
    write_reg(fd, REG_RESET, 0xB6)
    time.sleep(0.003)
    calib = load_calib(fd)
    write_reg(fd, REG_CONFIG, 0x00)
    write_reg(fd, REG_CTRL_HUM, 0x01)
    write_reg(fd, REG_CTRL_MEAS, 0x25)
    time.sleep(0.01)
    for _ in range(50):
        if not read_regs(fd, REG_STATUS, 1)[0] & STATUS_MEASURING:
            break
        time.sleep(0.002)
    else:
        raise TimeoutError("measurement never finished")
    return compensate(calib, read_regs(fd, REG_DATA, 8))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bus", default="/dev/i2c-1")
    ap.add_argument("--addr", type=lambda v: int(v, 0), default=0x76)
    ap.add_argument("--simulate", action="store_true",
                    help="use datasheet example values, no hardware")
    args = ap.parse_args()

    if args.simulate:
        result = compensate(SIM_CALIB, SIM_RAW)
    else:
        fd = open_bus(args.bus, args.addr)
        try:
            result = sample(fd, args.addr)
        finally:
            os.close(fd)
    json.dump(result, sys.stdout, sort_keys=True)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
