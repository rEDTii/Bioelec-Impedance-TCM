#!/bin/sh
# 调试用脚本，开启数据缓冲区，用于测试和调试

DEVICE="/sys/bus/iio/devices/iio:device1"

echo 1 > $DEVICE/scan_elements/in_current0_en
echo 1 > $DEVICE/scan_elements/in_current1_en
echo 1 > $DEVICE/scan_elements/in_voltage0_en
echo 1 > $DEVICE/scan_elements/in_voltage1_en
echo 1 > $DEVICE/buffer/enable