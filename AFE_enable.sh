#!/bin/sh

DEVICE="/sys/bus/iio/devices/iio:device1"

echo 1 > $DEVICE/scan_elements/in_current0_en
echo 1 > $DEVICE/scan_elements/in_current1_en
echo 1 > $DEVICE/scan_elements/in_voltage0_en
echo 1 > $DEVICE/scan_elements/in_voltage1_en
echo 1 > $DEVICE/buffer/enable