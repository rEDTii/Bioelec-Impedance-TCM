#!/bin/sh
# 调试用脚本，关闭数据缓冲区，用于测试和调试

# 定义设备路径变量，便于维护
DEVICE="/sys/bus/iio/devices/iio:device1"

# 关闭数据缓冲区
echo 0 > $DEVICE/buffer/enable