#!/bin/sh
# 调试用脚本，测试CPU占用 (兼容busybox)
# 采样10秒，输出内核CPU占用

echo "采样10秒CPU数据 请勿操作..."

# 第一次读取数据
cpu1=$(awk '/^cpu /{print $2,$3,$4,$5}' /proc/stat)
sleep 10
# 第二次读取数据
cpu2=$(awk '/^cpu /{print $2,$3,$4,$5}' /proc/stat)

# 计算差值 (busybox awk 直接计算，最稳定)
echo "$cpu1 $cpu2" | awk '{
    # 第一次累计值
    u1=$1; n1=$2; s1=$3; i1=$4;
    # 第二次累计值
    u2=$5; n2=$6; s2=$7; i2=$8;

    # 总时间片
    total = (u2+n2+s2+i2) - (u1+n1+s1+i1);
    # 内核态CPU时间
    sys = s2 - s1;
    # 总CPU占用
    usage = 100 - ( (i2-i1)*100/total );

    print "====================================="
    print "内核态CPU占用(sys): " (sys*100/total) "%"
    print "总CPU占用: " usage "%"
    print "====================================="
}'