#!/bin/bash
P=$(dirname $0)
echo 1 | sudo tee /sys/bus/event_source/devices/cpu/rdpmc
echo 2 | sudo tee /sys/bus/event_source/devices/cpu/rdpmc
sudo insmod ${P}/msr-pulsar.ko
sudo chmod a+rw /dev/cpu/*/pulsar_msr
sudo chmod a+rw /dev/cpu/msr_batch
