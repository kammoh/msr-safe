#!/bin/sh
sudo insmod ./msr-pulsar.ko
sudo chmod a+rw /dev/cpu/*/pulsar_msr
sudo chmod a+rw /dev/cpu/msr_batch
