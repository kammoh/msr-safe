<<<<<<< HEAD
#!/bin/bash
P=$(dirname $0)
echo 1 | sudo tee /sys/bus/event_source/devices/cpu/rdpmc
echo 2 | sudo tee /sys/bus/event_source/devices/cpu/rdpmc
sudo insmod ${P}/msr-pulsar.ko
=======
#!/bin/sh
sudo insmod ./msr-pulsar.ko
>>>>>>> 1af904a5344ddde1e1a380623162a471b6b29056
sudo chmod a+rw /dev/cpu/*/pulsar_msr
sudo chmod a+rw /dev/cpu/msr_batch
