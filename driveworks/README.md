# Cepton Driveworks

This driver is meant as reference code only. It is completely untested, and is not guaranteed to work.

## Getting started

Run `sample_lidar_replay`

```sh
cd /usr/local/driveworks/bin
sudo ifconfig br0 192.168.0.11 netmask 255.255.0.0
sudo ./sample_lidar_replay --protocol=lidar.socket --params=device=CUSTOM,ip=<lidar_ip_address>,port=8808,decoder=/home/nvidia/tmp/libcepton_driveworks.so,scan-frequency=10,protocol=udp
```