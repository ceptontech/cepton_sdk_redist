# Cepton Driveworks: WIP 

## TIPS and FAQs:
### Networking on Pegasaus: 
* Everything must be sent to the bridge (br0)
* Make sure br0 has the proper IPV4 address and netmask.

### Running the sample_lidar_replay
*  Command (notice sudo is needed)
```sh  
   cd /usr/local/driveworks/bin
   sudo ifconfig br0 192.168.0.11 netmask 255.255.0.0
   sudo ./sample_lidar_replay --protocol=lidar.socket --params=device=CUSTOM,ip=192.168.65.198,port=8808,decoder=/home/nvidia/tmp/libcepton_driveworks.so,scan-frequency=10,protocol=udp
```

