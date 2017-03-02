# Cepton SDK
Welcome to Cepton SDK distribution! Current version of SDK is V0.2 (beta)
## Release Notes
### Version 0.2 (beta) 2017-02-28
* Suport capture/replay, with sample code.
* Add win64_debug library.

### Version 0.1 (beta) 2017-02-25
* Initial release

## To setup repository
Do this to fetch the code and all its dependencies
```
git clone --recursive git@github.com:ceptontech/cepton_sdk_redist.git
cd cepton_sdk_redist
```

For WINDOWS
```
mkdir build_win64
cd build_win64
cmake -G "Visual Studio 14 Win64" ..
start build_win64\cepton_sdk_redist.sln
```

For Linux Makefile based
```
mkdir build_linux
cd build_linux
cmake ..
make
```


## SDK Interactions
To start interacting with a Cepton sensor, call ```cepton_sdk_initialize``` with a on_event callback. The main code:
```C
int main(int argc, char ** argv) {
  // Setup to listen for sensor output
  cepton_sdk_initialize(CEPTON_SDK_VERSION, 0, on_event);
  
  // Do other work or just wait until done with sensor
  ...

  // Teardown connections etc.
  cepton_sdk_deinitialize();

  return 1;
}
```

```on_event``` looks like this, here we initialize for newly detected sensors and listen to the data coming from this sensor:

```C

void on_event(int error_code, CeptonSensorHandle sensor,
  struct CeptonSensorInformation const *p_info, int event)
{
  if (error_code < 0)
    return; // Handle error here

  switch(event) {
    case CEPTON_EVENT_ATTACH:
      printf("Attached\n");
	  // Do stuff for newly detected sensor
	  ...
      break;
    case CEPTON_EVENT_FRAME:
      printf("New Frame\n");
      break;

	...

```

## SDK Reference
```C
#include <cepton_sdk.h>
```
### State/service management
```C
int cepton_sdk_initialize(int ver, unsigned flags, FpCeptonSensorEventCallback cb);
```
Allocates buffers, make connections, launch threads etc.
* NOTE: flags is reserved and must be 0 for now.
```C
int cepton_sdk_deinitialize();
```
Deallocation

### Sensor Information
```C
int cepton_sdk_get_number_of_sensors();
struct CeptonSensorInformation const *cepton_sdk_get_sensor_information(CeptonSensorHandle h);
struct CeptonSensorInformation const *cepton_sdk_get_sensor_information_by_index(int sensor_index);
```

### Set Callbacks to listen for frames/scanlines
```C
struct CeptonSensorPoint {
  uint64_t timestamp;  // Microseconds since last successful cepton_sdk_initialize()
  float x, y, z;       // These measurements in meters
  float intensity;     // 0-1 range
};
```

```C
typedef void (*FpCeptonSensorDataCallback)(int error_code, 
  CeptonSensorHandle sensor, size_t n_points, 
  struct CeptonSensorPoint const *p_points);
```

```C
int cepton_sdk_listen_frames(FpCeptonSensorDataCallback cb);
```
Set callback triggered at each frame change

```
int cepton_sdk_unlisten_frames(FpCeptonSensorDataCallback cb);
```
Remove callback previously set by cepton_sdk_listen_frames

```C
int cepton_sdk_listen_scanlines(FpCeptonSensorDataCallback cb);
```
Set callback triggered at completion of each scanline

```C
int cepton_sdk_unlisten_scanlines(FpCeptonSensorDataCallback cb);
```
Remove callback previously set by cepton_sdk_listen_scanlines

### Sensor Calibration
Each sensor has several calibration parameters which can be explicity set using a ```CeptonSensorCalibration``` structure.  
* See ```cepton_sdk.h``` for possible values. 
* Distances are in meters. 
* Angles are in radians.
```C
int cepton_sdk_set_calibration(CeptonSensorHandle h, 
  struct CeptonSensorCalibration const *cal);
```

### Networking
```C
void cepton_sdk_listen_network_packet(FpCeptonNetworkReceiveCb cb);
```
Set a callback to listen for network packets

```C
typedef void(*FpCeptonNetworkReceiveCb)(int error_code, uint64_t ipv4_address, 
  uint8_t const *mac, uint8_t const *buffer, size_t size);
void cepton_sdk_mock_network_receive(uint64_t ipv4_address, uint8_t const *mac, 
  uint8_t const *buffer, size_t size);
```
Cause a network packet to be received as though from the adapter


### SDK notes / FAQ
* A very common problem is firewall blocking UDP broadcast packets coming from the device. Make sure to check that first when there is no connection.
* Intensity output is set to 1.0 right now. Intensity will be supported very shortly.
* Sensor detection and reporting of the first data can be slightly delayed (up to 200ms) if the SDK needs to discover calibration first. This will not happen in production level devices.
