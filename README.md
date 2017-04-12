# Cepton SDK
Welcome to Cepton SDK distribution! Current version of SDK is v0.4 (beta)
## Table of contents
* [Release Notes](#release-notes)
* [To setup repository](#to-setup-repository)
* [SDK Interactions](#sdk-interactions)
* [SDK Reference](#sdk-reference)
* [Notes/FAQ](#notes--faq)

## Release Notes
### Version 0.4 (beta) 2017-04-12
* !NOTE: coordinate system changed! (to conform to popular world coordinates such as ROS, distance is Y and height is Z starting from SDK version 0.4)
* Support GPS timestamps (require external GPS hookup)
* cepton_sdk_initialize allows RETURN_UNMEASURABLE flag to support a full frame.
* listen_frame do not return partial frames anymore.
* data_exporter: support --split option
* Calibration improvements
* Stability and thread safety improvements

### Version 0.3 (beta) 2017-03-07
* Support Mac OSX starting this version
* Improved support for Linux (esp. Ubuntu 14.04)
* Included pre-built binary of CeptonViewer
* Additional sample for data_exporter to dump points into CSV or binary formats.
* Differentiate between real sensor and a replay of capture.
* Networking change to allow multiple applications to share the same device.

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

### Initialization and callbacks
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

### GPS timestamp support
If your sensor has a GPS module connected through the interface box, point cloud timestamps will be GPS based, and the raw GPS timestamp will also be available through ```CeptonSensorInformation``` structure. 

These are the GPS timestamp in the structure:
```C
  // Note: GPS timestamp reported here is GMT time
  uint8_t gps_ts_year; // e.g. 2017 => 17
  uint8_t gps_ts_month; // 1-12
  uint8_t gps_ts_day; // 1-31
  uint8_t gps_ts_hour; // 0-23
  uint8_t gps_ts_min; // 0-59
  uint8_t gps_ts_sec; // 0-59
```

And flag bits to indicate types of GPS signals we have connected to:
```C
  uint32_t is_pps_connected : 1; // Set if GPS/PPS is available
  uint32_t is_nmea_connected : 1; // Set if GPS/NMEA is available
```

## SDK Reference
```C
#include <cepton_sdk.h>
```
### State/service management
```C
int cepton_sdk_initialize(int version, unsigned flags, FpCeptonSensorEventCallback cb);
```
Allocates buffers, make connections, launch threads etc. Error will be returned if called while SDK is already initialized.
* ```version``` should always be ```CEPTON_SDK_VERSION```, this is a safeguard against linking with the wrong library.
* ```flags``` is a bit field that controls the SDK behavior. Pass ```0``` for default operations.

```C
int cepton_sdk_deinitialize();
```
Deallocation. Will do nothing and return ```CEPTON_ERROR_NOT_INITIALIZED``` if called before ```cepton_sdk_initialize```.

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

```C
int cepton_sdk_unlisten_frames(FpCeptonSensorDataCallback cb);
```
Remove callback previously set by cepton_sdk_listen_frames

```C
int cepton_sdk_listen_scanlines(FpCeptonSensorDataCallback cb);
```
Set callback triggered at completion of a scanline. A scanline contains points in a vertical line. Because of our symmetric design we always return two scanlines at each callback.

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
void cepton_sdk_mock_network_receive(uint64_t ipv4_address, uint8_t const *mac, 
  uint8_t const *buffer, size_t size);
```
Cause a network packet to be received as though from the adapter. Used to replay a capture file (see sample code provided)


## Notes / FAQ
* A very common problem is firewall blocking UDP broadcast packets coming from the device. Make sure to check that first when there is no connection.
* Intensity output is set to 1.0 right now. Intensity will be supported very shortly.
* Sensor detection and reporting of the first data can be slightly delayed (up to 200ms) if the SDK needs to discover calibration first. This will not happen in production level devices.

### Technical notes from the internals
* All the callbacks are invoked from the same network receive thread that gets launched at
the ```cepton_sdk_initialize``` time. It is a good practice to not spend too much time
servicing the callbacks. If you need more than ~1ms to handle the callback, it is probably
time to consider feeding data into a queue and processing them asynchronously. 
* The callbacks are not re-entrant since they come from the same thread.
