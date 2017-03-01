# Cepton SDK
Welcome to Cepton SDK distribution! Current version of SDK is V0.1 (beta)
## Release Notes
### Version 0.2 (beta)
* Suport capture/replay, with sample code.
* Add win64_debug library.

### Version 0.1 (beta)
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
TODO, please see cepton_sdk.h for details.


### SDK notes / FAQ
* A very common problem is firewall blocking UDP broadcast packets coming from the device. Make sure to check that first when there is no connection.
* Intensity output is set to 1.0 right now. Intensity will be supported very shortly.
* Sensor detection and reporting of the first data can be slightly delayed (up to 200ms) if the SDK needs to discover calibration first. This will not happen in production level devices.
