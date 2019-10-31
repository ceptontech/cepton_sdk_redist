# Cepton SDK Release Notes

## Version 1.16.1 2019-11-01

- Minor CeptonViewer UI tweaks.

## Version 1.16.0 2019-10-24

- Remove deprecated sensor models.
- Improve `cepton_sdk::util::FrameDetector` (API changed).
- Add `cepton_is_sora`, `cepton_is_hr80`, `cepton_is_vista`.
- Add `segment_id` into `CeptonSensorImagePoint`.
- Improve documentation.

## Version 1.15 2019-09-12

- Add serial lines callback for receiving GPS/INS data.
- Remove `*_def.h`/`*_undef.h` files.
- Add error macros.
- Add `VERSION` files.
- Add `python/samples/cepton_georeference.py` script.

## Version 1.14 2019-07-10

- Fix cmake redist bugs.
- Improve support for "Vista-P60".
- Fix point cloud organizer bugs.
- Fix unused error checking.
- Allow speed=0 in cepton_sdk_capture_replay_set_speed() to mean replay without delay.

## Version 1.13 2019-05-01

- Add support for Vista-Gen2.
- Updated product naming: Vista M -> Vista-M90, Vista X -> Vista-X120.
- Point cloud organizer utilities.
- Add runtime assert.
- Unhandled errors will now print error message to stderr.

## Version 1.12 2019-03-18

- Minor bugfix release.
- IMPORTANT: User must call `cepton_sdk_deinitialize` manually now.

## Version 1.11 2019-01-25

- Add preliminary support for "Vista M" and "Vista X".
- Rename model name "Vista 860 Gen2" to "Vista 860".
- Add C# wrapper library.

## Version 1.10 2018-12-07

- Change `CEPTON_SDK_VERSION` to 18.
- Add `cepton_sdk_capture_replay_get_filename`.
- Add `CeptonSensorInformation::is_over_heated`.
- Add pdf docs.
- Fix CMake for samples.

## Version 1.9 2018-11-02

- High level API changes.
- Make `cepton_sdk::api::wait` wait forever if `t_length < 0` (default).
- Add `CEPTON_SDK_CONTROL_HOST_TIMESTAMPS` control flag.
- `cepton_sdk::api::initialize` only waits for sensors with capture file.
- Add `cepton_sdk::api::load_replay`.
- Make `cepton_sdk::util::SensorPoint` subclass of `cepton_sdk::ImagePoint`.
- Add `cepton_sdk::get_version_string`, `cepton_sdk::get_version_major`, `cepton_sdk::get_version_minor`.
- Add `CMakeLists.txt` and `samples/CMakeLists.txt`.

## Version 1.8 2018-10-08

- Fix calibration loading bug.

## Version 1.7 2018-10-02

- **IMPORTANT:** change `CeptonSensorInformation::measurement_period`: `microseconds` -> `seconds`.
- **IMPORTANT**: change global limits from inline functions to #define.
- Add `cepton_sdk::SensorError::msg`.
- Change `CeptonSensorErrorCode` type: `int` -> `int32`.
- Change `CeptonSensorModel` type: `int32` ->`int16`.
- Add `CeptonSensorModel::VISTA_860_GEN2`.
- Removed `cepton_data_exporter` tool. Use `cepton_export` from Python SDK instead.
- Removed `cepton_serial_redist.py` tool.

## Version 1.6 2018-08-31

- Support for intensities for Vista 860
- New set of APIs for global limits
- Stray filter (experimental)
- Minor improvements in API definitions.
- Change to mock_network_receive to allow accurate timestamping
- Bug fixes

## Version 1.5 2018-08-10

- Fix networking bugs.
- Refactor capture/replay.
- Add samples.

## Version 1.4 2018-08-02

- Improve point timestamps.
- Combine `set_mock_time_base` and `mock_network_receive`.
- Improve errors.
- Add return type.
- Add more sample code.
- Bugfixes.

## Version 1.3 2018-07-23

- Support more accurate HR80W calibrations
- Improved self-diagnosis and fault reporting
- Improvements to python SDK
- Bugfixes

## Version 1.2 2018-07-09

- Add support for latest VISTA_860 revision.
- Change CeptonViewer point colormaps.
- Change timestamps from uint64_t to int64_t.
- Fix threading bugs.

## Version 1.1 2018-06-03

- Added `saturated` flag for all returned points.
- Improved calibrations and better support for intensity output
- Improved support for HR80T Rev2
- Renamed data_exporter to cepton_data_exporter
- Bugfixes and improvements

## Version 1.0 2018-05-01

- Added API documentation.
- Major C interface changes.
- Removed CeptonSensorPoint. All functions return CeptonSensorImagePoint.
- Added frame accumulation modes.
- CeptonViewer bugfixes/improvements.
- Added C++, MATLAB, Python bindings.
- Released partial source code.

## Version 0.9 (beta) 2018-01-09

- New product line supported: Vista 860
- New feature supported in all products: Multiple returns
- Deprecated support for very old firmware version (V1xx)
- Bugfixes

## Version 0.8 (beta) 2017-10-24

- New product line supported: SORA 200
- Internal bug fixes and improvements.
- Better capture replay support

## Version 0.7a (beta) 2017-08-30

- Model enumerations in CeptonSensorInformation
- Lots of internal improvements for clipping, calibration, intensity and GPS.
- Support for new models and firmware variations.
- Bugfixes

## Version 0.7 (beta) 2017-08-11

- Control flags to remove clipped edges
- Control flags to return near pixels (uncalibrated right now)
- Control flags to use SDK without networking stack
- Allow working with sensors through different UDP ports
- Better support for multiple sensors through world coordinate transformations
- Internal bugfixes and improvements.

## Version 0.6d (beta) 2017-07-12

- Correct intensity for units shipped with intensity calibration data.
- Capture replay functionality improvements

## Version 0.6c (beta) 2017-07-06

- New SDK APIs for more capture replay functionality.
- Some calibration data updated.

## Version 0.6b (beta) 2017-06-28

- Service release of binary images only. There is no SDK interface change.
- Fixed a timestamp overflow bug where reported timestamp is incorrect.
- Fixed captured pcap file problems.
- Removed capture_replay sample as the improved functionality is supported in SDK directly.

## Version 0.6a (beta) 2017-06-20

- Service release of binary images only. There is no SDK interface change.
- Key reason for this binary release is some networking stability improvements. Notably fixed a problem where sometimes the SDK is holding up the exit process.

## Version 0.6 (beta) 2017-06-06

- New supported model: HR80W
- SDK entries for image space
- Networking improvements
- SDK in both static and dynamic libraries for all the archs we support
- Capture/Replay functionality embedded in SDK
- New improved CeptonViewer binary is included.

## Version 0.5 (beta) 2017-04-24

- New architechure supported: ARM64. This is primarily for NVIDIA's Jetson TX/TK systems and DrivePX2.
- Explicit support for multiple sensors with per-sensor transformations.
- Decoupled calibration from SDK so that we don't need to rev SDK for calibration changes.
- New improved CeptonViewer binary is included.

## Version 0.4 (beta) 2017-04-12

- !NOTE: coordinate system changed! (to conform to popular world coordinates such as ROS, distance is Y and height is Z starting from SDK version 0.4)
- Support GPS timestamps (require external GPS hookup)
- cepton_sdk_initialize allows RETURN_UNMEASURABLE flag to support a full frame.
- listen_frame do not return partial frames anymore.
- data_exporter: support --split option
- Calibration improvements
- Stability and thread safety improvements

## Version 0.3 (beta) 2017-03-07

- Support Mac OSX starting this version
- Improved support for Linux (esp. Ubuntu 14.04)
- Included pre-built binary of CeptonViewer
- Additional sample for data_exporter to dump points into CSV or binary formats.
- Differentiate between real sensor and a replay of capture.
- Networking change to allow multiple applications to share the same device.

## Version 0.2 (beta) 2017-02-28

- Suport capture/replay, with sample code.
- Add win64_debug library.

## Version 0.1 (beta) 2017-02-25

- Initial release
