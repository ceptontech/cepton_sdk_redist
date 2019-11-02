# Cepton SDK

## Overview

The Cepton SDK provides the following features

- **Networking**: Listen for sensor packets.
- **Capture Replay**: Read sensor packets from a PCAP file.
- **Parsing**: Parse sensor packets.
- **Calibration**: Apply sensor calibration.
- **Frame Accumulation**: Accumulate sensor points and detect frames.

Currently, the Cepton LiDAR packet formats are under active development, and are not publicly available. The SDK is required for **Parsing** and **Calibration**. All other SDK features are optional, and can be done manually by the user.

## Installation

First, install [CeptonViewer](https://ceptontech.github.io/cepton_sdk_redist/cepton_viewer.html).

To clone the repository, run

```sh
git clone https://github.com/ceptontech/cepton_sdk_redist.git
```

## Documentation

<https://ceptontech.github.io/cepton_sdk_redist/>

## Getting Started

See `samples`.

## Directories

- **bin**: Executable binaries.
- **cmake**: Extra CMake files.
- **csharp**: C# SDK.
- **docs**: Documentation.
- **driveworks**: NVIDIA DriveWorks SDK.
- **include**: C/C++ Headers.
- **lib**: Library binaries.
- **licenses**: Third party licenses.
- **matlab**: MATLAB SDK.
- **python**: Python SDK.
- **ros**: ROS SDK.
- **samples**: Sample C/C++ code.
- **setup**: Installers.

## Compatibility

The library requires C++11 support.

| OS              | Minimum Version |
| --------------- | --------------- |
| `osx`           | `OSX 10.15`     |
| `win64`         | `Windows Vista` |
| `linux-i386`    | `Ubuntu 16.04`  |
| `linux-x86_64`  | `Ubuntu 16.04`  |
| `linux-arm`     | `Ubuntu 16.04`  |
| `linux-aarch64` | `Ubuntu 16.04`  |
