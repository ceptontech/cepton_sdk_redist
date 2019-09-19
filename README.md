# Cepton SDK

## Overview

The Cepton SDK provides the following features

- **Parsing**: parse sensor packets
- **Calibration**: apply sensor calibration
- **Networking**: listen for sensor packets
- **Capture Replay**: read sensor packets from a PCAP file
- **Frame Accumulation**: accumulate and output sensor points by frame

Currently, the Cepton LiDAR packet formats are under active development, and are not publicly available. The SDK is required for **Parsing** and **Calibration**. All other SDK features are optional, and can be done manually by the user.

Please read the API documentation at `docs/html/index.html`.

## Installation

To just install the executable tools, download one of the installers in `cepton_sdk_redist/setup`.

To clone the entire repository, run

```sh
git clone https://github.com/ceptontech/cepton_sdk_redist.git
```

## Compatibility

The library requires C++11.

| OS              | Compiler             | Target          |
| --------------- | -------------------- | --------------- |
| `osx`           | `LLVM 9.1`           |                 |
| `win64`         | `Visual Studio 2017` | `Windows Vista` |
| `linux-x86_64`  | `gcc-5`              | `Ubuntu 16.04`  |
| `linux-arm`     | `gcc-5`              | `Ubuntu 16.04`  |
| `linux-aarch64` | `gcc-5`              | `Ubuntu 16.04`  |

## Getting Started

See `samples`.