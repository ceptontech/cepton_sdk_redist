## Building

### Unix

```sh
cd cepton_sdk_redist/samples
mkdir build
cd build
cmake ..
make
```

### Windows

The following commands are for a UNIX command line (e.g. Git Bash).

```sh
cd cepton_sdk_redist/samples
mkdir build
cd build
cmake -G "Visual Studio 16 2019" ..
```

To build from the command line, run

```sh
cmake --build . --config Release
```

To build in Visual Studio

- Click `File` -> `Open` -> `Project/Solution`. Select `cepton_sdk_redist/samples/build/cepton_sdk_samples.sln`.
- Build the project.