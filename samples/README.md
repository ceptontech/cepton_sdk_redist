## Building

### Unix

```sh
mkdir build
cd build
cmake ..
make
```

### Windows

```sh
mkdir build
cd build
cmake -G "Visual Studio 15 2017 Win64" ..
```

In Visual Studio:

- Click `File` -> `Open` -> `Project/Solution`. Select "cepton_sdk_redist/samples/build/cepton_sdk_samples.sln".
- Build the project.