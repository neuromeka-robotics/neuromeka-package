# Neuromeka Robot API
![PyPI](https://img.shields.io/pypi/v/neuromeka)

This package provides client protocols for users to interact with Neuromeka's robot products, including Indy, Moby, Ecat, and Motor.

* Website: http://www.neuromeka.com
* Source code: https://github.com/neuromeka-robotics/neuromeka-package
* PyPI package: https://pypi.org/project/neuromeka/
* Documents: https://docs.neuromeka.com


## Installation
The current default grpc version for both python and c++ is 1.59.

### Python
You can install the package from PyPI:

Install dependencies if missing:
```bash
sudo apt-get install libjpeg-dev zlib1g-dev 
```

On python < 3.12: 
```bash
sudo apt install -y python<your_version>-distutils 
```
or simply 
```bash
sudo apt install -y python3-distutils 
```

Install Neuromeka package
```bash
pip3 install neuromeka
```

### C++ (On Linux)

#### **1. Install GRPC**

This guide is summarized from grpc website. For more information, please visit [grpc.io](https://grpc.io/docs/languages/cpp/quickstart/). 

Choose a directory to hold locally installed packages. This page assumes that the environment variable MY_INSTALL_DIR holds this directory path. For example:
```bash
export MY_INSTALL_DIR=$HOME/.local
```
Ensure that the directory exists:
```bash
mkdir -p $MY_INSTALL_DIR
```

Add the local bin folder to your path variable, for example:
```bash
export PATH="$MY_INSTALL_DIR/bin:$PATH"
```

Install cmake\
You need version 3.13 or later of cmake. Install it by following these instructions:
```bash
sudo apt install -y cmake
```
Check the version of cmake:
```bash
cmake --version
```

On Linux, the system-wide CMake version might be outdated. To install a more recent version in your local installation directory, use:
```bash
wget -q -O cmake-linux.sh https://github.com/Kitware/CMake/releases/download/v3.19.6/cmake-3.19.6-Linux-x86_64.sh
sh cmake-linux.sh -- --skip-license --prefix=$MY_INSTALL_DIR
rm cmake-linux.sh
```

Install other required tools\
Install the basic tools required to build gRPC:

```bash
$ sudo apt install -y build-essential autoconf libtool pkg-config
```

Clone the grpc repo and its submodules:
```bash
$ git clone --recurse-submodules -b v1.59.0 --depth 1 --shallow-submodules https://github.com/grpc/grpc
```

Build and Install gRPC and Protocol Buffers

gRPC applications often use Protocol Buffers for service definitions and data serialization, as in the example code that follows. To build and locally install gRPC and Protocol Buffers, run:

```bash
cd grpc
mkdir -p cmake/build
pushd cmake/build
cmake -DgRPC_INSTALL=ON \
      -DgRPC_BUILD_TESTS=OFF \
      -DCMAKE_INSTALL_PREFIX=$MY_INSTALL_DIR \
      ../..
make -j 4
make install
popd
```

**Important**
It is strongly recommended to install gRPC locally with a properly set **CMAKE_INSTALL_PREFIX**, as there is no simple way to uninstall gRPC after a global installation.

#### **2. Download and Build example**

Clone the Example Repository
```bash
git clone https://github.com/neuromeka-robotics/neuromeka-package.git
```

Navigate to the C++ Folder
```bash
cd neuromeka-package/cpp/
```

Create and Enter the Build Directory
```bash
mkdir build && cd build
```

Run CMake Configuration and Build the Example
```bash
cmake -DBUILD_PROTO=OFF ..
cmake --build .
```
<!-- If you need to rebuild the **.proto** files, set **-DBUILD_PROTO=ON**. The build_proto executable file will be generated. After running this file, the generated proto files will be located in the proto/cpp_generated folder. -->

### C++ (On Windows)
It is recommended to use Visual Studio Code for this setup.

#### **1. Setup your environment**
Install [Visual Studio Code](https://code.visualstudio.com/download)
- Install following extensions: C/C++, C/C++ Extension Pack and CMake Tools.

Install [vs_BuildTools](https://visualstudio.microsoft.com/downloads/)
- In BuildTools option, please choose Desktop Development with C++ and Visual Studio extension development.

#### **2. Install GRPC**

When you are all set, please open Developer Command Prompt and run the following commands:
```bash
git clone -b v1.59.0 https://github.com/grpc/grpc
cd grpc
git submodule update --init
```

Create build folder
```bash
cd cmake
mkdir build
cd build
```

Configure and build using Cmake
```bash
cmake -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=OFF -DABSL_PROPAGATE_CXX_STD=ON -DgRPC_INSTALL=ON -DgRPC_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=..\..\..\install ../..
```
You can set you install path with "-DCMAKE_INSTALL_PREFIX=\<your path\>

Build and install grpc
```bash
cmake --build . --config Release --target install
```

#### **3. Download and Build example**
Clone the Example Repository
```bash
git clone https://github.com/neuromeka-robotics/neuromeka-package.git
```
Open Visual Studio Code and open **neuromeka-package/cpp/** folder
Config your build with settings.json in .vscode folder. If you need to rebuild the **.proto** files, set **-DBUILD_PROTO=ON**. The build_proto executable file will be generated.

Then, modify the CMakeList.txt. Change following lines to point to your grpc installation directory.
```cmake
set(absl_DIR "E:/Example/install/lib/cmake/absl")
set(utf8_range_DIR "E:/Example/install/lib/cmake/utf8_range")
set(protobuf_DIR "E:/Example/install/cmake")
set(gRPC_DIR "E:/Example/install/lib/cmake/grpc")
```

"Press Ctrl+Shift+P or go to View -> Command Palette to open the Command Palette, and choose the following commands:
- CMake: Scan for kits: To scan for compilers on your computer.
- CMake: Select a kit: To select a specific compiler (e.g., Visual Studio Build Tools 2022 Release - amd64).

When you are done, press the Build button at the bottom left.

## Usage

### Python
Python `neuromeka` package contatins the following client classes:

* IndyDCP3 in indydcp3.py
* IndyEye in eye.py
* EtherCAT in ecat.py
* Moby in moby.py

To use a client class, simply import it and create an instance:

```python
from neuromeka import IndyDCP3, IndyEye, EtherCAT, MobyClient

moby = MobyClient("192.168.214.20")
indy = IndyDCP3("192.168.0.11")
eye = IndyEye("192.168.0.12")
ecat = EtherCAT("192.168.0.11")
```

### C++
C++ `neuromeka` package contains the following client class:
* IndyDCP3 in indydcp3.cpp

To use a client class, please refer to example_indydcp3.cpp 

## Dependencies

### Python
This package requires the following dependencies:

* grpcio
* grpcio-tools
* protobuf
* requests
* Pillow
* numpy
* pyModbusTCP
* netifaces

These dependencies will be automatically installed when you install the package using pip.

## Examples
Please refer to the 'python/examples' and 'cpp/src' folder in the package for Python and C++ usage examples.

## Support
If you encounter any issues or need help, please open an issue on the project's repository.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
