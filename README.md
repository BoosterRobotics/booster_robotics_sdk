# Booster Robotics SDK
Booster Robotics SDK aims to provide a simple and easy-to-use interface for developers to control the Booster Robotics products.
Booster Robotics SDK provides APIs for both C++ and Python.

## Prebuild environment
* OS  (Ubuntu 22.04 LTS)  
* CPU  (aarch64 and x86_64)   
* Compiler  (gcc version 11.4.0) 

## Installation of deps
```bash
sudo ./install.sh
```
## C++ SDK Usage
### Build C++ examples
```bash
mkdir build
cd build
cmake ..
make
```

### Run examples
#### 1. run b1_loco_example_client locally
```
cd build
./b1_loco_example_client 127.0.0.1
```
#### 2. run b1_low_level_subscriber locally
```
cd build
./b1_low_level_subscriber
```
#### 3. run other example xxx locally
```
cd build
./xxx 127.0.0.1
```

## Python SDK Usage
### Option 1: Install via pip (Recommended)
The easiest way to use the SDK with Python is to install the pre-built package.

```bash
pip install booster_robotics_sdk_python --user
```

### Option 2: Build from Source
If you need to build the Python binding locally for development or debugging purposes, follow these steps:
#### Install build dependencies
```bash
pip3 install pybind11
pip3 install pybind11-stubgen
```
if pybind11-stubgen cannot be found even after pip install, export PATH
```bash
export PATH=/home/[user name]/.local/bin:$PATH
```
#### Build and install
```bash
mkdir build
cd build
cmake .. -DBUILD_PYTHON_BINDING=on
make
sudo make install
```

### Python example
```bash
python3 python_example/sdk_pybind_b1_exmaple.py 127.0.0.1
```
Note: If you installed via pip, you can find the examples at ~/.local/lib/python3.10/site-packages/python_example.

## License

This project is licensed under the Apache License, Version 2.0. See the LICENSE file for details.

This project uses the following third-party libraries:
- fastDDS (Apache License 2.0)
- pybind11 (BSD 3-Clause License)
- pybind11-stubgen (MIT License)