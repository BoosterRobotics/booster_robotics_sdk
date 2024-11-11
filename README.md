# Booster Robotics SDK
Booster Robotics SDK aims to provide a simple and easy-to-use interface for developers to control the Booster Robotics products. 

## Prebuild environment
* OS  (Ubuntu 22.04 LTS)  
* CPU  (aarch64 and x86_64)   
* Compiler  (gcc version 11.4.0) 

## Installation
```bash
sudo ./install.sh
```

## Install python package for building python binding locally
```bash
pip3 install pybind11
pip3 install pybind11-stubgen
```

## Build examples
```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

## Build python binding api and install
```bash
mkdir build
cd build
cmake .. -DBUILD_PYTHON_BINDING=on
make
sudo make install
```