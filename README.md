# Booster Robotics SDK
Booster Robotics SDK aims to provide a simple and easy-to-use interface for developers to control the Booster Robotics products.
This release package provides the C++ SDK. Python SDK delivery is handled by the pip package.

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
The Python SDK is delivered separately through pip.

```bash
pip install booster_robotics_sdk_python --user
```

## License

This project is licensed under the Apache License, Version 2.0. See the LICENSE file for details.

This project uses the following third-party libraries:
- fastDDS (Apache License 2.0)
