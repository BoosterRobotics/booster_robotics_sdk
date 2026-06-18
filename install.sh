#!/bin/bash

booster_sdk_dir=$(
    cd $(dirname $0)
    pwd
)
echo "Booster Robotics SDK Dir = $booster_sdk_dir"

cpu_arch=$(uname -m)
echo "CPU Arch=$cpu_arch"

# Perform version check early to avoid wasting time on apt update for unsupported systems
ubuntu_version=$(lsb_release -rs)
case $ubuntu_version in
    24.*) ubuntu_version_flag=24 ;;
    22.*) ubuntu_version_flag=22 ;;
    *) 
        # Print a red error message and exit if the version is not 22.* or 24.*
        echo -e "\033[31m[ERROR] Unsupported Ubuntu version: $ubuntu_version \033[0m"
        echo -e "\033[31m[HINT] This SDK only supports Ubuntu 22.04 and above. Installation aborted.\033[0m"
        exit 1 
        ;;
esac

echo "Ubuntu Version = $ubuntu_version_flag"

set -e

apt update

apt install -y build-essential
apt install -y cmake

booster_sdk_lib_dir=$booster_sdk_dir/lib/$cpu_arch
if [ ! -d "$booster_sdk_lib_dir" ] && [ -d "$booster_sdk_dir/lib/$cpu_arch/$ubuntu_version_flag" ]; then
    booster_sdk_lib_dir=$booster_sdk_dir/lib/$cpu_arch/$ubuntu_version_flag
fi

echo "SDK Lib Dir = $booster_sdk_lib_dir"

if [ ! -d "$booster_sdk_lib_dir" ]; then
    echo -e "\033[31m[ERROR] SDK library directory not found: $booster_sdk_lib_dir \033[0m"
    exit 1
fi

cp -r "$booster_sdk_dir"/include/* /usr/local/include
cp -r "$booster_sdk_lib_dir"/* /usr/local/lib
echo "Booster Robotics SDK installed successfully!"

ldconfig
