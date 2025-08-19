#!/bin/bash

booster_sdk_dir=$(
    cd $(dirname $0)
    pwd
)
echo "Booster Robotics SDK Dir = $booster_sdk_dir"

cpu_arch=$(uname -m)
echo "CPU Arch=$cpu_arch"

third_party_dir=$booster_sdk_dir/third_party
echo "Third Party Dir = $third_party_dir"

set -e

apt update

apt install -y git
apt install -y build-essential
apt install -y cmake
apt install -y libssl-dev
apt install -y libasio-dev
apt install -y libtinyxml2-dev

ubuntu_version=$(lsb_release -rs)
ubuntu_version_flag=20
case $ubuntu_version in
    22.*) ubuntu_version_flag=22 ;;
    20.*) ubuntu_version_flag=20 ;;
    18.*) ubuntu_version_flag=18 ;;
    *) ubuntu_version_flag=0 ;;
esac

if [ $ubuntu_version_flag -eq 22 ]; then
    booster_sdk_lib_dir=$booster_sdk_dir/lib/$cpu_arch
    third_party_lib_dir=$third_party_dir/lib/$cpu_arch
else
    booster_sdk_lib_dir=$booster_sdk_dir/lib/$cpu_arch/$ubuntu_version_flag
    third_party_lib_dir=$third_party_dir/lib/$cpu_arch/$ubuntu_version_flag
fi

echo "SDK Lib Dir = $booster_sdk_lib_dir"
echo "Third Party Lib Dir = $third_party_lib_dir"

cp -r $booster_sdk_dir/include/* /usr/local/include
cp -r $booster_sdk_lib_dir/* /usr/local/lib
echo "Booster Robotics SDK installed successfully!"

cp -r $third_party_dir/include/* /usr/local/include
cp -r $third_party_lib_dir/* /usr/local/lib
echo "Third Party Libraries installed successfully!"

ldconfig
