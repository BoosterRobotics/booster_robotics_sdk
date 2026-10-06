#!/bin/bash

booster_sdk_dir=$(
    cd $(dirname $0)
    pwd
)
echo "Booster Robotics SDK Dir = $booster_sdk_dir"

cpu_arch=$(uname -m)
echo "CPU Arch=$cpu_arch"

# Extract the Ubuntu major version generically instead of matching an
# explicit, ever-growing list of versions. This means new Ubuntu releases
# (26, 28, 30, ...) work automatically without editing this script, as long
# as a compatible lib directory is shipped or can be fallen back to below.
ubuntu_version=$(lsb_release -rs)
ubuntu_major_version=${ubuntu_version%%.*}

if [ -z "$ubuntu_major_version" ] || ! [[ "$ubuntu_major_version" =~ ^[0-9]+$ ]]; then
    echo -e "\033[31m[ERROR] Could not determine Ubuntu version from '$ubuntu_version' \033[0m"
    exit 1
fi

if [ "$ubuntu_major_version" -lt 22 ]; then
    echo -e "\033[31m[ERROR] Unsupported Ubuntu version: $ubuntu_version \033[0m"
    echo -e "\033[31m[HINT] This SDK only supports Ubuntu 22.04 and above. Installation aborted.\033[0m"
    exit 1
fi

echo "Ubuntu Version = $ubuntu_major_version"

set -e

apt update

apt install -y build-essential
apt install -y cmake

# Resolve which lib directory to use:
# 1. lib/$cpu_arch (arch-only layout, no per-version split)
# 2. lib/$cpu_arch/$ubuntu_major_version (exact version match)
# 3. Highest available lib/$cpu_arch/<version> that is <= the detected
#    version (best-effort fallback for Ubuntu releases newer than any the
#    SDK has been explicitly packaged for, e.g. 28.04, 30.04, ...)
booster_sdk_lib_dir=""

if [ -d "$booster_sdk_dir/lib/$cpu_arch" ] && [ -n "$(ls -A "$booster_sdk_dir/lib/$cpu_arch" 2>/dev/null | grep -v '^[0-9]\+$')" ]; then
    booster_sdk_lib_dir="$booster_sdk_dir/lib/$cpu_arch"
elif [ -d "$booster_sdk_dir/lib/$cpu_arch/$ubuntu_major_version" ]; then
    booster_sdk_lib_dir="$booster_sdk_dir/lib/$cpu_arch/$ubuntu_major_version"
else
    best_match=""
    if [ -d "$booster_sdk_dir/lib/$cpu_arch" ]; then
        for dir in "$booster_sdk_dir/lib/$cpu_arch"/*/; do
            version_dir=$(basename "$dir")
            if [[ "$version_dir" =~ ^[0-9]+$ ]] && [ "$version_dir" -le "$ubuntu_major_version" ]; then
                if [ -z "$best_match" ] || [ "$version_dir" -gt "$best_match" ]; then
                    best_match=$version_dir
                fi
            fi
        done
    fi

    if [ -n "$best_match" ]; then
        booster_sdk_lib_dir="$booster_sdk_dir/lib/$cpu_arch/$best_match"
        echo -e "\033[33m[WARN] No lib build packaged for Ubuntu $ubuntu_major_version. Falling back to closest available version: $best_match \033[0m"
    fi
fi

echo "SDK Lib Dir = $booster_sdk_lib_dir"

if [ -z "$booster_sdk_lib_dir" ] || [ ! -d "$booster_sdk_lib_dir" ]; then
    echo -e "\033[31m[ERROR] SDK library directory not found for CPU arch '$cpu_arch' / Ubuntu '$ubuntu_major_version' \033[0m"
    exit 1
fi

cp -r "$booster_sdk_dir"/include/* /usr/local/include
cp -r "$booster_sdk_lib_dir"/* /usr/local/lib
echo "Booster Robotics SDK installed successfully!"

ldconfig
