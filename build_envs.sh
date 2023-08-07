#!/bin/bash

BUILD_DIR=".pio/build/"
RELEASE_DIR="./release"
TAG=$(git tag --sort=taggerdate | tail -1)

# make release directory
mkdir -p ${RELEASE_DIR}

# build all configs
pio run

# copy builds to release
for config in `ls -d ${BUILD_DIR}/*/`
do
    config_name=$(basename "$config")
    if [[ "${config_name}" != *"debug"* ]]; then
        echo $config
        # gzip -c -f "${config}/firmware.hex" > "${RELEASE_DIR}/${config_name}-${TAG}.gz"
        cp "${config}/firmware.hex" "${RELEASE_DIR}/arduino-motorfocus-${config_name}-${TAG}.hex"
    fi
done

ls -lh ${RELEASE_DIR}