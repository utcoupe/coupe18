#!/bin/bash

# TODO select architecture
img_tag=utcoupe-ros-kinetic-armv7
img_dir=utcoupe-ros-kinetic-armv7
img_ws_root_dir=/utcoupe/coupe18

docker build -t ${img_tag} ${img_dir}

mkdir -p "${UTCOUPE_WORKSPACE}"/ros_ws/install

# looks slower than the other method
# docker run \
#     -i \
#     --mount type=bind,source="${UTCOUPE_WOKSPACE}",target="${img_ws_root_dir}" \
#     ${img_tag}

docker run \
    -i \
    --mount type=bind,source="${UTCOUPE_WORKSPACE}"/ros_ws/src,target="${img_ws_root_dir}"/ros_ws/src,readonly \
    --mount type=bind,source="${UTCOUPE_WORKSPACE}"/libs,target="${img_ws_root_dir}"/libs,readonly \
    --mount type=bind,source="${UTCOUPE_WORKSPACE}"/ros_ws/install,target="${img_ws_root_dir}"/ros_ws/install \
    ${img_tag}
# Makes cmake crash
#    --tmpfs "${img_ws_root_dir}"/ros_ws/devel
#    --tmpfs "${img_ws_root_dir}"/ros_ws/build
