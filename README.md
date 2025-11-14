# rpicam-apps
This is a small suite of libcamera-based applications to drive the cameras on a Raspberry Pi platform.

>[!WARNING]
>These applications and libraries have been renamed from `libcamera-*` to `rpicam-*`. Symbolic links to allow users to keep using the old application names have now been removed.

Build
-----

Prerequisites:
apt install -y libcamera-dev
apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
apt install -y libepoxy0 libepoxy-dev libpng16-16t64 libpng++-dev libpng-dev libjpeg62-turbo libjpeg62-turbo-dev libtiff6 libtiff-dev
apt install -y clang-format-19
apt install -y libpaho-mqttpp-dev libpaho-mqtt-dev libpaho-mqttpp3-1 mosquitto-clients

Update ffmpeg libraries


For usage and build instructions, see the official Raspberry Pi documentation pages [here.](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera-and-rpicam-apps)

License
-------

The source code is made available under the simplified [BSD 2-Clause license](https://spdx.org/licenses/BSD-2-Clause.html).

Status
------

[![ToT libcamera build/run test](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml/badge.svg)](https://github.com/raspberrypi/rpicam-apps/actions/workflows/rpicam-test.yml)
