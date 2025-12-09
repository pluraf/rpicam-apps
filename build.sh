meson setup build -Denable_libav=enabled -Denable_drm=disabled -Denable_egl=disabled -Denable_qt=disabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled

clear; ninja -C build
./build/apps/cnode -n -t 1ms -o f --mqtt_topic hello --mqtt_host 192.168.1.88 --mqtt_client_id ff

