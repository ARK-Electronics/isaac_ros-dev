TARGET_IP=192.168.40.4
TARGET_PORT=5600
gst-launch-1.0 --gst-plugin-path=install/gst_bridge/lib/gst_bridge/ rosimagesrc ros-topic=/camera/infra1/image_rect_raw ! queue max-size-buffers=1 ! video/x-raw,format=GRAY8 ! videoconvert ! x264enc bitrate=2000 tune=zerolatency speed-preset=ultrafast ! video/x-h264,stream-format=byte-stream ! rtph264pay config-interval=1 pt=96 ! udpsink host=$TARGET_IP port=$TARGET_PORT sync=false
