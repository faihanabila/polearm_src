kalo mau colcon pake command di bawah,

colcon build --base-paths src

kalo mau run gui,

sourcing, terus

ros2 run barcode_hmi hmi_node

terus si backend cameranya,

buka terminal baru, sourcing, terus

ros2 run barcode_hmi live_backend --ros-args -p camera1_device:=2 -p camera2_device:=4

**2 ama 4 sesuaiin ama alamat si webcam yoii