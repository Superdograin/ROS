#!/bin/bash
xterm -title "node_usb_cam" -hold -e "roslaunch usb_cam usb_cam-test.launch" & sleep 1
sleep 5
xterm -title "node_ros_bridge" -hold -e "roslaunch rbx2_gui rosbridge.launch" & sleep 1
sleep 2
xterm -title "node_test_sub" -hold -e "python3 ~/catkin_ws/src/rbx2_gui/scripts/test_ros_sub.py" & sleep 1
