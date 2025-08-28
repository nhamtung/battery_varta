# battery_varta

# Require
- Ubuntu 20.04
- ROS Noetic
- USB-CAN-PEAK

- Install can-utils: $sudo apt-get install can-utils

# Hardware

# Dependency
- ros_canopen package: https://github.com/nhamtung/ros_canopen.git

# Publisher
- /battery (sensor_msgs::BatteryState)
- /diagnostic_battery (diagnostic_msgs::DiagnosticArray)

# Build
- Direct to battery_varta folder (git folder)
- Select .so file:
    + Check architect: $lscpu
    + For x86_64 architect: $scp -r ./battery_tada/library/libcontrolcan_raspberry.so ./battery_tada/library/libcontrolcan.so
    + For aarch64 architect: $scp -r ./battery_tada/library/libcontrolcan_jetson.so ./battery_tada/library/libcontrolcan.so
- Direct to workspace (_ws)
- Catkin: $catkin_make install
- Source: $source install/setup.bash

# Permit access for USB_CAN_B
- Check USB Device: $lsusb
- Add config file:
    + Open file: $sudo nano /etc/udev/rules.d/99-usb-can-b.rules
    + Add to file: ACTION=="add", SUBSYSTEMS=="usb", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0053", GROUP="users", MODE="777"
- Restart sdev service: $sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
- Reconnect the USB device

# Test
- Connect battery to PC via USB-CAN-B
- Check port: $lsusb
- Launch: $roslaunch battery_tada battery.launch
- Check: 
    + Data: $rostopic echo /battery
    + Diagnostic: $rostopic echo /diagnostic_battery
