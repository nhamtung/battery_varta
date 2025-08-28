# battery_varta

# Require
- Ubuntu 20.04
- ROS Noetic
- USB-CAN-PEAK

- Install can-utils: $sudo apt-get install can-utils

# Hardware

# Dependency
- ros_canopen package: https://github.com/nhamtung/ros_canopen.git

# Rename CAN device
- Check CAN_PEAK: $lsusb | grep -i peak
- Open config file: $sudo nano /etc/udev/rules.d/80-can.rules
- Add to file:
```
# Gán tên cố định cho USB-CAN PEAK
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="0c72", ATTRS{idProduct}=="000c", NAME:="can_peak"
```
- Restart: 
    + $sudo udevadm control --reload-rules
    + $sudo udevadm trigger
    + Reboot IPC
- Check result: $ip link show

# Config CAN for Battery
- Open config file: $sudo nano /etc/systemd/network/81-can.network
- Add to file:
```
[Match]
Name=can_peak
[CAN]
BitRate=500K
```
- Enable: $sudo systemctl enable systemd-networkd
- Start: $sudo systemctl start systemd-networkd
- Restart: $sudo systemctl restart systemd-networkd
- Command with CAN:
    + Disable CanOpen: $sudo ip link set can_peak down
    + Enable CanOpen: $sudo ip link set can_peak up type can bitrate 250000
    + Listen CanOpen: $candump can_peak
    + Send: $cansend can_peak 123#deadbeef

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
