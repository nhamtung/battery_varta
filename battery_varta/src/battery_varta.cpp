#include <sys/ioctl.h>      // ioctl
#include <net/if.h>         // struct ifreq, IFNAMSIZ
#include <linux/can.h>      // struct can_frame
#include <linux/can/raw.h>  // CAN_RAW
#include <unistd.h>         // close
#include <cstring>          // memset
#include <cstdio>           // printf
#include <cstdlib>          // exit

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/BatteryState.h"

#include "agv_define/define.h"
#include "agv_dynparam/dynparam_lib.h"
#include "dynamic_reconfigure/Config.h"
#include <diagnostic_msgs/DiagnosticArray.h> 


#define DESIGN_CAPACITY 32;     // Ah
#define NODE_ID 0x01            // Defaule 0x01
#define BAUDRATE 250000;        // Default 250kb

#define VOLTAGE_RATIO 0.001     // V
#define CURRENT_RATIO 0.001     // A
#define TEMP_RATIO 0.1          // C

ros::Publisher battery_pub;
sensor_msgs::BatteryState prev_battery;
sensor_msgs::BatteryState battery;

ros::Publisher diagnostic_pub;
diagnostic_msgs::DiagnosticArray prev_diagnostic;
diagnostic_msgs::DiagnosticStatus connection;
diagnostic_msgs::DiagnosticStatus battery_data;

PowerSupplyStatusStruct power_supply_status;
PowerSupplyHealthStruct power_supply_health;
PowerSupplyTechnologyStruct power_supply_technology;

DynparamLib_ns::DynparamLib* dynparam_obj;
ros::Subscriber dynparam_sub;

boost::thread* receive_data_thread;
boost::thread* check_data_thread;
std::string can_interface = "can0";

int info_timeout = 30;   // second
int count_charging_state = 0;
ros::Duration updateInterval;
ros::Time lastUpdateTimestamp;

void pubDiagnostic() {
    diagnostic_msgs::DiagnosticArray diagnostic;
    
    diagnostic.status.push_back(connection);
    diagnostic.status.push_back(battery_data);

    if(diagnostic.status != prev_diagnostic.status) {
      diagnostic.header.seq = prev_diagnostic.header.seq+1;
      diagnostic.header.stamp = ros::Time::now();
      diagnostic.header.frame_id = "battery";

      diagnostic_pub.publish(diagnostic);
      prev_diagnostic = diagnostic;
    }
}
bool checkPassedTime(){
	ros::Duration passedTime = ros::Time::now() - lastUpdateTimestamp;
	return(passedTime >= updateInterval ? true:false);
}
void pubBattery(){
    battery.charge = battery.percentage;
    battery.design_capacity = DESIGN_CAPACITY;                            // A
    battery.capacity = battery.charge*battery.design_capacity/100.0;   // Ah
    
    battery.power_supply_status = power_supply_status.POWER_SUPPLY_STATUS_DISCHARGING;
    if(battery.current > 0) count_charging_state++;
    else count_charging_state = 0;
    if(count_charging_state >= 10){
      battery.power_supply_status = power_supply_status.POWER_SUPPLY_STATUS_CHARGING;
      count_charging_state = 10;
    }
    battery.power_supply_health = power_supply_health.POWER_SUPPLY_HEALTH_GOOD;      
    battery.power_supply_technology = power_supply_technology.POWER_SUPPLY_TECHNOLOGY_LION;   
    battery.present = true;

    battery.location = "";
    battery.serial_number = "";

    if(prev_battery != battery){
      battery.header.seq++;
      battery.header.stamp = ros::Time::now();
      battery.header.frame_id = "battery";
      battery_pub.publish(battery); 
      prev_battery = battery;
    }
}

float decodeVoltage(const struct can_frame& frame) {
    return (frame.data[0] | (frame.data[1] << 8)) * VOLTAGE_RATIO;
}
float decodeCurrent(const struct can_frame& frame) {
    // Ghép 2 byte thành 16-bit
    int16_t raw = frame.data[4] | (frame.data[5] << 8);
    // Chuyển sang float với hệ số
    return static_cast<float>(raw) * CURRENT_RATIO; //0.001f;
}
float decodeTemperature(const struct can_frame& frame) {
    return (frame.data[2] | (frame.data[3] << 8)) * TEMP_RATIO;
}
float decodeCapacity(const struct can_frame& frame) {
    return (frame.data[0] | (frame.data[1] << 8)) * CURRENT_RATIO;
}
float decodeFullCapacity(const struct can_frame& frame) {
    return (frame.data[2] | (frame.data[3] << 8)) * CURRENT_RATIO;
}
float decodeRemainCapacity(const struct can_frame& frame) {
    return (frame.data[4] | (frame.data[5] << 8)) * CURRENT_RATIO;
}

void dynparamCallback(const dynamic_reconfigure::Config::ConstPtr& msg){}

int receiveDataThread() {
    // setup socketCAN
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(s < 0) { perror("SocketCAN"); return 1; }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, can_interface.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { 
        perror("Bind"); 
        connection.level = OPERATOR_ERROR;
        connection.message = "[battery_varta] Can not connect to CAN module";
        pubDiagnostic();
        return 1; 
    }

    lastUpdateTimestamp = ros::Time::now();
    connection.level = OPERATOR_OK;
    connection.message = "[battery_varta] Connected to CAN";
    pubDiagnostic();

    struct can_frame frame;
    ros::Rate loop_rate(50); // 50 Hz
    while (ros::ok()) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes > 0) {
            // ROS_INFO("[battery_varta] Receive data size: %d", nbytes);
            // ROS_INFO("[battery_varta] frame.can_id: %x", frame.can_id);
            // ROS_INFO("[battery_varta] frame.can_dlc: %d", frame.can_dlc);
            // ROS_INFO("[battery_varta] frame.__pad: %d", frame.__pad);
            // ROS_INFO("[battery_varta] frame.__res0: %d", frame.__res0);
            // ROS_INFO("[battery_varta] frame.__res1: %d", frame.__res1);

            // fprintf(stderr, "frame - (id [dlc] data) : %04X   [%d]  ", frame.can_id, frame.can_dlc);
            // for(int i=0; i<frame.can_dlc; i++){
            //     fprintf(stderr, "%02X ", frame.data[i]);
            // }
            // fprintf(stderr, "\n");

            lastUpdateTimestamp = ros::Time::now();
            battery_data.level = OPERATOR_OK;
            battery_data.message = "[battery_varta] Got battery data";
            if(frame.can_id == 0x181) {                         // ID battery voltage && current
                battery.voltage = decodeVoltage(frame);
                ROS_INFO_THROTTLE(30, "[battery_varta] Voltage: %.3f V", battery.voltage);
                battery.current = decodeCurrent(frame);
                ROS_INFO_THROTTLE(30, "[battery_varta] Current: %.3f A", battery.current);
            }
            if(frame.can_id == 0x281){
                battery.temperature = decodeTemperature(frame);
                ROS_INFO_THROTTLE(30, "[battery_varta] Temperature: %.1f C", battery.temperature);
            }
            if(frame.can_id == 0x381){
                battery.capacity = decodeCapacity(frame);
                ROS_INFO_THROTTLE(30, "[battery_varta] capacity: %.3f A", battery.capacity);
                battery.design_capacity = decodeFullCapacity(frame);
                ROS_INFO_THROTTLE(30, "[battery_varta] design_capacity: %.3f A", battery.design_capacity);
                float remain_capacity = decodeRemainCapacity(frame);
                ROS_INFO_THROTTLE(30, "[battery_varta] remain_capacity: %.3f A", remain_capacity);
                battery.percentage = 100*remain_capacity/battery.design_capacity;
                ROS_INFO_THROTTLE(30, "[battery_varta] percentage: %.1f percent", battery.percentage);
            }
            if(frame.can_id == 0x481){
                // Status of battery
            }
        }
        pubBattery();
        pubDiagnostic();
        ros::spinOnce();
        loop_rate.sleep();
    }
    close(s);
}
void checkDataThread(){
    ros::Rate loop_rate(2);
    while (ros::ok()) {
        if(checkPassedTime()){
          battery_data.level = OPERATOR_ERROR;
          battery_data.message = "[battery_varta] Timeout (" + to_string(info_timeout) + "s) when try to get battery data";
        }
        pubDiagnostic();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool loadParam(ros::NodeHandle n, std::string node_name){
    n.param<int>(node_name + "/info_timeout", info_timeout, 10);
    ROS_INFO("%s/info_timeout: %d", node_name.c_str(), info_timeout);
    n.param<std::string>(node_name + "/can_interface", can_interface, "can0");
    ROS_INFO("%s/can_interface: %s", node_name.c_str(), can_interface.c_str());

    return true;
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "battery_varta");
    ros::NodeHandle n;

    std::string node_name = ros::this_node::getName();
    ROS_INFO("node_name: %s", node_name.c_str());
    loadParam(n, node_name);

  	diagnostic_pub = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostic_battery", 10);
  	battery_pub = n.advertise<sensor_msgs::BatteryState>("/battery", 10);
    
    dynparam_obj = new DynparamLib_ns::DynparamLib("agv_dynparam");
    dynparam_sub = n.subscribe<dynamic_reconfigure::Config>("/agv_dynparam/parameter_updates", 1, dynparamCallback);
    ROS_INFO("[battery_varta] Subscriber topic /agv_dynparam/parameter_updates");

    ros::Duration(1).sleep();
    // Connection with Battery
    connection.name = "connect_can";
    connection.hardware_id = "Battery";
    connection.level = OPERATOR_STABLE;
    connection.message = "Start Node";
    // battery_data
    battery_data.name = "battery_data";
    battery_data.hardware_id = "Battery";
    battery_data.level = OPERATOR_STABLE;
    battery_data.message = "Start Node";
    pubDiagnostic();

    ros::Duration(1).sleep();

    receive_data_thread = new boost::thread (&receiveDataThread);      
    check_data_thread = new boost::thread (&checkDataThread);     
    updateInterval = ros::Duration(info_timeout);     

    ros::spin();
    delete(dynparam_obj);
    return 0;
}
