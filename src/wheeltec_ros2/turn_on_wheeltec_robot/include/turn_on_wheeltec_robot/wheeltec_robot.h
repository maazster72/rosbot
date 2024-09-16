#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <rcl/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "wheeltec_robot_msg/msg/data.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std;

// Macro definitions
#define SEND_DATA_CHECK   1
#define READ_DATA_CHECK   0
#define FRAME_HEADER      0X7B
#define FRAME_TAIL        0X7D
#define RECEIVE_DATA_SIZE 24
#define SEND_DATA_SIZE    11
#define PI                3.1415926f
#define GYROSCOPE_RATIO   0.00026644f
#define ACCEl_RATIO       1671.84f

extern sensor_msgs::msg::Imu Mpu6050;

const double odom_pose_covariance[36] = { /* covariance values */ };
const double odom_twist_covariance[36] = { /* covariance values */ };

// Data structures
typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
} Vel_Pos_Data;

typedef struct __MPU6050_DATA_
{
    short accele_x_data;
    short accele_y_data;
    short accele_z_data;
    short gyros_x_data;
    short gyros_y_data;
    short gyros_z_data;
} MPU6050_DATA;

typedef struct _SEND_DATA_
{
    uint8_t tx[SEND_DATA_SIZE];
    float X_speed;
    float Y_speed;
    float Z_speed;
    unsigned char Frame_Tail;
} SEND_DATA;

typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    float X_speed;
    float Y_speed;
    float Z_speed;
    float Power_Voltage;
    unsigned char Frame_Tail;
} RECEIVE_DATA;

// Robot chassis class
class turn_on_robot : public rclcpp::Node
{
public:
    turn_on_robot();
    ~turn_on_robot(); // Destructor
    void Control(); // Loop control code
    void update_tf(geometry_msgs::msg::TransformStamped::SharedPtr odom_tf);
    void Publish_Odom(); // Publish odometry topic
    serial::Serial Stm32_Serial; // Serial object

private:
    rclcpp::Time _Now, _Last_Time; // Time-dependent variables
    float Sampling_Time; // Sampling time
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr Akm_Cmd_Vel_Sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    rclcpp::Publisher<wheeltec_robot_msg::msg::Data>::SharedPtr robotpose_publisher;
    rclcpp::Publisher<wheeltec_robot_msg::msg::Data>::SharedPtr robotvel_publisher;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bro;
    rclcpp::TimerBase::SharedPtr test_timer;
    rclcpp::TimerBase::SharedPtr odom_timer;
    rclcpp::TimerBase::SharedPtr imu_timer;
    rclcpp::TimerBase::SharedPtr voltage_timer;
    rclcpp::TimerBase::SharedPtr robotpose_timer;
    rclcpp::TimerBase::SharedPtr robotvel_timer;

    // New member variables for command velocities
    std::string akm_cmd_vel; // Command for Ackermann
    std::string cmd_vel;     // Command for Twist

    void declare_parameters();
    void get_parameters();
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
    void Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl);
    void Publish_ImuSensor();
    void Publish_Voltage();
    auto createQuaternionMsgFromYaw(double yaw);

    // Read sensor data from serial port
    bool Get_Sensor_Data();
    unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode);
    short IMU_Trans(uint8_t Data_High, uint8_t Data_Low);
    float Odom_Trans(uint8_t Data_High, uint8_t Data_Low);

    string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id; // Related variables
    int serial_baud_rate; // Serial baud rate
    RECEIVE_DATA Receive_Data; // Serial reception data structure
    SEND_DATA Send_Data; // Serial transmission data structure

    Vel_Pos_Data Robot_Pos; // Robot position
    Vel_Pos_Data Robot_Vel; // Robot speed
    MPU6050_DATA Mpu6050_Data; // IMU data
    float Power_voltage; // Power supply voltage
    size_t count_; // Count variable
};

#endif

