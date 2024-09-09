#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string.h>
#include <string> 
#include <math.h> 
#include <stdlib.h>    
#include <unistd.h>      
#include <rcl/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
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
#define SEND_DATA_CHECK   1          // Send data check flag
#define READ_DATA_CHECK   0          // Receive data check flag
#define FRAME_HEADER      0X7B       // Frame header
#define FRAME_TAIL        0X7D       // Frame tail
#define RECEIVE_DATA_SIZE 24         // Length of data sent by the lower computer
#define SEND_DATA_SIZE    11         // Length of data sent to the lower machine
#define PI                3.1415926f // Value of PI

// IMU data conversion ratios
#define GYROSCOPE_RATIO   0.00026644f // Gyroscope conversion ratio
#define ACCEl_RATIO       1671.84f    // Accelerometer conversion ratio

extern sensor_msgs::msg::Imu Mpu6050; // External IMU topic data

// Covariance matrices for odometry data
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
                                          0, 1e-3, 0, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e3};

const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
                                           0, 1e-3, 0, 0, 0, 0,
                                           0, 0, 1e6, 0, 0, 0,
                                           0, 0, 0, 1e6, 0, 0,
                                           0, 0, 0, 0, 1e6, 0,
                                           0, 0, 0, 0, 0, 1e3};

// Data structure for velocity and position
typedef struct __Vel_Pos_Data_
{
    float X; // X position
    float Y; // Y position
    float Z; // Z position
} Vel_Pos_Data;

// IMU data structure
typedef struct __MPU6050_DATA_
{
    short accele_x_data; // X acceleration
    short accele_y_data; // Y acceleration
    short accele_z_data; // Z acceleration
    short gyros_x_data;  // X gyroscope data
    short gyros_y_data;  // Y gyroscope data
    short gyros_z_data;  // Z gyroscope data
} MPU6050_DATA;

// Data structure for sending data to the lower machine
typedef struct _SEND_DATA_
{
    uint8_t tx[SEND_DATA_SIZE]; // Transmission data
    float X_speed;              // X speed
    float Y_speed;              // Y speed
    float Z_speed;              // Z speed
    unsigned char Frame_Tail;   // Frame tail
} SEND_DATA;

// Data structure for receiving data from the lower machine
typedef struct _RECEIVE_DATA_
{
    uint8_t rx[RECEIVE_DATA_SIZE]; // Reception data
    uint8_t Flag_Stop;              // Stop flag
    unsigned char Frame_Header;     // Frame header
    float X_speed;                  // X speed
    float Y_speed;                  // Y speed
    float Z_speed;                  // Z speed
    float Power_Voltage;           // Power voltage
    unsigned char Frame_Tail;       // Frame tail
} RECEIVE_DATA;

// Robot chassis class to initialize data and publish topics
class turn_on_robot : public rclcpp::Node
{
    public:
        turn_on_robot();
        ~turn_on_robot(); // Destructor
        void Control();   // Loop control code
        void update_tf(geometry_msgs::msg::TransformStamped::SharedPtr odom_tf);
        void Publish_Odom(); // Publish odometry topic
        serial::Serial Stm32_Serial; // Serial object

    private:
        rclcpp::Time _Now, _Last_Time; // Time-dependent variables
        float Sampling_Time;            // Sampling time
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub; // Velocity subscription
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr Akm_Cmd_Vel_Sub; // Ackermann command subscription

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher; // Odometry publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher; // Voltage publisher
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher; // IMU publisher
        rclcpp::Publisher<wheeltec_robot_msg::msg::Data>::SharedPtr robotpose_publisher; // Robot pose publisher
        rclcpp::Publisher<wheeltec_robot_msg::msg::Data>::SharedPtr robotvel_publisher; // Robot velocity publisher

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bro;
        rclcpp::TimerBase::SharedPtr test_timer;
        rclcpp::TimerBase::SharedPtr odom_timer;
        rclcpp::TimerBase::SharedPtr imu_timer;
        rclcpp::TimerBase::SharedPtr voltage_timer;
        rclcpp::TimerBase::SharedPtr robotpose_timer;
        rclcpp::TimerBase::SharedPtr robotvel_timer;

        void declare_parameters();
        void get_parameters();
        void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux); // Command velocity callback
        void Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl); // Ackermann command callback
        void Publish_ImuSensor(); // Publish IMU sensor topic
        void Publish_Voltage(); // Publish power supply voltage topic
        auto createQuaternionMsgFromYaw(double yaw); // Create quaternion from yaw

        // Read sensor data from serial port
        bool Get_Sensor_Data();   
        unsigned char Check_Sum(unsigned char Count_Number, unsigned char mode); // Checksum function
        short IMU_Trans(uint8_t Data_High, uint8_t Data_Low); // IMU data conversion
        float Odom_Trans(uint8_t Data_High, uint8_t Data_Low); // Odometer data conversion

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

