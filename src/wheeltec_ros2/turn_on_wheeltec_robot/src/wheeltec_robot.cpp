#include "turn_on_wheeltec_robot/wheeltec_robot.h"  // Robot control class and related functions.
#include "rclcpp/rclcpp.hpp"  // ROS2 client library.
#include "turn_on_wheeltec_robot/Quaternion_Solution.h"  // Quaternion operations (for orientation/rotation).
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"  // Ackermann steering control message.
#include "wheeltec_robot_msg/msg/data.hpp"  // Custom robot message types.

sensor_msgs::msg::Imu Mpu6050;  // IMU sensor message object.

using std::placeholders::_1;  // Placeholder for callback functions.
using namespace std;  // Use standard namespace.

rclcpp::Node::SharedPtr node_handle = nullptr;  // Shared pointer for the ROS2 node.

// Main entry point of the program. Describes the initialisation of ROS2, the instantiation of the `turn_on_robot class`, and the call to the control loop.
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Initialise ROS2.

    turn_on_robot Robot_Control;  // Create robot control object.
    Robot_Control.Control();  // Call control method.

    return 0;
}

// Combines two 8-bit values into a single 16-bit signed value for IMU sensor data.
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16 = 0;
  transition_16 |= Data_High << 8;  // Combine high 8 bits
  transition_16 |= Data_Low;        // Combine low 8 bits
  return transition_16;             // Return combined 16-bit value
}

// Combines two 8-bit values, then converts the 16-bit value from mm/s to m/s for odometry data.
float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16 = 0;
  transition_16 |= Data_High << 8;  // Combine high 8 bits
  transition_16 |= Data_Low;        // Combine low 8 bits
  // Convert mm/s to m/s by splitting integer and fractional parts
  float data_return = (transition_16 / 1000) + (transition_16 % 1000) * 0.001;
  return data_return;               // Return speed in m/s
}

// Transforms the input speed and steering angle from Ackermann commands into a specific data format (bytes) and sends it over a serial connection to control the robot’s movement
void turn_on_robot::Akm_Cmd_Vel_Callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr akm_ctl) 
{
  short transition;  // Intermediate variable to store and manipulate data.
  
  if(akm_cmd_vel == "ackermann_cmd") {
    RCLCPP_INFO(this->get_logger(),"is akm");  // Log a message when an Ackermann command is received.
  }

  // Prepare the data packet
  Send_Data.tx[0] = FRAME_HEADER; // Start of the frame (0x7B).
  Send_Data.tx[1] = 0; // Reserved (unused byte).
  Send_Data.tx[2] = 0; // Reserved (unused byte).

  // Process the target velocity on the X-axis (linear speed).
  transition = 0;
  transition = akm_ctl->drive.speed * 1000; // Convert speed (m/s) to mm/s.
  Send_Data.tx[4] = transition; // Lower 8 bits of the velocity.
  Send_Data.tx[3] = transition >> 8; // Higher 8 bits of the velocity.

  // Process the target angular velocity on the Z-axis (steering angle).
  transition = 0;
  transition = akm_ctl->drive.steering_angle * 1000 / 2; // Convert steering angle (radians) to a scaled value.
  Send_Data.tx[8] = transition; // Lower 8 bits of the steering angle.
  Send_Data.tx[7] = transition >> 8; // Higher 8 bits of the steering angle.

  // Calculate the checksum for the data frame (using the Check_Sum function).
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // Append checksum.
  Send_Data.tx[10] = FRAME_TAIL; // End of the frame (0x7D).

  // Send the data over the serial port to the robot.
  try
  { 
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Write the data to the STM32 serial interface.
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // Error message if serial communication fails.
  }
}

// Processes a velocity command for a robot, translates it into a serial data frame, and sends it to the robot's controller for execution. It handles linear velocity on both X and Y axes and angular velocity on the Z axis
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  short transition;  // Intermediate variable for handling data.
  
  // Check if no Ackermann command is being processed and log this.
  if(akm_cmd_vel == "none") {
    RCLCPP_INFO(this->get_logger(), "not akm");  // Logs a message indicating Ackermann command is not active.
  }

  // Prepare the data packet for sending via serial communication.
  Send_Data.tx[0] = FRAME_HEADER; // Start of the data frame (0x7B).
  Send_Data.tx[1] = 0; // Reserved byte (unused).
  Send_Data.tx[2] = 0; // Reserved byte (unused).

  // Convert and send the linear velocity for the X-axis (forward motion).
  transition = 0;
  transition = twist_aux->linear.x * 1000; // Convert speed from m/s to mm/s.
  Send_Data.tx[4] = transition;           // Lower 8 bits of the X-axis velocity.
  Send_Data.tx[3] = transition >> 8;      // Higher 8 bits of the X-axis velocity.

  // Convert and send the linear velocity for the Y-axis (lateral motion).
  transition = 0;
  transition = twist_aux->linear.y * 1000; // Convert speed from m/s to mm/s.
  Send_Data.tx[6] = transition;            // Lower 8 bits of the Y-axis velocity.
  Send_Data.tx[5] = transition >> 8;       // Higher 8 bits of the Y-axis velocity.

  // Convert and send the angular velocity for the Z-axis (rotation).
  transition = 0;
  transition = twist_aux->angular.z * 1000; // Convert angular velocity (rad/s).
  Send_Data.tx[8] = transition;             // Lower 8 bits of the Z-axis angular velocity.
  Send_Data.tx[7] = transition >> 8;        // Higher 8 bits of the Z-axis angular velocity.

  // Calculate checksum for data integrity.
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // Calculate checksum for the first 9 bytes.

  // Frame end (0x7D).
  Send_Data.tx[10] = FRAME_TAIL; // End of the data frame.

  // Try to send the data over the serial interface if no Ackermann command is active.
  try {
    if (akm_cmd_vel == "none") {
      Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Send data to the robot controller (STM32) via serial port.
    }
  } catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // Error message if serial communication fails.
  }
}

// Publishes IMU sensor data (orientation, angular velocity, and linear acceleration) to a ROS topic
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::msg::Imu Imu_Data_Pub; // Create a new IMU message.

  // Set the timestamp for the IMU data to the current time.
  Imu_Data_Pub.header.stamp = rclcpp::Node::now();

  // Set the IMU frame ID for transformation (used in robot_pose_ekf).
  Imu_Data_Pub.header.frame_id = gyro_frame_id;

  // Copy orientation (quaternion) from the sensor data (Mpu6050).
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x;
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;

  // Set orientation covariance (uncertainty in measurements).
  Imu_Data_Pub.orientation_covariance[0] = 1e6;
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;

  // Copy angular velocity (rad/s) from Mpu6050.
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x;
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;

  // Set angular velocity covariance.
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;

  // Copy linear acceleration (m/s²) from Mpu6050.
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x;
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;

  // Publish the IMU data to the topic.
  imu_publisher->publish(Imu_Data_Pub);
}

// Publishes odometry data (position, velocity, and orientation) in ROS, which is essential for tracking the robot's movement and pose in its environment
void turn_on_robot::Publish_Odom()
{
    // Convert Z-axis rotation to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, Robot_Pos.Z); // Z-axis rotation to quaternion
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    // Create instances for odometry, position, and velocity data
    wheeltec_robot_msg::msg::Data robotpose;
    wheeltec_robot_msg::msg::Data robotvel;
    nav_msgs::msg::Odometry odom;

    // Set current time and frame information for odometry message
    odom.header.stamp = rclcpp::Node::now();
    odom.header.frame_id = odom_frame_id;  // Parent frame (odom)
    odom.child_frame_id = robot_frame_id;  // Child frame (robot)

    // Set robot's position and orientation
    odom.pose.pose.position.x = Robot_Pos.X;
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; // Quaternion orientation

    // Set robot's linear and angular velocity
    odom.twist.twist.linear.x = Robot_Vel.X;
    odom.twist.twist.linear.y = Robot_Vel.Y;
    odom.twist.twist.angular.z = Robot_Vel.Z;

    // Populate custom position and velocity messages
    robotpose.x = Robot_Pos.X;
    robotpose.y = Robot_Pos.Y;
    robotpose.z = Robot_Pos.Z;

    robotvel.x = Robot_Vel.X;
    robotvel.y = Robot_Vel.Y;
    robotvel.z = Robot_Vel.Z;

    // Publish odometry, position, and velocity data
    odom_publisher->publish(odom);
    robotpose_publisher->publish(robotpose);
    robotvel_publisher->publish(robotvel);
}

// Periodically publishes the robot's power supply voltage to a ROS topic, helping monitor the battery or power status.
void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs; // Define message type for voltage
    static float Count_Voltage_Pub = 0;  // Static counter to control the publish rate

    if (Count_Voltage_Pub++ > 10) // Publish after every 10 cycles
    {
        Count_Voltage_Pub = 0;  // Reset counter
        voltage_msgs.data = Power_voltage; // Assign current voltage value to the message
        voltage_publisher->publish(voltage_msgs); // Publish voltage data in volts (V)
    }
}

// Calculates a checksum using a bitwise XOR operation. The checksum is used for data integrity during communication, ensuring that data is correctly transmitted or received.
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0) // Receiving data
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k]; // XOR with each received byte
    }
  }

  if (mode == 1) // Sending data
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k]; // XOR with each sent byte
    }
  }

  return check_sum; // Return the final checksum
}

//  Read and parse sensor data received from the robot's lower computer, validate it, and extract relevant information such as velocities, IMU data, and power voltage.
bool turn_on_robot::Get_Sensor_Data()
{ 
  short transition_16 = 0, j = 0, Header_Pos = 0, Tail_Pos = 0; // Intermediate variables
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE] = {0}; // Buffer for incoming data
  Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)); // Read data from serial port
  
  // Find positions of the frame header and tail
  for (j = 0; j < 24; j++)
  {
    if (Receive_Data_Pr[j] == FRAME_HEADER) Header_Pos = j;
    else if (Receive_Data_Pr[j] == FRAME_TAIL) Tail_Pos = j;    
  }

  if (Tail_Pos == (Header_Pos + 23))
  {
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr)); // Copy valid data directly
  }
  else if (Header_Pos == (1 + Tail_Pos))
  {
    for (j = 0; j < 24; j++)
      Receive_Data.rx[j] = Receive_Data_Pr[(j + Header_Pos) % 24]; // Correct and copy data
  }
  else 
  {
    return false; // Invalid packet
  }    

  // Extract header and tail information
  Receive_Data.Frame_Header = Receive_Data.rx[0];
  Receive_Data.Frame_Tail = Receive_Data.rx[23];

  if (Receive_Data.Frame_Header == FRAME_HEADER) 
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) 
    { 
      // Validate checksum or check if packets are interlaced
      if (Receive_Data.rx[22] == Check_Sum(22, READ_DATA_CHECK) || (Header_Pos == (1 + Tail_Pos))) 
      {
        // Extract data from the received packet
        Receive_Data.Flag_Stop = Receive_Data.rx[1];
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]); 
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]);
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6], Receive_Data.rx[7]); 

        // Extract IMU data
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);   
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]); 
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]); 
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]);  
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]);  
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]);  

        // Convert acceleration units
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;

        // Convert gyroscope units
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        // Read battery voltage
        transition_16 = 0;
        transition_16 |= Receive_Data.rx[20] << 8;
        transition_16 |= Receive_Data.rx[21];  
        Power_voltage = transition_16 / 1000 + (transition_16 % 1000) * 0.001; // Convert millivolts to volts

        return true; // Successful read
      }
    }
  } 
  return false; // Invalid frame or checksum
}

// Continuously read sensor data, compute the robot's position and orientation based on velocities and accelerations, and publish relevant information like IMU data, battery voltage, and odometry
void turn_on_robot::Control()
{
  rclcpp::Time current_time, last_time;
  current_time = rclcpp::Node::now();
  last_time = rclcpp::Node::now();
  
  while (rclcpp::ok())
  {
    current_time = rclcpp::Node::now();
    // Retrieve time interval for displacement calculation
    Sampling_Time = (current_time - last_time).seconds(); 

    // Read and validate data from the lower computer
    if (true == Get_Sensor_Data()) 
    {
      // Calculate displacement in the X direction (m)
      Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
      
      // Calculate displacement in the Y direction (m)
      Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;
      
      // Calculate angular displacement about the Z axis (rad)
      Robot_Pos.Z += Robot_Vel.Z * Sampling_Time;

      // Calculate the three-axis attitude from the IMU data
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                          Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
      
      // Publish IMU data
      Publish_ImuSensor(); // Publish IMU topic
      Publish_Voltage();   // Publish power supply voltage topic
      Publish_Odom();      // Publish odometry data

      rclcpp::spin_some(this->get_node_base_interface());
    }

    last_time = current_time; // Update last_time for the next iteration
  }
}

// The constructor sets up parameters, publishers, subscribers, and a serial connection to communicate with the robot's hardware
turn_on_robot::turn_on_robot()
: rclcpp::Node("wheeltec_robot")
{
  // Initialise data structures to zero
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  // Declare parameters
  this->declare_parameter<int>("serial_baud_rate");
  this->declare_parameter<std::string>("usart_port_name", "/dev/ttyACM1");
  this->declare_parameter<std::string>("cmd_vel", "cmd_vel");
  this->declare_parameter<std::string>("akm_cmd_vel", "ackermann_cmd");
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<std::string>("gyro_frame_id", "gyro_link");

  // Get parameters
  this->get_parameter("serial_baud_rate", serial_baud_rate);
  this->get_parameter("usart_port_name", usart_port_name);
  this->get_parameter("cmd_vel", cmd_vel);
  this->get_parameter("akm_cmd_vel", akm_cmd_vel);
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("robot_frame_id", robot_frame_id);
  this->get_parameter("gyro_frame_id", gyro_frame_id);

  // Create publishers
  odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  imu_publisher = create_publisher<sensor_msgs::msg::Imu>("mobile_base/sensors/imu_data", 10);
  voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 1);
  robotpose_publisher = create_publisher<wheeltec_robot_msg::msg::Data>("robotpose", 10);
  robotvel_publisher = create_publisher<wheeltec_robot_msg::msg::Data>("robotvel", 10);
  tf_bro = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Create subscriptions
  Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel, 100, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));

  Akm_Cmd_Vel_Sub = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      akm_cmd_vel, 100, std::bind(&turn_on_robot::Akm_Cmd_Vel_Callback, this, _1));

  try
  { 
    // Initialise and open the serial port
    Stm32_Serial.setPort(usart_port_name);
    Stm32_Serial.setBaudrate(serial_baud_rate);
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000);
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open();
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR(this->get_logger(), "wheeltec_robot cannot open serial port %s: %s", usart_port_name.c_str(), e.what());
  }

  if (Stm32_Serial.isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "wheeltec_robot serial port opened: %s", usart_port_name.c_str());
  }
}

// The destructor is executed only once and called by the system when an object ends its life cycle. Sends a stop command to the robot, closes the serial port
turn_on_robot::~turn_on_robot()
{
  // Send stop command to the lower machine before destruction
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  // Set target velocities to zero for X, Y, and Z axes
  Send_Data.tx[3] = 0;  
  Send_Data.tx[4] = 0;  
  Send_Data.tx[5] = 0;  
  Send_Data.tx[6] = 0;  
  Send_Data.tx[7] = 0;  
  Send_Data.tx[8] = 0;  
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // Checksum
  Send_Data.tx[10] = FRAME_TAIL; 

  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Send data to the serial port  
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to send data through serial port"); // Log error if sending fails
  }
  
  Stm32_Serial.close(); // Close the serial port  
  RCLCPP_INFO(this->get_logger(), "Shutting down"); // Log shutdown message
}





