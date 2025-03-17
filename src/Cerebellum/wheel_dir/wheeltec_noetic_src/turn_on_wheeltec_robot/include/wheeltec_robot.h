
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <turtlesim/Spawn.h>
using namespace std;

//cout相关，用于打印带颜色的信息
#define RESET   string("\033[0m")
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define PURPLE  "\033[35m"
#define CYAN    "\033[36m"

//Macro definition
//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI                3.1415926f //PI //圆周率

//自动回充相关
#define AutoCharge_HEADER      0X7C //Frame_header //自动回充数据帧头
#define AutoCharge_TAIL        0X7F //Frame_tail   //自动回充数据帧尾
#define AutoCharge_DATA_SIZE    8   //下位机发送过来的自动回充数据的长度

//Relative to the range set by the IMU gyroscope, the range is ±500°, corresponding data range is ±32768
//The gyroscope raw data is converted in radian (rad) units, 1/65.5/57.30=0.00026644
//与IMU陀螺仪设置的量程有关，量程±500°，对应数据范围±32768
//陀螺仪原始数据转换位弧度(rad)单位，1/65.5/57.30=0.00026644
#define GYROSCOPE_RATIO   0.00026644f
//Relates to the range set by the IMU accelerometer, range is ±2g, corresponding data range is ±32768
//Accelerometer original data conversion bit m/s^2 units, 32768/2g=32768/19.6=1671.84
//与IMU加速度计设置的量程有关，量程±2g，对应数据范围±32768
//加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
#define ACCEl_RATIO       1671.84f

extern sensor_msgs::Imu Mpu6050; //External variables, IMU topic data //外部变量，IMU话题数据

//Covariance matrix for speedometer topic data for robt_pose_ekf feature pack
//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0,
                                              0, 1e-3,    0,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0,  1e3 };

const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0,
                                              0, 1e-3, 1e-9,   0,   0,    0,
                                              0,    0,  1e6,   0,   0,    0,
                                              0,    0,    0, 1e6,   0,    0,
                                              0,    0,    0,   0, 1e6,    0,
                                              0,    0,    0,   0,   0, 1e-9} ;

//Data structure for speed and position
//速度、位置数据结构体
typedef struct __Vel_Pos_Data_
{
    float X;
    float Y;
    float Z;
}Vel_Pos_Data;

//IMU data structure
//IMU数据结构体
typedef struct __MPU6050_DATA_
{
    short accele_x_data;
    short accele_y_data;
    short accele_z_data;
    short gyros_x_data;
    short gyros_y_data;
    short gyros_z_data;

}MPU6050_DATA;

//The structure of the ROS to send data to the down machine
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_
{
        uint8_t tx[SEND_DATA_SIZE];
        float X_speed;
        float Y_speed;
        float Z_speed;
        unsigned char Frame_Tail;
}SEND_DATA;

//The structure in which the lower computer sends data to the ROS
//下位机向ROS发送数据的结构体
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
}RECEIVE_DATA;

//下位机发送的自动回充相关数据结构体
typedef struct _RECEIVE_AutoCharge_DATA_
{
        uint8_t rx[AutoCharge_HEADER];  //8字节
        unsigned char Frame_Header;     //帧头
        unsigned char Frame_Tail;       //帧尾
}RECEIVE_AutoCharge_DATA;

//The robot chassis class uses constructors to initialize data, publish topics, etc
//机器人底盘类，使用构造函数初始化数据和发布话题等
class turn_on_robot
{
    public:
        turn_on_robot();  //Constructor //构造函数
        ~turn_on_robot(); //Destructor //析构函数
        void Control();   //Loop control code //循环控制代码
        serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象
    private:
        ros::NodeHandle n;           //Create a ROS node handle //创建ROS节点句柄
        ros::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
        float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)

        ros::Subscriber Cmd_Vel_Sub,Recharge_Flag_Sub, Red_Vel_Sub; //Initialize the topic subscriber //初始化话题订阅者
        //The speed topic subscribes to the callback function
        //速度话题订阅回调函数
        void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);

        //Ir docking speed topic subscription callback function
        //红外对接速度话题订阅回调函数
        void Red_Vel_Callback(const geometry_msgs::Twist &twist_aux);
        //Automatic recharge mode flag bit subscription callback function
        //自动回充模式标志位订阅回调函数
        void Recharge_Flag_Callback(const std_msgs::Int8 &Recharge_Flag);

        bool Set_Charge_Callback(turtlesim::Spawn::Request &req,turtlesim::Spawn::Response &res);

        ros::Publisher odom_publisher, imu_publisher, voltage_publisher,Charging_publisher, Charging_current_publisher, RED_publisher; //Initialize the topic publisher //初始化话题发布者
        ros::ServiceServer SetCharge_Service;
        void Publish_Odom();      //Pub the speedometer topic //发布里程计话题
        void Publish_ImuSensor(); //Pub the IMU sensor topic //发布IMU传感器话题
        void Publish_Voltage();   //Pub the power supply voltage topic //发布电源电压话题
        void Publish_Charging();        //Pub a topic about whether the robot is charging //发布机器人是否在充电的话题
        void Publish_ChargingCurrent(); //Pub the charging current topic //发布充电电流话题
        void Publish_RED();             //Pub the topic whether the robot finds the infrared signal (charging station) //发布机器人是否寻找到红外信号(充电桩)的话题

        //从串口(ttyUSB)读取运动底盘速度、IMU、电源电压数据
        //Read motion chassis speed, IMU, power supply voltage data from serial port (ttyUSB)
        bool Get_Sensor_Data();
        bool Get_Sensor_Data_New();
        unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        unsigned char Check_Sum_AutoCharge(unsigned char Count_Number,unsigned char mode); //BBC check function //BBC校验函数
        short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);  //IMU data conversion read //IMU数据转化读取
        float Odom_Trans(uint8_t Data_High,uint8_t Data_Low); //Odometer data is converted to read //里程计数据转化读取

        string usart_port_name, robot_frame_id, gyro_frame_id, odom_frame_id,robot_type; //Define the related variables //定义相关变量
        int serial_baud_rate;      //Serial communication baud rate //串口通信波特率
        RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
        RECEIVE_AutoCharge_DATA Receive_AutoCharge_Data;  //串口接收自动回充数据结构体
        SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体

        Vel_Pos_Data Robot_Pos;    //The position of the robot //机器人的位置
        Vel_Pos_Data Robot_Vel;    //The speed of the robot //机器人的速度
        MPU6050_DATA Mpu6050_Data; //IMU data //IMU数据
        float Power_voltage;       //Power supply voltage //电源电压
        int8_t AutoRecharge=0;     //Flag bit of the automatic recharge command //自动回充命令标志位
        bool Charging=0;           //Whether the robot is charging the flag bit //机器人是否在充电的标志位
        float Charging_Current=0;  //Charging_Current //充电电流
        uint8_t Red=0;                //Whether the robot finds the marker bit of infrared signal (charging pile)  //机器人是否寻找到红外信号(充电桩)的标志位
        float odom_x_scale, odom_y_scale, odom_z_scale_positive, odom_z_scale_negative;       //Odometer correction parameters //里程计修正参数
};
#endif
