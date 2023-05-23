#include <ros/ros.h>

#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include "serial/serial.h"
#include <math.h>
#include <chrono>

#include <fstream>

#include <sensor_msgs/Imu.h>

sensor_msgs::Imu imu_data;
// 设置串口参数
serial::Serial imu_ser("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(100));

/**
 * @brief 用于解算imu回传四元数数据
 * @param data 回传报文
 * @param start_bit 四元数数据的起始位
 * @returns a q1/q2/q3/q4
*/
double read_qua(std::string data, int start_bit)
{
   char dot = '.';
   char a_1 = data[start_bit];
   char a_2 = data[start_bit + 1];
   char a_3[6];

    for (int i = 0; i < 6; i++)
    {
        a_3[i] = data[i + start_bit + 2];
    }

    bool a_1_ = a_1 - '0';
    int a_2_ = a_2 - '0';
    int a_3_ = a_3[0] - '0';
    int a_4_ = a_3[1] - '0';
    int a_5_ = a_3[2] - '0';
    int a_6_ = a_3[3] - '0';
    int a_7_ = a_3[4] - '0';
    int a_8_ = a_3[5] - '0';
    int decimal = dot == '.' ? 1 : 0;
    double a;
    if (a_1_)
    {
        a = (a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal) + a_7_ * pow(100000, -1 * decimal) + a_8_ * pow(1000000, -1 * decimal));
    }
    else
    {
        a = -(a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal) + a_7_ * pow(100000, -1 * decimal) + a_8_ * pow(1000000, -1 * decimal));        
    }
    return a;
}

/**
 * @brief 用于解算imu回传加速度数据
 * @param data 回传报文
 * @param start_bit 加速度数据的起始位
 * @returns a 某轴加速度
*/
double read_acc(std::string data, int start_bit)
{
    char dot = '.';
    char a_1 = data[start_bit];
    char a_2 = data[start_bit + 1];
    char a_3[4];
    for (int i = 0; i < 4; i++)
    {
        a_3[i] = data[i + start_bit + 2];
    }
    bool a_1_ = a_1 - '0';
    int a_2_ = a_2 - '0';
    int a_3_ = a_3[0] - '0';
    int a_4_ = a_3[1] - '0';
    int a_5_ = a_3[2] - '0';
    int a_6_ = a_3[3] - '0';
    int decimal = dot == '.' ? 1 : 0;
    double a;
    if (a_1_)
    {
        a = (a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));
    }
    else
    {
        a = -(a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));
    }

    // float a = (a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));

    return a;
}

/**
 * @brief 用于解算imu回传角度/角速度数据
 * @param data 回传报文
 * @param start_bit 角度/角速度数据的起始位
 * @returns g 某轴角度/角速度
*/
double read_gyo(std::string data, int start_bit)
{
    char dot = '.';
    char g_1 = data[start_bit];
    char g_2[3];
    for (int i = 0; i < 3; i++)
    {
        g_2[i] = data[i + start_bit + 1];
    }
    char g_3[2];
    for (int i = 0; i < 2; i++)
    {
        g_3[i] = data[i + start_bit + 4];
    }
    bool g_1_ = g_1 - '0';
    int g_2_ = g_2[0] - '0';
    int g_3_ = g_2[1] - '0';
    int g_4_ = g_2[2] - '0';

    int g_5_ = g_3[0] - '0';
    int g_6_ = g_3[1] - '0';

    int decimal = dot == '.' ? 1 : 0;
    double g;
    if (g_1_)
    {
        g = (g_2_ * 100 + g_3_ * 10 + g_4_ + g_5_ * pow(10, -1 * decimal) + g_6_ * pow(100, -1 * decimal));
    }
    else
    {
        g = -(g_2_ * 100 + g_3_ * 10 + g_4_ + g_5_ * pow(10, -1 * decimal) + g_6_ * pow(100, -1 * decimal));
    }

    // float a = (a_2_ + a_3_ * pow(10, -1 * decimal) + a_4_ * pow(100, -1 * decimal) + a_5_ * pow(1000, -1 * decimal) + a_6_ * pow(10000, -1 * decimal));

    return g;

}



using namespace std;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "bw_serial_node");

    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1000);

    // 待发送的报文
    uint8_t imu_send_data[] = {0x77, 0x04, 0x00, 0x59, 0x5D};
    int imu_send_size = sizeof(imu_send_data) / sizeof(uint8_t);

    // 接收回传报文
    std::string imu_response;

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        // 设置imu消息的时间戳
        imu_data.header.stamp = ros::Time::now();
        // 设置imu的坐标系
        imu_data.header.frame_id = "imu";

        // ----------------------------报文收发----------------------------
        // 将待发送的报文发送至串口
        imu_ser.write(imu_send_data, imu_send_size);

        // 接收回传报文
        vector<uint8_t> imu_recv_data;

        // 读取串口返回的数据，大小为48个bytes
        imu_ser.read(imu_recv_data, 48);

        // 如果接收到的数据不足48bytes，则跳过此次循环
        if (imu_recv_data.size() != 48)
        {
            continue;
        }

        // 将回传数据转为字符串
        imu_response.resize(48*2);
        static constexpr char hex_digits[] = "0123456789ABCDEF";
        for (std::size_t i = 0; i < 48; ++i)
        {
            imu_response[i * 2] = hex_digits[imu_recv_data[i] >> 4];
            imu_response[i * 2 + 1] = hex_digits[imu_recv_data[i] & 0xF];
        }

        // 给四元数赋值，由于只需要加表与角速度的原始数据，因此将其设为定值即可
        imu_data.orientation.w = 1;
        imu_data.orientation.x = 0;
        imu_data.orientation.y = 0;
        imu_data.orientation.z = 0;
        /* imu_data.orientation.w = read_qua(imu_response, 62);
        imu_data.orientation.x = read_qua(imu_response, 70);
        imu_data.orientation.y = read_qua(imu_response, 78);
        imu_data.orientation.z = read_qua(imu_response, 86); */
        // 给加速度赋值，bw传感器中反馈的是以G值为单位的加速度数据
        imu_data.linear_acceleration.x = read_acc(imu_response, 26) * 9.79134;
        imu_data.linear_acceleration.y = read_acc(imu_response, 32) * 9.79134;
        imu_data.linear_acceleration.z = read_acc(imu_response, 38) * 9.79134;
        // 给角速度赋值，bw传感器中反馈的属于与加速度所在坐标系存在偏差，因此进行如下变换
        imu_data.angular_velocity.x = - read_gyo(imu_response, 50) * M_PI /180;
        imu_data.angular_velocity.y = read_gyo(imu_response, 44) * M_PI /180;
        imu_data.angular_velocity.z = - read_gyo(imu_response, 56) * M_PI /180;
        // 发布话题
        imu_pub.publish(imu_data);
        // 清理串口
        imu_ser.flushInput();
        imu_ser.flushOutput();
    }
    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}
