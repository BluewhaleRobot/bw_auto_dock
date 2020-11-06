/******************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Bluewhale Robot
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Author: Xie fusheng, Randoms
 *******************************************************************************/

#include "bw_auto_dock/AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "bw_auto_dock/DockController.h"
#include "bw_auto_dock/StatusPublisher.h"
#include "bw_auto_dock/getDockPosition.h"
#include <std_msgs/String.h>

using namespace std;

inline bool exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bw_auto_dock_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB004");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    ROS_INFO_STREAM("port:" << port << " baud:" << baud);
    //获取小车机械参数
    double back_distance = 0;
    ros::param::param<double>("~back_distance", back_distance, 0.30);

    double crash_distance;
    ros::param::param<double>("~crash_distance", crash_distance, 70);

    //获取小车控制参数
    double max_linearspeed, max_rotspeed;

    ros::param::param<double>("~max_linearspeed", max_linearspeed, 0.2);
    ros::param::param<double>("~max_rotspeed", max_rotspeed, 0.2);

    double kp, ki, kd;
    ros::param::param<double>("~back_dock_kp", kp, 0.2);
    ros::param::param<double>("~back_dock_ki", ki, 0.04);
    ros::param::param<double>("~back_dock_kd", kd, 0.0);

    std::string global_frame_id;
    ros::param::param<std::string>("~global_frame_id", global_frame_id, "odom");

    std::string station_filename;
    ros::param::param<std::string>("~station_filename", station_filename, "dock_station.txt");

    double grid_length;
    ros::param::param<double>("~grid_length", grid_length, 4.0);

    int barDetectFlag;
    ros::param::param<int>("~barDetectFlag", barDetectFlag, 1);

    double power_scale;
    ros::param::param<double>("~power_scale", power_scale, 1.0);

    double power_threshold;
    ros::param::param<double>("~power_threshold", power_threshold, 41.0);

    double kp_theta_set, kd_theta_set, ki_theta_set;
    ros::param::param<double>("~kp_theta_set", kp_theta_set, 0.4);
    ros::param::param<double>("~kd_theta_set", kd_theta_set, 0.8);
    ros::param::param<double>("~ki_theta_set", ki_theta_set, 1.0);

    double kp_x_set, kd_x_set, ki_x_set;
    ros::param::param<double>("~kp_x_set", kp_x_set, 0.4);
    ros::param::param<double>("~kd_x_set", kd_x_set, 0.8);
    ros::param::param<double>("~ki_x_set", ki_x_set, 10.0);

    double max_x_speed, max_theta_speed;
    ros::param::param<double>("~max_x_speed", max_x_speed, 0.3);
    ros::param::param<double>("~max_theta_speed", max_theta_speed, 0.3);

    bw_auto_dock::StatusPublisher bw_status(crash_distance,power_scale);

    ros::NodeHandle mNH;
    ros::Publisher audio_pub = mNH.advertise<std_msgs::String>("/xiaoqiang_tts/text", 1, true);
    try
    {
        CallbackAsyncSerial serial(port, baud);

        serial.setCallback(boost::bind(&bw_auto_dock::StatusPublisher::Update, &bw_status, _1, _2));

        bw_auto_dock::DockController bw_controler(back_distance, max_linearspeed, max_rotspeed,crash_distance,barDetectFlag,global_frame_id, &bw_status, &serial);
        boost::thread bw_controlerThread(&bw_auto_dock::DockController::run, &bw_controler);
        bw_controler.setDockPid(kp, ki, kd, kp_theta_set, kd_theta_set, ki_theta_set,kp_x_set, kd_x_set, ki_x_set, max_theta_speed, max_x_speed);
        bw_controler.setPowerParam(power_threshold);
        //计算充电桩位置
        bw_auto_dock::CaculateDockPosition caculate_DockPosition(grid_length, global_frame_id, station_filename,
                                                                 &bw_controler, &bw_status);
        boost::thread caculate_DockPositionThread(&bw_auto_dock::CaculateDockPosition::run, &caculate_DockPosition);
        bw_controler.setDockPositionCaculate(&caculate_DockPosition);

        // send reset cmd
        char resetCmd[] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I' };
        serial.write(resetCmd, 5);

        ros::Rate r(30);  //发布周期为50hz
        while (ros::ok())
        {
            if (serial.errorStatus() || serial.isOpen() == false)
            {
                ROS_ERROR_STREAM("Error: serial port closed unexpectedly");
                break;
            }
            bw_status.Refresh();  //定时发布状态
            bw_controler.dealing_status(false);
            r.sleep();
        }

    quit:
        serial.close();
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Open " << port << " failed.");
      ROS_ERROR_STREAM("Exception: " << e.what());
      // 检查串口设备是否存在
      if (!exists(port))
      {
          // 发送语音提示消息
          std_msgs::String audio_msg;
          audio_msg.data = "未发现充电串口设备，请检查充电串口连接";
          audio_pub.publish(audio_msg);
      }
    }

    ros::shutdown();
    return 0;
}
