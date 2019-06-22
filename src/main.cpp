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

using namespace std;

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

    //充电导航点到充电部位的距离
    double station_distance = 0;
    ros::param::param<double>("~station_distance", station_distance, 0.80);

    double crash_distance;
    ros::param::param<double>("~crash_distance", crash_distance, 70);

    //获取小车控制参数
    double y_min_set, x_min_set,theta_min_set;

    ros::param::param<double>("~y_min_set", y_min_set, 0.05);
    ros::param::param<double>("~x_min_set", x_min_set, 0.3);
    ros::param::param<double>("~theta_min_set", theta_min_set, 0.3);

    double kp_theta_set, kd_theta_set, ki_theta_set;
    ros::param::param<double>("~kp_theta_set", kp_theta_set, 0.4);
    ros::param::param<double>("~kd_theta_set", kd_theta_set, 0.8);
    ros::param::param<double>("~ki_theta_set", ki_theta_set, 1.0);

    double kp_y_set, kd_y_set, ki_y_set;
    ros::param::param<double>("~kp_y_set", kp_y_set, 0.4);
    ros::param::param<double>("~kd_y_set", kd_y_set, 0.8);
    ros::param::param<double>("~ki_y_set", ki_y_set, 10.0);

    double kp_x_set, kd_x_set, ki_x_set;
    ros::param::param<double>("~kp_x_set", kp_x_set, 0.4);
    ros::param::param<double>("~kd_x_set", kd_x_set, 0.8);
    ros::param::param<double>("~ki_x_set", ki_x_set, 10.0);

    double max_x_speed, max_y_speed, max_theta_speed;
    ros::param::param<double>("~max_x_speed", max_x_speed, 0.3);
    ros::param::param<double>("~max_y_speed", max_y_speed, 0.3);
    ros::param::param<double>("~max_theta_speed", max_theta_speed, 0.3);

    double goal_x_error, goal_y_error, goal_theta_error;
    ros::param::param<double>("~goal_x_error", goal_x_error, 0.01);
    ros::param::param<double>("~goal_y_error", goal_y_error, 0.01);
    ros::param::param<double>("~goal_theta_error", goal_theta_error, 0.02);

    std::string global_frame_id;
    ros::param::param<std::string>("~global_frame_id", global_frame_id, "odom");

    std::string station_filename;
    ros::param::param<std::string>("~station_filename", station_filename, "dock_station.txt");


    int barDetectFlag;
    ros::param::param<int>("~barDetectFlag", barDetectFlag, 1);

    double power_scale;
    ros::param::param<double>("~power_scale", power_scale, 1.0);

    double power_threshold;
    ros::param::param<double>("~power_threshold", power_threshold, 41.0);

    bw_auto_dock::StatusPublisher bw_status(crash_distance,power_scale);
    try
    {

        bw_auto_dock::DockController bw_controler(back_distance,0.2,0.3,crash_distance,barDetectFlag,global_frame_id, &bw_status);
        boost::thread bw_controlerThread(&bw_auto_dock::DockController::run, &bw_controler);
        bw_controler.setDockPid(kp_theta_set, kd_theta_set, ki_theta_set,kp_y_set, kd_y_set, ki_y_set, kp_x_set, kd_x_set, ki_x_set);
        bw_controler.setScaleParam(theta_min_set, y_min_set, x_min_set, max_x_speed, max_y_speed, max_theta_speed, goal_theta_error, goal_y_error,goal_x_error);
        bw_controler.setPowerParam(power_threshold);
        //计算充电桩位置
        bw_auto_dock::CaculateDockPosition caculate_DockPosition(station_distance, global_frame_id, station_filename,
                                                                 &bw_controler, &bw_status);
        boost::thread caculate_DockPositionThread(&bw_auto_dock::CaculateDockPosition::run, &caculate_DockPosition);
        bw_controler.setDockPositionCaculate(&caculate_DockPosition);

        ros::Rate r(30);  //发布周期为50hz
        while (ros::ok())
        {
            bw_status.Refresh();  //定时发布状态
            bw_controler.dealing_status();
            r.sleep();
        }

    }
    catch (std::exception& e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
    }

    ros::shutdown();
    return 0;
}
