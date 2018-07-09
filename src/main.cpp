
#include "bw_auto_dock/AsyncSerial.h"

#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "bw_auto_dock/DockControler.h"
#include "bw_auto_dock/StatusPublisher.h"
#include "bw_auto_dock/getDockPosition.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bw_auto_dock_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/ttyUSB004");
    int baud;
    ros::param::param<int>("~baud", baud, 115200);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;
    //获取小车机械参数
    double back_distance=0;
    ros::param::param<double>("~back_distance", back_distance, 0.30);
    bw_auto_dock::StatusPublisher bw_status;

    //获取小车控制参数
    double max_linearspeed,max_rotspeed;

    ros::param::param<double>("~max_linearspeed", max_linearspeed, 0.2);
    ros::param::param<double>("~max_rotspeed", max_rotspeed, 0.2);

    double kp,ki,kd;
    ros::param::param<double>("~back_dock_kp",kp, 0.2);
    ros::param::param<double>("~back_dock_ki",ki, 0.04);
    ros::param::param<double>("~back_dock_kd",kd, 0.0);

    std::string odom_frame_id;
    ros::param::param<std::string>("~odom_frame_id", odom_frame_id, "odom");

    std::string station_filename;
    ros::param::param<std::string>("~station_filename", station_filename, "dock_station.txt");

    double grid_length;
    ros::param::param<double>("~grid_length",grid_length, 4.0);

    try {
        CallbackAsyncSerial serial(port,baud);

        serial.setCallback(boost::bind(&bw_auto_dock::StatusPublisher::Update,&bw_status,_1,_2));

        bw_auto_dock::DockControler bw_controler(back_distance,max_linearspeed,max_rotspeed,&bw_status,&serial);
        boost::thread bw_controlerThread(&bw_auto_dock::DockControler::run,&bw_controler);
        bw_controler.setDockPid(kp,ki,kd);

        //计算充电桩位置
        bw_auto_dock::CaculateDockPosition caculate_DockPosition(grid_length,odom_frame_id,station_filename,&bw_controler,&bw_status);
        boost::thread caculate_DockPositionThread(&bw_auto_dock::CaculateDockPosition::run,&caculate_DockPosition);
        bw_controler.setDockPositionCaculate(&caculate_DockPosition);

        // send reset cmd
        char resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};
        serial.write(resetCmd, 5);

        ros::Rate r(30);//发布周期为50hz
        while (ros::ok())
        {
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port closed unexpectedly"<<endl;
                break;
            }
            bw_status.Refresh();//定时发布状态
            bw_controler.dealing_status();
            r.sleep();
            //cout<<"run"<<endl;
        }

        quit:
        serial.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }

    ros::shutdown();
    return 0;
}
