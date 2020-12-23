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

#include "bw_auto_dock/getDockPosition.h"

namespace bw_auto_dock
{
CaculateDockPosition::CaculateDockPosition(double grid_length, std::string frame_id, std::string dock_station_filename,
                                           DockController* dock_controler, StatusPublisher* bw_status)
{
    mdock_controler_ = dock_controler;
    mbw_status_ = bw_status;
    mdock_station_filename_ = dock_station_filename;
    mlocal_grid_ = new LocalGrid(0.1, grid_length / 2.0 / 0.1, frame_id);
    station_distance_ = 0.8;
}
bool CaculateDockPosition::getDockPosition(float (&station_pose1)[3], float (&station_pose2)[3])
{
    //获取充电桩参考点
    boost::mutex::scoped_lock lock(mMutex_);
    float theta = 0;
    std::string dbfile_path = mdock_station_filename_;//std::string(std::getenv("HOME")) + std::string("/slamdb") + std::string("/") + mdock_station_filename_;
    if (!boost::filesystem::exists(dbfile_path.c_str()))
    {
        return false;
    }
    else
    {
        std::ifstream dock_station_file(dbfile_path);
        if (dock_station_file.is_open())
        {
            dock_station_file >> station_pose1[0];
            dock_station_file >> station_pose1[1];
            dock_station_file >> station_pose2[0];
            dock_station_file >> station_pose2[1];
            dock_station_file >> theta;
            dock_station_file >> theta;
            dock_station_file >> theta;
        }
        else
        {
            return false;
        }
    }
    station_pose1[2] = theta;
    station_pose2[2] = theta;
    return true;
}

void CaculateDockPosition::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub1 =
        nodeHandler.subscribe("/bw_auto_dock/dockposition_save", 1, &CaculateDockPosition::updateMapsaveFlag, this);
    ros::Rate r(10);
    int num = 0;
    bool dock_postion_update_ = false;
    while (ros::ok())
    {
        num++;
        if (num % 100 == 0)
        {
            mlocal_grid_->runPub();
        }
        if (num % 100 == 0 && dock_postion_update_)
        {
            // 10秒自动保存一次
            dock_postion_update_ = false;
            this->saveDockPositon();
            float station_pose1[2], station_pose2[2];
        }
        DOCK_POSITION sensor_value = mbw_status_->get_dock_position();
        float ir_pose[3];

        if (mdock_controler_->getIRPose(ir_pose) && sensor_value != DOCK_POSITION::not_found)
        {
            if (!mlocal_grid_->get_ready_flag())
            {
                float center[3];
                center[0] = ir_pose[0];
                center[1] = ir_pose[1];
                center[2] = ir_pose[2];
                mlocal_grid_->setOrigin(center);
            }
            mlocal_grid_->update_clear(ir_pose);
            mlocal_grid_->update_sensor(sensor_value, ir_pose);
            // if (mbw_status_->sensor_status.power > 9.0)
            // {
            //     if(mbw_status_->get_charge_status() == CHARGE_STATUS::freed)
            //     {
            //       //触发充电桩
            //       if (mlocal_grid_->set_dock_position(ir_pose))
            //           dock_postion_update_ = true;
            //       // ROS_INFO("dock_realposition_set %d %f %f %f",num,ir_pose[0],ir_pose[1],ir_pose[2]);
            //     }
            // }
        }

        ros::spinOnce();
        r.sleep();
    }
}

void CaculateDockPosition::saveDockPositon()
{
    boost::mutex::scoped_lock lock(mMutex_);
    boost::filesystem::path p(mdock_station_filename_);
    boost::filesystem::path dir = p.parent_path();
    std::string dbfile_rootpath = p.parent_path().string();
    float dock_pose[3], station_pose1[2], station_pose2[2];
    if (mlocal_grid_->get_dock_position(dock_pose))
    {
        ROS_INFO("dock_position_saved  %f %f %f", dock_pose[0], dock_pose[1], dock_pose[2]);
        //(station_distance_,station_distance_) (station_distance_,-station_distance_)
        station_pose1[0] =
            dock_pose[0] + station_distance_ * cos(dock_pose[2]) - 0.5 * station_distance_ * sin(dock_pose[2]);
        station_pose1[1] =
            dock_pose[1] + station_distance_ * sin(dock_pose[2]) + 0.5 * station_distance_ * cos(dock_pose[2]);

        station_pose2[0] =
            dock_pose[0] + station_distance_ * cos(dock_pose[2]) + 0.5 * station_distance_ * sin(dock_pose[2]);
        station_pose2[1] =
            dock_pose[1] + station_distance_ * sin(dock_pose[2]) - 0.5 * station_distance_ * cos(dock_pose[2]);

        if (!boost::filesystem::exists(dbfile_rootpath.c_str()))
        {
            boost::filesystem::create_directories(dbfile_rootpath.c_str());
        }
        std::ofstream dock_station_file;
        dock_station_file.open(mdock_station_filename_);
        dock_station_file << station_pose1[0] << " " << station_pose1[1] << std::endl
                          << station_pose2[0] << " " << station_pose2[1] << std::endl;
        dock_station_file << dock_pose[0] << " " << dock_pose[1] << " " << dock_pose[2] << std::endl;
        dock_station_file.close();
    }
}

void CaculateDockPosition::updateMapsaveFlag(const std_msgs::Bool& currentFlag)
{
    if (currentFlag.data)
    {
        float ir_pose[3];
        DOCK_POSITION sensor_value = mbw_status_->get_dock_position();
        if (mdock_controler_->getIRPose(ir_pose) && sensor_value != DOCK_POSITION::not_found)
        {
            mlocal_grid_->set_dock_position(ir_pose);
            this->saveDockPositon();
        }

    }
}
}  // namespace bw_auto_dock
