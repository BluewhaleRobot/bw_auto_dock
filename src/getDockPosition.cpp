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
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MARKER_ID_DETECTION 0

namespace bw_auto_dock
{
CaculateDockPosition::CaculateDockPosition(double station_distance, std::string frame_id, std::string dock_station_filename,
                                           DockController* dock_controler, StatusPublisher* bw_status):tf2_(tf2_buffer_)
{
    mdock_controler_ = dock_controler;
    mbw_status_ = bw_status;
    mdock_station_filename_ = dock_station_filename;
    station_distance_ = station_distance;

    dock_realposition_[0] = 0.0;
    dock_realposition_[1] = 0.0;
    dock_realposition_[2] = 0.0;
    dock_realposition_ready_ = false;
    dock_mark_ready_ = false;
    tf2::toMsg(tf2::Transform::getIdentity(), mMarker_pose_base_.pose);
    tf2::toMsg(tf2::Transform::getIdentity(), mMarker_pose_camera_.pose);
    mTf_flag_ = false;
    dock_base_link_position_[0] = 0.0;
    dock_base_link_position_[1] = 0.0;
    dock_base_link_position_[2] = 0.0;
}

bool CaculateDockPosition::getDockPosition(float (&station_pose1)[2], float (&station_pose2)[2])
{
    //获取充电桩参考点
    boost::mutex::scoped_lock lock(mMutex_);
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
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool CaculateDockPosition::getDockPosition(float (&station_pose_baselink_goal)[3])
{
    //获取充电桩参考点
    boost::mutex::scoped_lock lock(mMutex_);
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
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[0];
            dock_station_file >> station_pose_baselink_goal[1];
            dock_station_file >> station_pose_baselink_goal[2];
        }
        else
        {
            return false;
        }
    }
    return true;
}

void CaculateDockPosition::run()
{
    ros::NodeHandle nodeHandler;
    ros::Subscriber sub1 = nodeHandler.subscribe("/bw_auto_dock/dockposition_save", 1, &CaculateDockPosition::updateMapsaveFlag, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/ar_pose_marker", 1, &CaculateDockPosition::updateMarkerPose, this);
    ros::Rate r(30);
    int num = 0;
    bool dock_postion_update_ = false;
    int lost_num=0;
    while (ros::ok())
    {
      num++;
      if (num % 300 == 0 && dock_postion_update_)
      {
          // 10秒自动保存一次
          dock_postion_update_ = false;
          this->saveDockPositon();
          float station_pose1[2], station_pose2[2];
      }
      float ir_pose[3];

      // if (mdock_controler_->getIRPose(ir_pose))
      // {
      //     if (mbw_status_->sensor_status.power > 9.0)
      //     {
      //         if(mbw_status_->get_charge_status() == CHARGE_STATUS::freed)
      //         {
      //           //触发充电桩
      //           if (this->set_dock_position(ir_pose))
      //           {
      //             this->getMarkerPose(dock_base_link_position_);
      //             dock_postion_update_ = true;
      //           }
      //
      //           // ROS_INFO("dock_realposition_set %d %f %f %f",num,ir_pose[0],ir_pose[1],ir_pose[2]);
      //         }
      //     }
      // }
      {
        boost::mutex::scoped_lock lock(mMutex_);
        if(!dock_mark_update_)
        {
          lost_num ++;
        }
        else
        {
          dock_mark_update_ = false;
          lost_num = 0;
        }

        if(lost_num>=10)
        {
          dock_mark_ready_ = false;
        }
      }
      ros::spinOnce();
      r.sleep();
    }
}

void CaculateDockPosition::saveDockPositon()
{
    boost::filesystem::path p(mdock_station_filename_);
    boost::filesystem::path dir = p.parent_path();
    std::string dbfile_rootpath = p.parent_path().string();
    float dock_pose[3], station_pose1[2], station_pose2[2];
    if (this->get_dock_position(dock_pose))
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

        {
            boost::mutex::scoped_lock lock(mMutex_);
            if (!boost::filesystem::exists(dbfile_rootpath.c_str()))
            {
                boost::filesystem::create_directories(dbfile_rootpath.c_str());
            }
            std::ofstream dock_station_file;
            dock_station_file.open(mdock_station_filename_);
            dock_station_file << station_pose1[0] << " " << station_pose1[1] << std::endl
                              << station_pose2[0] << " " << station_pose2[1] << std::endl;
            dock_station_file << dock_pose[0] << " " << dock_pose[1] << " " << dock_pose[2] << std::endl;
            dock_station_file << dock_base_link_position_[0] << " " << dock_base_link_position_[1] << " " << dock_base_link_position_[2] << std::endl;
            dock_station_file.close();
        }

    }
}

void CaculateDockPosition::updateMapsaveFlag(const std_msgs::Bool& currentFlag)
{
    if (currentFlag.data)
    {
        float ir_pose[3];
        if (mdock_controler_->getIRPose(ir_pose))
        {
            this->set_dock_position(ir_pose);
            this->getMarkerPose(dock_base_link_position_);
        }
        this->saveDockPositon();
    }
}

void CaculateDockPosition::updateMarkerPose(const ar_track_alvar_msgs::AlvarMarkers& currentMarkers)
{
    for( auto& marker_msg : currentMarkers.markers)
    {
      if(marker_msg.id == MARKER_ID_DETECTION)
      {
        boost::mutex::scoped_lock lock(mMutex_);
        mMarker_pose_camera_.header.frame_id = marker_msg.header.frame_id;
        mMarker_pose_camera_.pose = marker_msg.pose.pose;
        mMarker_pose_camera_.header.stamp = ros::Time();//获取最近时间的base_link坐标系下姿态

        if(mTf_flag_)
        {
          try
          {
            tf2_buffer_.transform(mMarker_pose_camera_, mMarker_pose_base_,std::string("base_link"));
          }
          catch (tf2::LookupException& ex)
          {
            ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
            mTf_flag_ = false;
            dock_mark_update_ = false;
            dock_mark_ready_ = false;
            return;
          }
          {
            dock_mark_update_ = true;
            dock_mark_ready_ =true;
          }
        }
        else
        {
          std::string tf_error;
          if(tf2_buffer_.canTransform(std::string("base_link"), std::string(mMarker_pose_camera_.header.frame_id), ros::Time(), ros::Duration(0.1), &tf_error))
          {
            mTf_flag_ = true;
          }
          else
          {
            mTf_flag_ = false;
            ROS_DEBUG("Timed out waiting for transform from base_link to %s to become available before running costmap, tf error: %s",
                   std::string(mMarker_pose_camera_.header.frame_id).c_str(),  tf_error.c_str());
          }
          dock_mark_update_ = false;
          dock_mark_ready_ = false;
        }
      }
    }
}

bool CaculateDockPosition::getMarkerPose(float (&marker_pose)[3])
{
    boost::mutex::scoped_lock lock(mMutex_);
    if (!dock_mark_ready_)
        return false;
    geometry_msgs::Pose current_pose = mMarker_pose_base_.pose;
    marker_pose[0] = current_pose.position.x;
    marker_pose[1] = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    marker_pose[2] = yaw;
    return true;
}

bool CaculateDockPosition::set_dock_position(float (&pose)[3])
{
    boost::mutex::scoped_lock lock(mMutex_);
    if (!dock_mark_ready_)
        return false;
    dock_realposition_[0] = pose[0];
    dock_realposition_[1] = pose[1];
    dock_realposition_[2] = pose[2];
    dock_realposition_ready_ = true;
    // ROS_INFO("dock_realposition_1 %f %f %f",dock_realposition_[0],dock_realposition_[1],dock_realposition_[2]);
    return true;
}

bool CaculateDockPosition::get_dock_position(float (&pose)[3])
{
    boost::mutex::scoped_lock lock(mMutex_);
    if (!dock_realposition_ready_)
        return false;
    pose[0] = dock_realposition_[0];
    pose[1] = dock_realposition_[1];
    pose[2] = dock_realposition_[2];
    return true;
}

}  // namespace bw_auto_dock
