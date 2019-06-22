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

#ifndef __GETDOCKPOSITION_H__
#define __GETDOCKPOSITION_H__
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <iomanip>
#include <fstream>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include "bw_auto_dock/DockController.h"

namespace bw_auto_dock
{
class DockController;
class CaculateDockPosition
{
  public:
    CaculateDockPosition(double station_distance, std::string frame_id, std::string dock_station_filename,
                         DockController* dock_controler, StatusPublisher* bw_status);
    bool getDockPosition(float (&station_pose_baselink_goal)[3]);
    bool getDockPosition(float (&station_pose1)[2], float (&station_pose2)[2]);
    void run();
    void updateMapsaveFlag(const std_msgs::Bool& currentFlag);
    void saveDockPositon();
    void updateMarkerPose(const ar_track_alvar_msgs::AlvarMarkers& currentMarkers);
    bool getMarkerPose(float (&marker_pose)[3]);
    bool set_dock_position(float (&pose)[3]);
    bool get_dock_position(float (&pose)[3]);

  private:
    DockController* mdock_controler_;
    StatusPublisher* mbw_status_;
    std::string mdock_station_filename_;
    float station_distance_;
    boost::mutex mMutex_;
    float dock_realposition_[3];
    bool dock_realposition_ready_;
    bool dock_mark_ready_;
    bool dock_mark_update_;

    float dock_base_link_position_[3];

    geometry_msgs::PoseStamped mMarker_pose_base_;
    geometry_msgs::PoseStamped mMarker_pose_camera_;
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_;

    bool mTf_flag_;
};

}  // namespace bw_auto_dock
#endif  // GETDOCKPOSITION_H
