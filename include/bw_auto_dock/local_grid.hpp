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

#ifndef __LOCAL_GRID_H__
#define __LOCAL_GRID_H__

#include <ros/ros.h>
#include <ros/console.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include "bw_auto_dock/gridlinetraversal.h"
#include "bw_auto_dock/point.h"
#include "bw_auto_dock/DockController.h"

namespace bw_auto_dock
{
class LocalGrid
{
  public:
    LocalGrid(float resolution, int half_length, std::string frame_id)
    {
        resolution_ = resolution;
        half_length_ = half_length;
        frame_id_ = frame_id;

        enforce_dock_position_ = false;

        side_num_ = half_length_ * 2;
        total_cell_num_ = side_num_ * side_num_;

        left_dock_cells1 = new float[total_cell_num_];
        left_dock_cells2 = new float[total_cell_num_];

        right_dock_cells1 = new float[total_cell_num_];
        right_dock_cells2 = new float[total_cell_num_];

        back_dock_cells1 = new float[total_cell_num_];
        back_dock_cells2 = new float[total_cell_num_];

        left_sensor_cells = new int8_t[total_cell_num_];
        right_sensor_cells = new int8_t[total_cell_num_];
        back_sensor_cells = new int8_t[total_cell_num_];

        mean_dock_cells1 = new int8_t[total_cell_num_];
        mean_dock_cells2 = new float[total_cell_num_];

        robot_clear_cells = new int8_t[total_cell_num_];

        for (int i = 0; i < total_cell_num_; i++)
        {
            left_dock_cells1[i] = 0.0;
            left_dock_cells2[i] = 0.0;
            right_dock_cells1[i] = 0.0;
            right_dock_cells2[i] = 0.0;
            back_dock_cells1[i] = 0.0;
            back_dock_cells2[i] = 0.0;

            left_sensor_cells[i] = 0;
            right_sensor_cells[i] = 0;
            back_sensor_cells[i] = 0;

            mean_dock_cells1[i] = 0;
            mean_dock_cells2[i] = 0.0;

            robot_clear_cells[i] = -1;
        }

        origin_[0] = 0.0;
        origin_[1] = 0.0;
        origin_[2] = 0.0;
        ir_length_ = 1.5;
        //构建clear模板
        float robot_x = 0.6;
        float robot_y = 0.3;
        float robot_yi = 0.0, robot_xi = 0.0;
        robot_clear_points_.resize(400, std::vector<float>(2));  // x y
        index_clear_ = 0;
        while (robot_xi <= (robot_x + 0.0001))
        {
            robot_yi = -robot_y;
            while (robot_yi <= (robot_y + 0.0001))
            {
                robot_clear_points_[index_clear_][0] = robot_xi;
                robot_clear_points_[index_clear_][1] = robot_yi;
                robot_yi += resolution_ * 0.5;
                index_clear_++;
            }
            robot_xi += resolution_ * 0.5;
        }
        //构建update模板,由60度、85度、90度、95度、120度4条线分割成4个区域
        IntPoint p0 = IntPoint((int)(ir_length_ / resolution_), 0);
        GridLineTraversalLine l1;
        IntPoint* m_linePoints1 = new IntPoint[100];
        l1.points = m_linePoints1;
        IntPoint p1 = IntPoint((int)(2 * ir_length_ / resolution_), (int)(ir_length_ / resolution_));
        GridLineTraversal::gridLine(p0, p1, &l1);

        GridLineTraversalLine l2;
        IntPoint* m_linePoints2 = new IntPoint[100];
        l2.points = m_linePoints2;
        IntPoint p2 = IntPoint((int)((1.0875 * ir_length_) / resolution_), (int)(ir_length_ / resolution_));
        GridLineTraversal::gridLine(p0, p2, &l2);

        GridLineTraversalLine l3;
        IntPoint* m_linePoints3 = new IntPoint[100];
        l3.points = m_linePoints3;
        IntPoint p3 = IntPoint((int)(ir_length_ / resolution_), (int)(ir_length_ / resolution_));
        GridLineTraversal::gridLine(p0, p3, &l3);

        GridLineTraversalLine l4;
        IntPoint* m_linePoints4 = new IntPoint[100];
        l4.points = m_linePoints4;
        IntPoint p4 = IntPoint((int)((0.9125 * ir_length_) / resolution_), (int)(ir_length_ / resolution_));
        GridLineTraversal::gridLine(p0, p4, &l4);

        GridLineTraversalLine l5;
        IntPoint* m_linePoints5 = new IntPoint[100];
        l5.points = m_linePoints5;
        IntPoint p5 = IntPoint(0, (int)(ir_length_ / resolution_));
        GridLineTraversal::gridLine(p0, p5, &l5);

        left_score_points1_.resize(600, std::vector<float>(3));  // x y theta
        left_score_points2_.resize(150, std::vector<float>(3));  // x y theta
        left_score_points3_.resize(300, std::vector<float>(3));  // x y theta
        left_score_points4_.resize(300, std::vector<float>(3));  // x y theta

        right_score_points1_.resize(600, std::vector<float>(3));  // x y theta
        right_score_points2_.resize(150, std::vector<float>(3));  // x y theta
        right_score_points3_.resize(300, std::vector<float>(3));  // x y theta
        right_score_points4_.resize(300, std::vector<float>(3));  // x y theta

        back_score_points1_.resize(600, std::vector<float>(3));  // x y theta
        back_score_points2_.resize(150, std::vector<float>(3));  // x y theta
        back_score_points3_.resize(300, std::vector<float>(3));  // x y theta
        back_score_points4_.resize(300, std::vector<float>(3));  // x y theta

        index1_ = 0;
        index2_ = 0;
        index3_ = 0;
        index4_ = 0;

        for (int i = 0; i < l5.num_points; i++)
        {
            float x1, y1, theta;
            // l5到l1构成1区 左侧  右侧  后侧
            y1 = l5.points[i].y * resolution_;
            for (int j = l5.points[i].x; j <= l1.points[i].x; j++)
            {
                if (l5.points[i].y == p0.y)
                    continue;
                x1 = (j - p0.x) * resolution_;
                if (j == p0.x)
                {
                    theta = PI_temp / 2.0;
                }
                else
                {
                    theta = atan2(y1, x1);
                }

                left_score_points1_[index1_][0] = x1;
                left_score_points1_[index1_][1] = y1;
                left_score_points1_[index1_][2] = theta - PI_temp;

                right_score_points1_[index1_][0] = -x1;
                right_score_points1_[index1_][1] = -y1;
                right_score_points1_[index1_][2] = theta;

                back_score_points1_[index1_][0] = -y1;
                back_score_points1_[index1_][1] = x1;
                theta += PI_temp / 2.0;
                if (theta > PI_temp)
                    theta = theta - 2.0 * PI_temp;
                if (theta > 0)
                {
                    back_score_points1_[index1_][2] = theta - PI_temp;
                }
                else
                {
                    back_score_points1_[index1_][2] = theta + PI_temp;
                }

                index1_++;
            }

            // l4到l2构成2区 左中  右中  后中
            y1 = l4.points[i].y * resolution_;
            for (int j = l4.points[i].x; j <= l2.points[i].x; j++)
            {
                if (l4.points[i].y == p0.y)
                    continue;
                x1 = (j - p0.x) * resolution_;
                if (j == p0.x)
                {
                    theta = PI_temp / 2.0;
                }
                else
                {
                    theta = atan2(y1, x1);
                }

                left_score_points2_[index2_][0] = x1;
                left_score_points2_[index2_][1] = y1;
                left_score_points2_[index2_][2] = theta - PI_temp;

                right_score_points2_[index2_][0] = -x1;
                right_score_points2_[index2_][1] = -y1;
                right_score_points2_[index2_][2] = theta;

                back_score_points2_[index2_][0] = -y1;
                back_score_points2_[index2_][1] = x1;
                theta += PI_temp / 2.0;
                if (theta > PI_temp)
                    theta = theta - 2.0 * PI_temp;
                if (theta > 0)
                {
                    back_score_points2_[index2_][2] = theta - PI_temp;
                }
                else
                {
                    back_score_points2_[index2_][2] = theta + PI_temp;
                }

                index2_++;
            }
            // l3到l1构成3区 左上  右下  后左
            y1 = l3.points[i].y * resolution_;
            for (int j = l3.points[i].x; j <= l1.points[i].x; j++)
            {
                if (l3.points[i].y == p0.y)
                    continue;
                x1 = (j - p0.x) * resolution_;
                if (j == p0.x)
                {
                    theta = PI_temp / 2.0;
                }
                else
                {
                    theta = atan2(y1, x1);
                }
                // ROS_INFO("index3_ %d %d %d ",index3_,j,l3.points[i].y);
                left_score_points3_[index3_][0] = x1;
                left_score_points3_[index3_][1] = y1;
                left_score_points3_[index3_][2] = theta - PI_temp;

                right_score_points3_[index3_][0] = -x1;
                right_score_points3_[index3_][1] = -y1;
                right_score_points3_[index3_][2] = theta;

                back_score_points3_[index3_][0] = -y1;
                back_score_points3_[index3_][1] = x1;
                theta += PI_temp / 2.0;
                if (theta > PI_temp)
                    theta = theta - 2.0 * PI_temp;
                if (theta > 0)
                {
                    back_score_points3_[index3_][2] = theta - PI_temp;
                }
                else
                {
                    back_score_points3_[index3_][2] = theta + PI_temp;
                }

                index3_++;
            }
            // l5到l3构成4区 左下  右上  后右
            y1 = l5.points[i].y * resolution_;
            for (int j = l5.points[i].x; j <= l3.points[i].x; j++)
            {
                if (l5.points[i].y == p0.y)
                    continue;
                x1 = (j - p0.x) * resolution_;
                if (j == p0.x)
                {
                    theta = PI_temp / 2.0;
                }
                else
                {
                    theta = atan2(y1, x1);
                }

                left_score_points4_[index4_][0] = x1;
                left_score_points4_[index4_][1] = y1;
                left_score_points4_[index4_][2] = theta - PI_temp;

                right_score_points4_[index4_][0] = -x1;
                right_score_points4_[index4_][1] = -y1;
                right_score_points4_[index4_][2] = theta;

                back_score_points4_[index4_][0] = -y1;
                back_score_points4_[index4_][1] = x1;
                theta += PI_temp / 2.0;
                if (theta > PI_temp)
                    theta = theta - 2.0 * PI_temp;
                if (theta > 0)
                {
                    back_score_points4_[index4_][2] = theta - PI_temp;
                }
                else
                {
                    back_score_points4_[index4_][2] = theta + PI_temp;
                }

                index4_++;
            }
        }

        // frame_id_=std::string("map");
        occupancygrid_msg_.info.resolution = resolution_;
        occupancygrid_msg_.info.height = side_num_;
        occupancygrid_msg_.info.width = side_num_;
        occupancygrid_msg_.info.origin.position.z = 0;
        occupancygrid_msg_.header.frame_id = frame_id_;
        grid_ready_ = false;

        pub_occupancy_grid_ =
            nodeHandler_.advertise<nav_msgs::OccupancyGrid>("bw_auto_dock/dock_occupancy_map", 1, true);
    }

    ~LocalGrid()
    {
        if (side_num_ > 0)
        {
            delete[] left_dock_cells1;
            delete[] left_dock_cells2;
            delete[] right_dock_cells1;
            delete[] right_dock_cells2;
            delete[] back_dock_cells1;
            delete[] back_dock_cells2;

            delete[] left_sensor_cells;
            delete[] right_sensor_cells;
            delete[] back_sensor_cells;

            delete[] mean_dock_cells1;
            delete[] mean_dock_cells2;
        }
    }

    void runPub(void)
    {
        if (!grid_ready_)
            return;
        ros::Time current_time = ros::Time::now();
        occupancygrid_msg_.info.origin.position.x = origin_[0];
        occupancygrid_msg_.info.origin.position.y = origin_[1];

        tf::Quaternion q1;
        q1.setRPY(0, 0, origin_[2]);

        occupancygrid_msg_.info.origin.orientation.x = q1.x();
        occupancygrid_msg_.info.origin.orientation.y = q1.y();
        occupancygrid_msg_.info.origin.orientation.z = q1.z();
        occupancygrid_msg_.info.origin.orientation.w = q1.w();

        occupancygrid_msg_.info.map_load_time = current_time;

        occupancygrid_msg_.header.seq += 1;
        occupancygrid_msg_.header.stamp = current_time;
        occupancygrid_msg_.header.frame_id = frame_id_;

        std::vector<int8_t> a(total_cell_num_, -1);
        occupancygrid_msg_.data = a;
        {
            boost::mutex::scoped_lock lock(mMutex_);
            this->update_mean();
            // memcpy(&occupancygrid_msg_.data[0],mean_dock_cells1,sizeof(int8_t)*(total_cell_num_));
            memcpy(&occupancygrid_msg_.data[0], mean_dock_cells1, sizeof(int8_t) * (total_cell_num_));
        }
        pub_occupancy_grid_.publish(occupancygrid_msg_);
    }

    void setOrigin(const float (&center)[3])
    {
        boost::mutex::scoped_lock lock(mMutex_);
        origin_[0] = center[0] - ((half_length_ - 0.5) * resolution_) * cos(center[2]) +
                     ((half_length_ - 0.5) * resolution_) * sin(center[2]);
        origin_[1] = center[1] - ((half_length_ - 0.5) * resolution_) * sin(center[2]) -
                     ((half_length_ - 0.5) * resolution_) * cos(center[2]);
        origin_[2] = center[2];
        grid_ready_ = true;
        // ROS_INFO("setorigin %f %f %f %f %f ",center[0],center[1],center[2],origin_[0],origin_[1]);
    }

    void update_clear(const float (&pose)[3])
    {
        boost::mutex::scoped_lock lock(mMutex_);
        if (!grid_ready_)
            return;
        static int pose_index_last = -1;
        int x_num, y_num;

        float x, y, float_temp, theta;
        float dx, dy;

        dx = pose[0] - origin_[0];
        dy = pose[1] - origin_[1];
        x = cos(origin_[2]) * dx + sin(origin_[2]) * dy;
        y = -sin(origin_[2]) * dx + cos(origin_[2]) * dy;

        float_temp = (x) / resolution_;
        x_num = static_cast<int>(float_temp);
        float_temp = (y) / resolution_;
        y_num = static_cast<int>(float_temp);

        if (x_num > (side_num_ - 1) || x_num < 0)
            return;
        if (y_num > (side_num_ - 1) || y_num < 0)
            return;

        int pose_index = x_num + y_num * side_num_;

        if (pose_index == pose_index_last)
            return;
        pose_index_last = pose_index;

        float cos_theta, sin_theta;
        cos_theta = cos(pose[2]);
        sin_theta = sin(pose[2]);
        float cos_theta0, sin_theta0;
        cos_theta0 = cos(origin_[2]);
        sin_theta0 = sin(origin_[2]);
        // ROS_INFO("update_clear1  %f %f %f %f %d %d %d",pose[0],pose[1],x,y,x_num,y_num,pose_index);
        // ROS_INFO("update_clear1.2  %f %f %f %f %f %f",pose[2],origin_[2],cos_theta,cos_theta0,sin_theta,sin_theta0);
        for (int i = 0; i < index_clear_; i++)
        {
            x = robot_clear_points_[i][0] * cos_theta - robot_clear_points_[i][1] * sin_theta +
                pose[0];  //转换到frame_id坐标系
            y = robot_clear_points_[i][0] * sin_theta + robot_clear_points_[i][1] * cos_theta + pose[1];

            //转换到occupancy坐标系
            dx = x - origin_[0];
            dy = y - origin_[1];
            x = cos_theta0 * dx + sin_theta0 * dy;
            y = -sin_theta0 * dx + cos_theta0 * dy;

            float_temp = (x) / resolution_;
            x_num = static_cast<int>(float_temp);
            float_temp = (y) / resolution_;
            y_num = static_cast<int>(float_temp);

            // ROS_INFO("update_clear2  %f %f %f %f %d %d
            // %d",robot_clear_points_[i][0],robot_clear_points_[i][1],x,y,x_num,y_num,i);

            if (x_num > (side_num_ - 1) || x_num < 0)
                continue;
            if (y_num > (side_num_ - 1) || y_num < 0)
                continue;

            pose_index = x_num + y_num * side_num_;
            robot_clear_cells[pose_index] = 1;  //清除,置位
            left_dock_cells1[pose_index] = 0.0;
            left_dock_cells2[pose_index] = 0.0;
            right_dock_cells1[pose_index] = 0.0;
            right_dock_cells2[pose_index] = 0.0;
            back_dock_cells1[pose_index] = 0.0;
            back_dock_cells2[pose_index] = 0.0;
        }
    }
    void update_sensor(const DOCK_POSITION dock_position_current, const float (&pose)[3])
    {
        boost::mutex::scoped_lock lock(mMutex_);
        if (!grid_ready_)
            return;
        int x_num, y_num;

        float x, y, float_temp, theta;
        float dx, dy;
        dx = pose[0] - origin_[0];
        dy = pose[1] - origin_[1];
        x = cos(origin_[2]) * dx + sin(origin_[2]) * dy;
        y = -sin(origin_[2]) * dx + cos(origin_[2]) * dy;

        float_temp = (x) / resolution_;
        x_num = static_cast<int>(float_temp);
        float_temp = (y) / resolution_;
        y_num = static_cast<int>(float_temp);

        if (x_num > (side_num_ - 1) || x_num < 0)
            return;
        if (y_num > (side_num_ - 1) || y_num < 0)
            return;

        int pose_index = x_num + y_num * side_num_;
        std::vector<std::vector<float>>* score_points;  // x y theta
        float* dock_cells1;
        float* dock_cells2;
        int index, weight;

        switch (dock_position_current)
        {
            // left
            case DOCK_POSITION::left:
                if ((left_sensor_cells[pose_index] & 4) == 4)
                    return;
                left_sensor_cells[pose_index] += 4;
                weight = 1;
                index = index1_;
                score_points = &left_score_points1_;
                dock_cells1 = left_dock_cells1;
                dock_cells2 = left_dock_cells2;
                break;
            case DOCK_POSITION::left_center:
                if ((left_sensor_cells[pose_index] & 8) == 8)
                    return;
                left_sensor_cells[pose_index] += 8;
                weight = 6;
                index = index2_;
                score_points = &left_score_points2_;
                dock_cells1 = left_dock_cells1;
                dock_cells2 = left_dock_cells2;
                break;
            case DOCK_POSITION::left_up:
                if ((left_sensor_cells[pose_index] & 1) == 1)
                    return;
                left_sensor_cells[pose_index] += 1;
                weight = 3;
                index = index3_;
                score_points = &left_score_points3_;
                dock_cells1 = left_dock_cells1;
                dock_cells2 = left_dock_cells2;
                break;
            case DOCK_POSITION::left_down:
                if ((left_sensor_cells[pose_index] & 2) == 2)
                    return;
                left_sensor_cells[pose_index] += 2;
                weight = 3;
                index = index4_;
                score_points = &left_score_points4_;
                dock_cells1 = left_dock_cells1;
                dock_cells2 = left_dock_cells2;
                break;

            // right
            case DOCK_POSITION::right:
                if ((right_sensor_cells[pose_index] & 4) == 4)
                    return;
                right_sensor_cells[pose_index] += 4;
                weight = 1;
                index = index1_;
                score_points = &right_score_points1_;
                dock_cells1 = right_dock_cells1;
                dock_cells2 = right_dock_cells2;
                break;
            case DOCK_POSITION::right_center:
                if ((right_sensor_cells[pose_index] & 8) == 8)
                    return;
                right_sensor_cells[pose_index] += 8;
                weight = 6;
                index = index2_;
                score_points = &right_score_points2_;
                dock_cells1 = right_dock_cells1;
                dock_cells2 = right_dock_cells2;
                break;
            case DOCK_POSITION::right_down:
                if ((right_sensor_cells[pose_index] & 1) == 1)
                    return;
                right_sensor_cells[pose_index] += 1;
                weight = 3;
                index = index3_;
                score_points = &right_score_points3_;
                dock_cells1 = right_dock_cells1;
                dock_cells2 = right_dock_cells2;
                break;
            case DOCK_POSITION::right_up:
                if ((right_sensor_cells[pose_index] & 2) == 2)
                    return;
                right_sensor_cells[pose_index] += 2;
                weight = 3;
                index = index4_;
                score_points = &right_score_points4_;
                dock_cells1 = right_dock_cells1;
                dock_cells2 = right_dock_cells2;
                break;

            // back
            case DOCK_POSITION::back:
                if ((back_sensor_cells[pose_index] & 4) == 4)
                    return;
                back_sensor_cells[pose_index] += 4;
                weight = 1;
                index = index1_;
                score_points = &back_score_points1_;
                dock_cells1 = back_dock_cells1;
                dock_cells2 = back_dock_cells2;
                break;
            case DOCK_POSITION::back_center:
                if ((back_sensor_cells[pose_index] & 8) == 8)
                    return;
                back_sensor_cells[pose_index] += 8;
                weight = 6;
                index = index2_;
                score_points = &back_score_points2_;
                dock_cells1 = back_dock_cells1;
                dock_cells2 = back_dock_cells2;
                break;
            case DOCK_POSITION::back_left:
                if ((back_sensor_cells[pose_index] & 1) == 1)
                    return;
                back_sensor_cells[pose_index] += 1;
                weight = 3;
                index = index3_;
                score_points = &back_score_points3_;
                dock_cells1 = back_dock_cells1;
                dock_cells2 = back_dock_cells2;
                break;
            case DOCK_POSITION::back_right:
                if ((back_sensor_cells[pose_index] & 2) == 2)
                    return;
                back_sensor_cells[pose_index] += 2;
                weight = 3;
                index = index4_;
                score_points = &back_score_points4_;
                dock_cells1 = back_dock_cells1;
                dock_cells2 = back_dock_cells2;
                break;

            default:
                return;
        }

        float cos_theta, sin_theta;
        cos_theta = cos(pose[2]);
        sin_theta = sin(pose[2]);
        float cos_theta0, sin_theta0;
        cos_theta0 = cos(origin_[2]);
        sin_theta0 = sin(origin_[2]);
        // ROS_INFO("update_sensor1 %d %f %f %f %f %f %d %d
        // %d",(int)dock_position_current,pose[0],pose[1],pose[2],origin_[0],origin_[1],x_num,y_num,index);

        for (int i = 0; i < index; i++)
        {
            x = (*score_points)[i][0] * cos_theta - (*score_points)[i][1] * sin_theta + pose[0];  //转换到frame_id坐标系
            y = (*score_points)[i][0] * sin_theta + (*score_points)[i][1] * cos_theta + pose[1];

            //转换到occupancy坐标系
            dx = x - origin_[0];
            dy = y - origin_[1];
            x = cos_theta0 * dx + sin_theta0 * dy;
            y = -sin_theta0 * dx + cos_theta0 * dy;

            theta = (*score_points)[i][2] + pose[2];

            if (theta >= PI_temp)
                theta -= 2 * PI_temp;
            if (theta < -PI_temp)
                theta += 2 * PI_temp;

            float_temp = (x) / resolution_;
            x_num = static_cast<int>(float_temp);
            float_temp = (y) / resolution_;
            y_num = static_cast<int>(float_temp);

            if (x_num > (side_num_ - 1) || x_num < 0)
                continue;
            if (y_num > (side_num_ - 1) || y_num < 0)
                continue;

            pose_index = x_num + y_num * side_num_;
            // ROS_INFO("update_sensor2.1  %d %d %d  %d %d",pose_index,x_num,y_num,i,weight);
            if (robot_clear_cells[pose_index] == 1)
                continue;  //清除已使能,robot可以到达所以肯定没有充电桩
            // ROS_INFO("update_sensor2.2  %d %d %d  %d %d",pose_index,x_num,y_num,i,weight);
            // int value_temp = dock_cells1[pose_index]+weight;
            // if(value_temp>=127) continue;
            dock_cells1[pose_index] += weight;
            dock_cells2[pose_index] += weight * theta;
        }
    }

    void clear_all(void)
    {
        boost::mutex::scoped_lock lock(mMutex_);
        max_indexs_.clear();
        for (int i = 0; i < total_cell_num_; i++)
        {
            left_dock_cells1[i] = 0;
            left_dock_cells2[i] = 0.0;
            right_dock_cells1[i] = 0;
            right_dock_cells2[i] = 0.0;
            back_dock_cells1[i] = 0;
            back_dock_cells2[i] = 0.0;

            left_sensor_cells[i] = 0;
            right_sensor_cells[i] = 0;
            back_sensor_cells[i] = 0;

            mean_dock_cells1[i] = 0;
            mean_dock_cells2[i] = 0.0;

            robot_clear_cells[i] = -1;
        }

        origin_[0] = 0.0;
        origin_[1] = 0.0;
        origin_[2] = 0.0;
        grid_ready_ = false;
        enforce_dock_position_ = false;
    }

    bool set_dock_position(float (&pose)[3])
    {
        boost::mutex::scoped_lock lock(mMutex_);
        if (!grid_ready_)
            return false;
        dock_realposition_[0] = pose[0];
        dock_realposition_[1] = pose[1];
        dock_realposition_[2] = pose[2];
        enforce_dock_position_ = true;
        // ROS_INFO("dock_realposition_1 %f %f %f",dock_realposition_[0],dock_realposition_[1],dock_realposition_[2]);
        return true;
    }

    bool get_dock_position_flag(float (&pose)[3])
    {
        boost::mutex::scoped_lock lock(mMutex_);
        return enforce_dock_position_;
    }

    bool get_dock_position(float (&pose)[3])
    {
        boost::mutex::scoped_lock lock(mMutex_);
        if (!grid_ready_)
            return false;
        if (!enforce_dock_position_)
            this->update_mean();
        pose[0] = dock_realposition_[0];
        pose[1] = dock_realposition_[1];
        pose[2] = dock_realposition_[2];
        return true;
    }
    bool get_ready_flag()
    {
        boost::mutex::scoped_lock lock(mMutex_);
        return grid_ready_;
    }

    void update_mean(void)
    {
        max_indexs_.clear();
        float max_value = 1.0;
        for (int i = 0; i < total_cell_num_; i++)
        {
            //记录最大值,计算权重平均值
            float value;
            value = left_dock_cells1[i] + right_dock_cells1[i] + back_dock_cells1[i];
            // ROS_INFO("update_mean1 %f %f %f %d",left_dock_cells1[i],right_dock_cells1[i],back_dock_cells1[i],i);
            if (value <= (max_value + 6.0001) && value >= (max_value - 5.9999))
            {
                max_indexs_.push_back(i);
            }
            else if (value > (max_value + 5.0001))
            {
                max_indexs_.clear();
                max_value = value;
                max_indexs_.push_back(i);
            }
            float theta;
            if (value <= 0.0001)
            {
                theta = 0.0;
            }
            else
            {
                theta = (left_dock_cells2[i] + right_dock_cells2[i] + back_dock_cells2[i]) / value;
            }

            if (theta > PI_temp)
                theta -= 2.0 * PI_temp;
            if (theta < -PI_temp)
                theta += 2.0 * PI_temp;

            mean_dock_cells2[i] = theta;
        }
        // ROS_INFO("update_mean1 %f %d ",max_value,(int)max_indexs_.size());
        for (int i = 0; i < total_cell_num_; i++)
        {
            //记录最大值,计算权重平均值
            float value;
            value = left_dock_cells1[i] + right_dock_cells1[i] + back_dock_cells1[i];

            if (value <= 0.0001)
            {
                mean_dock_cells1[i] = -1;
            }
            else
            {
                float value_temp = 100 * value / max_value;
                if (value_temp > 99)
                    value_temp = 101;

                mean_dock_cells1[i] = (int8_t)value_temp;
                // ROS_INFO("update_mean1.2 %f %f %d %f ",value,value_temp,mean_dock_cells1[i],mean_dock_cells2[i]);
            }
        }
        if (!enforce_dock_position_)
        {
            //计算充电桩的最大概率位置
            dock_realposition_[0] = 0.;
            dock_realposition_[1] = 0.;
            dock_realposition_[2] = 0.;

            float weight = 1.0 / max_indexs_.size();

            float cos_theta = cos(origin_[2]);
            float sin_theta = sin(origin_[2]);

            for (int i = 0; i < max_indexs_.size(); i++)
            {
                int index, index_x, index_y;
                index = max_indexs_[i];
                // occupancy坐标系下
                float x1, y1;
                index_y = index / side_num_;
                index_x = index % side_num_;
                x1 = index_x * resolution_;
                y1 = index_y * resolution_;
                //转换到frame_id坐标系
                float x, y;
                x = x1 * cos_theta - y1 * sin_theta + origin_[0];
                y = x1 * sin_theta + y1 * cos_theta + origin_[1];

                dock_realposition_[0] += weight * x;
                dock_realposition_[1] += weight * y;
                dock_realposition_[2] += weight * mean_dock_cells2[index];

                // ROS_INFO("update_mean2 %d %d  %f %f %f ",index_x,index_y,x,y,weight);
            }
        }
        ROS_DEBUG("dock_realposition_ %f %f %f", dock_realposition_[0], dock_realposition_[1], dock_realposition_[2]);
    }

  private:
    ros::NodeHandle nodeHandler_;
    float ir_length_;  //红外有效距离,1.5米
    float origin_[3];
    float resolution_;
    int half_length_;
    float* left_dock_cells1;   //权重
    float* left_dock_cells2;   //角度
    float* right_dock_cells1;  //权重
    float* right_dock_cells2;  //角度
    float* back_dock_cells1;   //权重
    float* back_dock_cells2;   //角度

    int8_t* left_sensor_cells;   //权重
    int8_t* right_sensor_cells;  //权重
    int8_t* back_sensor_cells;   //权重

    int8_t* mean_dock_cells1;  //权重,平均值用于发布
    float* mean_dock_cells2;   //角度,平均值用于发布

    int8_t* robot_clear_cells;

    bool grid_ready_;

    std::vector<int> max_indexs_;
    //每次的观测值产生的更新点
    //每组传感器分成四种情况
    std::vector<std::vector<float>> left_score_points1_;  // x y theta
    std::vector<std::vector<float>> left_score_points2_;  // x y theta
    std::vector<std::vector<float>> left_score_points3_;  // x y theta
    std::vector<std::vector<float>> left_score_points4_;  // x y theta

    std::vector<std::vector<float>> right_score_points1_;  // x y theta
    std::vector<std::vector<float>> right_score_points2_;  // x y theta
    std::vector<std::vector<float>> right_score_points3_;  // x y theta
    std::vector<std::vector<float>> right_score_points4_;  // x y theta

    std::vector<std::vector<float>> back_score_points1_;  // x y theta
    std::vector<std::vector<float>> back_score_points2_;  // x y theta
    std::vector<std::vector<float>> back_score_points3_;  // x y theta
    std::vector<std::vector<float>> back_score_points4_;  // x y theta

    std::vector<std::vector<float>> robot_clear_points_;  // x y

    int index1_, index2_, index3_, index4_, index_clear_;

    int side_num_;
    int total_cell_num_;
    boost::mutex mMutex_;
    ros::Publisher pub_occupancy_grid_;
    nav_msgs::OccupancyGrid occupancygrid_msg_;

    float dock_realposition_[3];

    bool enforce_dock_position_;

  public:
    std::string frame_id_;
};

}  // namespace bw_auto_dock
#endif /* LOCAL_GRID_H_ */
