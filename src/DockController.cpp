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
#include "bw_auto_dock/DockController.h"
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #define kp_theta_set 0.4
// #define kd_theta_set 0.8
// #define ki_theta_set 1
//
// #define kp_y_set 0.4
// #define kd_y_set 0.8
// #define ki_y_set 10
//
// #define kp_x_set 0.4
// #define kd_x_set 0.8
// #define ki_x_set 10

namespace bw_auto_dock
{
DockController::DockController(double back_distance, double max_linearspeed, double max_rotspeed,double crash_distance, int barDetectFlag,std::string global_frame,
                             StatusPublisher* bw_status):tf2_(tf2_buffer_),  global_frame_(global_frame)
{
    if(barDetectFlag==1)
    {
      barDetectFlag_ = true;
    }
    else
    {
      barDetectFlag_ = false;
    }
    back_distance_ = back_distance;
    max_linearspeed_ = max_linearspeed;
    max_rotspeed_ = max_rotspeed;
    crash_distance_ = crash_distance;
    bw_status_ = bw_status;
    mcharge_status_ = CHARGE_STATUS::freed;
    bw_status_->set_charge_status(mcharge_status_);
    mPose3_ = new float[3];
    mPose4_ = new float[3];

    mstationPose3_[0] = 0;
    mstationPose3_[1] = 0;

    mcurrentChargeFlag_ = false;
    left2_error1_ = 0.;
    left2_error2_ = 0.;
    left2_error3_ = 0.;
    right2_error1_ = 0.;
    right2_error2_ = 0.;
    right2_error3_ = 0.;
    rot_z_ = 0.;
    usefull_num_ = 0;
    unusefull_num_ = 0;
    kp_ = 0.;
    ki_ = 0.;
    kd_ = 0.;
    current_average_ = 1.0;
    mPose_flag_ = false;

    min_x2_ = 100.0;

    min_x2_4_ = 100.0;//用来设置发散检查

    mTf_flag_ = false;
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose_.pose);
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose_.pose);

    power_threshold_ = 41.0;
    y_min_set_ = 0.05; //对准过程中y轴偏差阈值
    x_min_set_ = 0.3; //对准过程中x轴偏差阈值,小于这个值需要回退到参考点
    theta_min_set_ = 0.3; //对准过程中角度偏差阈值

    goal_position_[0]= 0;
    goal_position_[1]= 0;
    goal_position_[2]= 0;

    error_theta_last_ = 0;
    error_theta_sum_ = 0;
    error_y_last_ = 0;
    error_y_sum_ = 0;
    error_x_last_ = 0;
    error_x_sum_= 0;
}

void DockController::setPowerParam(double power_threshold)
{
  power_threshold_ = power_threshold;
}

void DockController::run()
{
    ros::NodeHandle nodeHandler;
    mCmdvelPub_ = nodeHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    mbarDetectPub_ = nodeHandler.advertise<std_msgs::Bool>("/barDetectFlag", 1, true);
    mlimitSpeedPub_ = nodeHandler.advertise<std_msgs::Bool>("/limitSpeedFlag", 1, true);
    ros::Subscriber sub1 =
        nodeHandler.subscribe("/bw_auto_dock/EnableCharge", 1, &DockController::updateChargeFlag, this);
    ros::Subscriber sub2 = nodeHandler.subscribe("/odom", 1, &DockController::updateOdom, this);
    ros::spin();
}

void DockController::updateChargeFlag(const std_msgs::Bool& currentFlag)
{
    boost::mutex::scoped_lock lock(mMutex_charge);
    mcurrentChargeFlag_ = currentFlag.data;
    usefull_num_ = 0;
    unusefull_num_ = 0;
}

void DockController::updateOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_pose_.header.frame_id = msg->header.frame_id;
    robot_pose_.pose = msg->pose.pose;
    robot_pose_.header.stamp = ros::Time();//获取最近时间的map坐标系下姿态

    {
      boost::mutex::scoped_lock lock(mMutex_pose);
      global_station3_pose_.header.frame_id = global_frame_;
      global_station3_pose_.header.stamp = ros::Time();//获取最近时间的map坐标系下姿态
      global_station3_pose_.point.x = mstationPose3_[0];
      global_station3_pose_.point.y = mstationPose3_[1];
      global_station3_pose_.point.z = 0;
    }
    if(mTf_flag_)
    {
      try
      {
        tf2_buffer_.transform(robot_pose_, global_pose_, global_frame_);
        tf2_buffer_.transform(global_station3_pose_, local_station3_pose_, std::string("base_link"));
      //  ROS_ERROR("station %f %f , %f %f",global_station3_pose_.point.x,global_station3_pose_.point.y,local_station3_pose_.point.x,local_station3_pose_.point.y);
      }
      catch (tf2::LookupException& ex)
      {
        boost::mutex::scoped_lock lock(mMutex_pose);
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        mTf_flag_ = false;
        mPose_flag_ = false;
        return;
      }
      {
        boost::mutex::scoped_lock lock(mMutex_pose);
        mRobot_pose_ = global_pose_.pose;
        mPose_flag_ = true;
      }
    }
    else
    {
      std::string tf_error;
      if(tf2_buffer_.canTransform(global_frame_, std::string("base_link"), ros::Time(), ros::Duration(0.1), &tf_error))
      {
        mTf_flag_ = true;
      }
      else
      {
        mTf_flag_ = false;
        ROS_DEBUG("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
               "odom", global_frame_.c_str(), tf_error.c_str());
      }
      {
        boost::mutex::scoped_lock lock(mMutex_pose);
        mPose_flag_ = false;
      }
    }
}

void DockController::dealing_status()
{
    boost::mutex::scoped_lock lock1(mMutex_charge);
    geometry_msgs::Twist current_vel;
    if (!mPose_flag_)
    {
      if (mcharge_status_ != CHARGE_STATUS::freed && mcurrentChargeFlag_)
      {
        //停止移动
        current_vel.linear.x = 0;
        current_vel.linear.y = 0;
        current_vel.linear.z = 0;
        current_vel.angular.x = 0;
        current_vel.angular.y = 0;
        current_vel.angular.z = 0;
        ROS_ERROR("map to base_link not ready!");
        mCmdvelPub_.publish(current_vel);
      }
      return;  //里程计没有开启
    }
    if (mcurrentChargeFlag_)
    {
        if (mcharge_status_ == CHARGE_STATUS::freed)
        {
            if (bw_status_->sensor_status.power > 9.0)
            {
                //已经侦测到电压，进入充电状态
                mcharge_status_temp_ = CHARGE_STATUS_TEMP::charging1;
                current_average_ = 1.0;
                mcharge_status_ = CHARGE_STATUS::charging;
                bw_status_->set_charge_status(mcharge_status_);
                usefull_num_ = 0;
                unusefull_num_ = 0;
                //停止移动
                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
            }
            else
            {
                mcharge_status_ = CHARGE_STATUS::finding;
                mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding0;
                bw_status_->set_charge_status(mcharge_status_);
                usefull_num_ = 0;
                unusefull_num_ = 0;
            }
        }

        float marker_pose_current[3]={0.0,0.0,0.0};
        bool marker_pose_ready = mdock_position_caculate_->getMarkerPose(marker_pose_current);

        //ROS_ERROR("current_marker %d %f %f %f",marker_pose_ready, marker_pose_current[0],marker_pose_current[1],marker_pose_current[2]);

        if(mcharge_status_ == CHARGE_STATUS::docking && !marker_pose_ready)
        {
          //如果处于docking过程中丢失marker，回到finding2状态
          mcharge_status_ = CHARGE_STATUS::finding;
          mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding2;
          bw_status_->set_charge_status(mcharge_status_);
          usefull_num_ = 0;
          unusefull_num_ = 0;
          //停止移动
          current_vel.linear.x = 0;
          current_vel.linear.y = 0;
          current_vel.linear.z = 0;
          current_vel.angular.x = 0;
          current_vel.angular.y = 0;
          current_vel.angular.z = 0;
          mCmdvelPub_.publish(current_vel);
          error_theta_last_ = 0;
          error_theta_sum_ = 0;
          error_y_last_ = 0;
          error_y_sum_ = 0;
          error_x_last_ = 0;
          error_x_sum_= 0;
          return;
        }
        switch (mcharge_status_temp_)
        {
            case CHARGE_STATUS_TEMP::finding0:
                {
                  ROS_DEBUG("finding0.0");
                  if (usefull_num_ == 0)
                  {
                      //根据充电桩位置,计算两个移动参考点
                      usefull_num_++;
                      if (mdock_position_caculate_->getDockPosition(mstationPose1_, mstationPose2_))
                      {
                          //选择中间点当成station3
                          ROS_DEBUG("finding0.1");
                          this->caculateStation3();
                          if(!mdock_position_caculate_->getDockPosition(goal_position_))
                          {
                            // error,没有设置充电桩位置
                            ROS_ERROR("Can not get dock station position!2");
                            mcharge_status_ = CHARGE_STATUS::freed;
                            bw_status_->set_charge_status(mcharge_status_);
                            usefull_num_ = 0;
                            unusefull_num_ = 0;
                            mcurrentChargeFlag_ = false;
                            //停止移动
                            current_vel.linear.x = 0;
                            current_vel.linear.y = 0;
                            current_vel.linear.z = 0;
                            current_vel.angular.x = 0;
                            current_vel.angular.y = 0;
                            current_vel.angular.z = 0;
                            mCmdvelPub_.publish(current_vel);
                          }
                          else
                          {
                            ROS_DEBUG("goal_position_ %f %f %f", goal_position_[0],goal_position_[1],goal_position_[3]);
                          }
                      }
                      else
                      {
                          // error,没有设置充电桩位置
                          ROS_ERROR("Can not get dock station position!1");
                          mcharge_status_ = CHARGE_STATUS::freed;
                          bw_status_->set_charge_status(mcharge_status_);
                          usefull_num_ = 0;
                          unusefull_num_ = 0;
                          mcurrentChargeFlag_ = false;
                          //停止移动
                          current_vel.linear.x = 0;
                          current_vel.linear.y = 0;
                          current_vel.linear.z = 0;
                          current_vel.angular.x = 0;
                          current_vel.angular.y = 0;
                          current_vel.angular.z = 0;
                          mCmdvelPub_.publish(current_vel);
                      }
                  }
                  else
                  {
                      ROS_DEBUG("finding0.2");
                      min_x2_ = 100.0;
                      //进入直线运动finding1
                      mcharge_status_ = CHARGE_STATUS::finding;
                      mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding1;
                      bw_status_->set_charge_status(mcharge_status_);
                      usefull_num_ = 0;
                      unusefull_num_ = 0;
                      //停止移动
                      current_vel.linear.x = 0;
                      current_vel.linear.y = 0;
                      current_vel.linear.z = 0;
                      current_vel.angular.x = 0;
                      current_vel.angular.y = 0;
                      current_vel.angular.z = 0;
                      mCmdvelPub_.publish(current_vel);
                  }
                }
                break;
            case CHARGE_STATUS_TEMP::finding1:
                {
                  ROS_DEBUG("finding1.0");
                  //原地旋转，直到发现
                  static float last_theta = 0,theta_delta_sum=0;

                  float theta;
                  geometry_msgs::Pose current_pose = mRobot_pose_;
                  tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                                    current_pose.orientation.w);
                  tf::Matrix3x3 m1(q1);
                  double roll, pitch, yaw;
                  m1.getRPY(roll, pitch, yaw);
                  theta = yaw;
                  if (usefull_num_ == 0)
                  {
                      usefull_num_++;
                      //记录当前角度
                      last_theta = theta;
                  }
                  else
                  {
                      float theta_error = theta - last_theta;
                      if (theta_error <= -PI_temp)
                          theta_error += 2 * PI_temp;
                      if (theta_error > PI_temp)
                          theta_error -= 2 * PI_temp;
                      theta_delta_sum = theta_delta_sum + fabs(theta_error);

                      if (fabs(theta_delta_sum) >= (2.5 * PI_temp))
                      {
                          //旋转超过360度后还是没有发现，进入finding2
                          mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding2;
                          usefull_num_ = 0;
                          unusefull_num_ = 0;
                          //停止移动
                          current_vel.linear.x = 0;
                          current_vel.linear.y = 0;
                          current_vel.linear.z = 0;
                          current_vel.angular.x = 0;
                          current_vel.angular.y = 0;
                          current_vel.angular.z = 0;
                          mCmdvelPub_.publish(current_vel);
                          error_theta_last_ = 0;
                          error_theta_sum_ = 0;
                          error_y_last_ = 0;
                          error_y_sum_ = 0;
                          error_x_last_ = 0;
                          error_x_sum_= 0;
                      }
                  }
                  last_theta = theta;
                  if (marker_pose_ready)
                  {
                      mcharge_status_temp_ = CHARGE_STATUS_TEMP::docking1;
                      mcharge_status_ = CHARGE_STATUS::docking;
                      bw_status_->set_charge_status(mcharge_status_);
                      //停止移动
                      current_vel.linear.x = 0;
                      current_vel.linear.y = 0;
                      current_vel.linear.z = 0;
                      current_vel.angular.x = 0;
                      current_vel.angular.y = 0;
                      current_vel.angular.z = 0;
                      mCmdvelPub_.publish(current_vel);
                      //重置误差
                      left2_error1_ = 0.;
                      left2_error2_ = 0.;
                      left2_error3_ = 0.;
                      right2_error1_ = 0.;
                      right2_error2_ = 0.;
                      right2_error3_ = 0.;
                      rot_z_ = 0.;
                      usefull_num_ = 0;
                      unusefull_num_ = 0;
                  }
                  else
                  {
                      // if (mdock__referenss_position_ == DOCK_POSITION::left_center)
                      // {
                      //     //右转
                      //     current_vel.angular.z = -0.3;
                      // }
                      // else
                      // {
                      //     //左转
                      //     current_vel.angular.z = 0.3;
                      // }
                      current_vel.angular.z = 0.3;

                      current_vel.linear.x = 0;
                      current_vel.linear.y = 0;
                      current_vel.linear.z = 0;
                      current_vel.angular.x = 0;
                      current_vel.angular.y = 0;
                      mCmdvelPub_.publish(current_vel);
                  }
                }
                break;
            case CHARGE_STATUS_TEMP::finding2:
                {
                  ROS_DEBUG("finding2");
                  //回退到station3参考点
                  if (this->goToStation3())
                  {
                      mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding0;
                      usefull_num_ = 0;
                      unusefull_num_ = 0;
                      current_vel.linear.x = 0;
                      current_vel.linear.y = 0;
                      current_vel.linear.z = 0;
                      current_vel.angular.x = 0;
                      current_vel.angular.y = 0;
                      current_vel.angular.z = 0;
                      mCmdvelPub_.publish(current_vel);
                  }
                }
                break;
            case CHARGE_STATUS_TEMP::docking1:
                {
                  ROS_DEBUG("docking1.1");
                  //旋转使车垂直充电桩
                  if(usefull_num_ == 0)
                  {
                    usefull_num_ ++;
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                  }

                  if(fabs(marker_pose_current[0]-goal_position_[0])<= x_min_set_)
                  {
                    //ROS_ERROR("x_min_set %f %f ", fabs(marker_pose_current[0]) ,x_min_set_);

                    //如果小于x轴设定值，回到finding2
                    mcharge_status_ = CHARGE_STATUS::finding;
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding2;
                    bw_status_->set_charge_status(mcharge_status_);
                    usefull_num_ = 0;
                    unusefull_num_ = 0;
                    //停止移动
                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                    error_y_last_ = 0;
                    error_y_sum_ = 0;
                    error_x_last_ = 0;
                    error_x_sum_= 0;
                    return;
                  }

                  if(fabs(marker_pose_current[2]-goal_position_[2])<theta_min_set_)
                  {
                      //ROS_ERROR(" theta_min_set%f %f ", fabs(marker_pose_current[2]-goal_position_[2]),theta_min_set_);
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::docking2;
                    //停止移动
                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                    usefull_num_ = 0;
                    unusefull_num_ = 0;
                  }
                  else
                  {
                    float kp_theta = kp_theta_set_;
                    float kd_theta = kp_theta_set_*30*kd_theta_set_;
                    float ki_theta = kp_theta_set_/30/ki_theta_set_;

                    float error_temp1 = marker_pose_current[2] - error_theta_last_;
                    error_theta_sum_ += marker_pose_current[2] - goal_position_[2];
                    if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                    if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                    current_vel.angular.z = kp_theta*(marker_pose_current[2]-goal_position_[2]) + kd_theta*error_temp1 + ki_theta*error_theta_sum_;

                    //ROS_ERROR("theta %f %f %f ,%f ,%f, %f %f",marker_pose_current[0],marker_pose_current[1],marker_pose_current[2],kp_theta,kd_theta,error_temp1,current_vel.angular.z);

                    if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
                    if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;

                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    mCmdvelPub_.publish(current_vel);
                  }
                  error_theta_last_ = marker_pose_current[2];
                }
                break;
            case CHARGE_STATUS_TEMP::docking2:
                ROS_DEBUG("docking2.1");
                {
                  //右侧移动使车垂直充电桩
                  if(usefull_num_ == 0)
                  {
                    usefull_num_ ++;
                    error_y_last_ = 0;
                    error_y_sum_ = 0;
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                  }
                  if(fabs(marker_pose_current[0] -goal_position_[0])<= x_min_set_ )
                  {
                    //如果小于x轴设定值，回到finding2
                    mcharge_status_ = CHARGE_STATUS::finding;
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding2;
                    bw_status_->set_charge_status(mcharge_status_);
                    usefull_num_ = 0;
                    unusefull_num_ = 0;
                    //停止移动
                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                    error_y_last_ = 0;
                    error_y_sum_ = 0;
                    error_x_last_ = 0;
                    error_x_sum_= 0;
                    return;
                  }
                  if(fabs(marker_pose_current[1]-goal_position_[1])<y_min_set_ && fabs(marker_pose_current[2]-goal_position_[2])<theta_min_set_)
                  {
                    //ROS_ERROR("%f %f ,%f %f", fabs(marker_pose_current[2]-goal_position_[2]) ,theta_min_set_, fabs(marker_pose_current[1]-goal_position_[1]) ,y_min_set_);

                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::docking3;
                    //停止移动
                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                    usefull_num_ = 0;
                    unusefull_num_ = 0;
                  }
                  else
                  {

                    float kp_theta = kp_theta_set_;
                    float kd_theta = kp_theta_set_*30*kd_theta_set_;
                    float ki_theta = kp_theta_set_/30/ki_theta_set_;

                    float error_temp1 = marker_pose_current[2] - error_theta_last_;
                    error_theta_sum_ += marker_pose_current[2] - goal_position_[2];
                    if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                    if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                    current_vel.angular.z = kp_theta*(marker_pose_current[2]-goal_position_[2]) + kd_theta*error_temp1 + ki_theta*error_theta_sum_;

                    //ROS_ERROR("theta %f %f %f ,%f ,%f, %f %f",marker_pose_current[0],marker_pose_current[1],marker_pose_current[2],kp_theta,kd_theta,error_temp1,current_vel.angular.z);

                    if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z=max_theta_speed_;
                    if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z=-max_theta_speed_;


                    float kp_y = kp_y_set_;
                    float kd_y = kp_y_set_*30*kd_y_set_;
                    float ki_y = kp_y_set_/30/ki_y_set_;

                    error_temp1 = marker_pose_current[1] - error_y_last_;
                    error_y_sum_ += marker_pose_current[1] - goal_position_[1];
                    if(error_y_sum_>3.0) error_y_sum_ = 3.0;
                    if(error_y_sum_<-3.0) error_y_sum_ = -3.0;

                    current_vel.linear.y = kp_y*(marker_pose_current[1]-goal_position_[1]) + kd_y*error_temp1 + ki_y*error_y_sum_;
                    if(current_vel.linear.y > max_y_speed_ ) current_vel.linear.y = max_y_speed_;
                    if(current_vel.linear.y < -max_y_speed_ ) current_vel.linear.y = -max_y_speed_;

                    //ROS_ERROR("y_set %f %f %f ,%f ,%f, %f %f",marker_pose_current[0],marker_pose_current[1],marker_pose_current[2],kp_y,kd_y,error_temp1,current_vel.linear.y);

                    current_vel.linear.x = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    mCmdvelPub_.publish(current_vel);
                  }
                  error_y_last_ = marker_pose_current[1];
                  error_theta_last_ = marker_pose_current[2];
                }
                break;
            case CHARGE_STATUS_TEMP::docking3:
                {
                  ROS_DEBUG("docking3.0");
                  //开始对准
                  if(usefull_num_ == 0)
                  {
                    usefull_num_ ++;
                    error_theta_last_ = 0;
                    error_theta_sum_ = 0;
                    error_y_last_ = 0;
                    error_y_sum_ = 0;
                    error_x_last_ = 0;
                    error_y_sum_ = 0;
                  }
                  if(fabs(marker_pose_current[0]-goal_position_[0])<= goal_x_error_ && fabs(marker_pose_current[1]-goal_position_[1])<goal_y_error_ && fabs(marker_pose_current[2]-goal_position_[2])< goal_theta_error_)
                  {
                    ROS_ERROR("%f %f ,%f %f, %f %f", fabs(marker_pose_current[2]-goal_position_[2]) ,goal_theta_error_, fabs(marker_pose_current[1] - goal_position_[1]) ,goal_y_error_,fabs(marker_pose_current[0]-goal_position_[0]) ,goal_x_error_ );

                    //如果x轴到达目标值，进入charging模式
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::charging1;
                    mcharge_status_ = CHARGE_STATUS::charging;
                    bw_status_->set_charge_status(mcharge_status_);
                    usefull_num_ = 0;
                    unusefull_num_ = 0;
                    //停止移动
                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                    return;
                  }
                  else
                  {
                    //ROS_ERROR("docking3 %f %f %f ",marker_pose_current[0],marker_pose_current[1],marker_pose_current[2]);

                      //pid 对准
                      float kp_theta = kp_theta_set_;
                      float kd_theta = kp_theta_set_*30*kd_theta_set_;
                      float ki_theta = kp_theta_set_/30/ki_theta_set_;

                      float error_temp1 = marker_pose_current[2] - error_theta_last_;
                      error_theta_sum_ += marker_pose_current[2] - goal_position_[2];
                      if(error_theta_sum_>3.0) error_theta_sum_ = 3.0;
                      if(error_theta_sum_<-3.0) error_theta_sum_ = -3.0;

                      current_vel.angular.z = kp_theta*(marker_pose_current[2]-goal_position_[2]) + kd_theta*error_temp1 + ki_theta*error_theta_sum_;
                      if(current_vel.angular.z > max_theta_speed_ ) current_vel.angular.z = max_theta_speed_;
                      if(current_vel.angular.z < -max_theta_speed_ ) current_vel.angular.z = -max_theta_speed_;

                      float kp_y = kp_y_set_;
                      float kd_y = kp_y_set_*30*kd_y_set_;
                      float ki_y = kp_y_set_/30/ki_y_set_;

                      error_temp1 = marker_pose_current[1] - error_y_last_;
                      error_y_sum_ += marker_pose_current[1] - goal_position_[1];
                      if(error_y_sum_>3.0) error_y_sum_ = 3.0;
                      if(error_y_sum_<-3.0) error_y_sum_ = -3.0;

                      current_vel.linear.y = kp_y*(marker_pose_current[1]-goal_position_[1]) + kd_y*error_temp1 + ki_y*error_y_sum_;
                      if(current_vel.linear.y > max_y_speed_ ) current_vel.linear.y = max_y_speed_;
                      if(current_vel.linear.y < -max_y_speed_ ) current_vel.linear.y = -max_y_speed_;

                      float kp_x = kp_x_set_;
                      float kd_x = kp_x_set_*30*kd_x_set_;
                      float ki_x = kp_x_set_/30/ki_x_set_;

                      error_temp1 = marker_pose_current[0] - error_x_last_;
                      error_x_sum_ += marker_pose_current[0] - goal_position_[0];
                      if(error_x_sum_>3.0) error_x_sum_ = 3.0;
                      if(error_x_sum_<-3.0) error_x_sum_ = -3.0;

                      current_vel.linear.x = kp_x*(marker_pose_current[0]-goal_position_[0]) + kd_x*error_temp1 + ki_x*error_x_sum_;
                      if(current_vel.linear.x > max_x_speed_ ) current_vel.linear.x = max_x_speed_;
                      if(current_vel.linear.x < -max_x_speed_ ) current_vel.linear.x = -max_x_speed_;

                      usefull_num_++;
                      current_vel.linear.z = 0;
                      current_vel.angular.x = 0;
                      current_vel.angular.y = 0;
                      mCmdvelPub_.publish(current_vel);
                      ROS_DEBUG("docking3.2 %d", usefull_num_);

                      error_theta_last_ = marker_pose_current[2];
                      error_y_last_ = marker_pose_current[1];
                      error_x_last_ = marker_pose_current[0];
                  }
                }
                break;
            case CHARGE_STATUS_TEMP::charging1:
                {
                  ROS_DEBUG("charging1.0");
                  mcharge_status_temp_ = CHARGE_STATUS_TEMP::charged1;
                  mcharge_status_ = CHARGE_STATUS::charged;
                  bw_status_->set_charge_status(mcharge_status_);
                  usefull_num_ = 0;
                  unusefull_num_ = 0;
                  //停止移动
                  current_vel.linear.x = 0;
                  current_vel.linear.y = 0;
                  current_vel.linear.z = 0;
                  current_vel.angular.x = 0;
                  current_vel.angular.y = 0;
                  current_vel.angular.z = 0;
                  mCmdvelPub_.publish(current_vel);
                }
                break;
            case CHARGE_STATUS_TEMP::charged1:
                {
                  ROS_DEBUG("charged1");
                  return;
                  if (usefull_num_ > 18000 || bw_status_->sensor_status.battery > power_threshold_)
                  {
                      usefull_num_ = 0;
                      unusefull_num_ = 0;
                      mcurrentChargeFlag_ = false;
                  }
                  else
                  {
                      usefull_num_++;
                  }
                  //停止移动
                  current_vel.linear.x = 0;
                  current_vel.linear.y = 0;
                  current_vel.linear.z = 0;
                  current_vel.angular.x = 0;
                  current_vel.angular.y = 0;
                  current_vel.angular.z = 0;
                  mCmdvelPub_.publish(current_vel);
                }
                break;
            default:
                 ROS_DEBUG("default %d",(int)mcharge_status_temp_);
                 return;
        }
    }
    else
    {
        if(mcharge_status_ == CHARGE_STATUS::charging || mcharge_status_ == CHARGE_STATUS::charged)
        {
          //先进入temp3,前进到mstationPose3_,再转入free
          error_theta_last_ = 0;
          error_theta_sum_ = 0;
          error_y_last_ = 0;
          error_y_sum_ = 0;
          error_x_last_ = 0;
          error_x_sum_= 0;
          mcharge_status_temp_ = CHARGE_STATUS_TEMP::temp3;
        }

        if(mcharge_status_temp_ == CHARGE_STATUS_TEMP::temp3)
        {
          //运动到目标点4
          ROS_DEBUG("temp3.1 ");
          if (this->goToStation3())
          {
              //转入free
              ROS_DEBUG("temp3.2 ");
              mcharge_status_temp_ = CHARGE_STATUS_TEMP::freed;
              //停止前进，
              current_vel.linear.x = 0;
              current_vel.linear.y = 0;
              current_vel.linear.z = 0;
              current_vel.angular.x = 0;
              current_vel.angular.y = 0;
              current_vel.angular.z = 0;
              mCmdvelPub_.publish(current_vel);
          }
          else
          {
              return;
          }
        }
        if (mcharge_status_ == CHARGE_STATUS::docking || mcharge_status_ == CHARGE_STATUS::finding)
        {
            //停止移动
            current_vel.linear.x = 0;
            current_vel.linear.y = 0;
            current_vel.linear.z = 0;
            current_vel.angular.x = 0;
            current_vel.angular.y = 0;
            current_vel.angular.z = 0;
            mCmdvelPub_.publish(current_vel);
        }
        mcharge_status_ = CHARGE_STATUS::freed;
        mcharge_status_temp_ = CHARGE_STATUS_TEMP::freed;
        bw_status_->set_charge_status(mcharge_status_);
    }
}

void DockController::caculatePose3()
{
    float theta1, theta2, x, y, theta;

    tf::Quaternion q1(mPose1_.orientation.x, mPose1_.orientation.y, mPose1_.orientation.z, mPose1_.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta1 = yaw;

    tf::Quaternion q2(mPose1_.orientation.x, mPose1_.orientation.y, mPose1_.orientation.z, mPose1_.orientation.w);
    tf::Matrix3x3 m2(q2);
    m2.getRPY(roll, pitch, yaw);
    theta2 = yaw;

    x = (mPose1_.position.x + mPose2_.position.x) / 2.0f;
    y = (mPose1_.position.y + mPose2_.position.y) / 2.0f;
    theta = (theta1 + theta2) / 2.0f;

    mPose3_[0] = x - back_distance_ * cos(theta);
    mPose3_[1] = y - back_distance_ * sin(theta);
    mPose3_[2] = theta;
    //ROS_DEBUG("theta12  %f  %f x1 y1 %f %f x2 y2 %f %f ", theta1, theta2, mPose1_.position.x, mPose1_.position.y,
    //            mPose2_.position.x, mPose2_.position.y);
}

bool DockController::backToPose3()
{
    geometry_msgs::Pose current_pose = mRobot_pose_;
    float x, y, theta, x2, y2;
    x = current_pose.position.x;
    y = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;
    //转到pose3本地坐标系
    x2 = cos(mPose3_[2]) * (x - mPose3_[0]) + sin(mPose3_[2]) * (y - mPose3_[1]);
    y2 = -sin(mPose3_[2]) * (x - mPose3_[0]) + cos(mPose3_[2]) * (y - mPose3_[1]);

    //ROS_DEBUG("theta3  %f  %f x3 y3 %f %f x y %f %f x2 y2 %f %f ", mPose3_[2], theta, mPose3_[0], mPose3_[1], x, y, x2,
    //          y2);

    if (fabs(x2) <= 0.03)
        return true;
    geometry_msgs::Twist current_vel;
    current_vel.linear.x = -0.1;
    current_vel.linear.y = 0;
    current_vel.linear.z = 0;
    current_vel.angular.x = 0;
    current_vel.angular.y = 0;
    current_vel.angular.z = 0;
    mCmdvelPub_.publish(current_vel);
    return false;
}

bool DockController::backToDock()
{
    geometry_msgs::Pose current_pose = mRobot_pose_;
    float x, y, theta;
    x = current_pose.position.x;
    y = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;
    //如果侦测到电压，停止
    if (bw_status_->sensor_status.power > 9.0)
        return true;

    //如果侦测到已进入死角，停止
    if (bw_status_->sensor_status.distance1 <= this->crash_distance_ && bw_status_->sensor_status.distance1>0.1)
        return true;

    if ((bw_status_->sensor_status.left_sensor2 == 0) && (bw_status_->sensor_status.right_sensor2 == 0))
    {
        usefull_num_++;
        unusefull_num_ = 0;
        if (usefull_num_ > 8)
        {
            return true;
        }
    }
    else
    {
        unusefull_num_++;
        usefull_num_ = 0;
    }

    geometry_msgs::Twist current_vel;
    current_vel.linear.x = -0.1;
    current_vel.linear.y = 0;
    current_vel.linear.z = 0;
    current_vel.angular.x = 0;
    current_vel.angular.y = 0;
    current_vel.angular.z = 0;
    // PID对准
    const float Ts = 1.0f / 30.0f;
    float kp = kp_, ki = kp_ * Ts / ki_, kd = kd_ * kp_ / Ts;

    left2_error1_ = this->computeDockError();

    float rot_error_temp1, rot_error_temp2, rot_delta;
    rot_error_temp1 = left2_error1_ - left2_error2_;
    rot_error_temp2 = left2_error1_ - 2 * left2_error2_ + left2_error3_;
    rot_delta = kp * rot_error_temp1 + ki * left2_error1_ + kd * rot_error_temp2;
    //ROS_DEBUG("ousp1 delta  %f  error %f %f %f sonsor %d ", rot_delta, left2_error1_, rot_error_temp1, rot_error_temp2,
    //          bw_status_->sensor_status.left_sensor2);
    // rot_error_temp1 = right2_error1_ - right2_error2_;
    // rot_error_temp2 = right2_error1_ - 2*right2_error2_ + right2_error3_;
    // rot_delta += kp*rot_error_temp1 + ki*right2_error1_ + kd*rot_error_temp2;
    // ROS_DEBUG("ousp2 delta  %f  error %f %f %f sonsor %d
    // ",rot_delta,right2_error1_,rot_error_temp1,rot_error_temp2,bw_status_->sensor_status.right_sensor2);
    if (rot_delta > max_rotspeed_)
        rot_delta = max_rotspeed_;
    if (rot_delta < -max_rotspeed_)
        rot_delta = -max_rotspeed_;
    rot_z_ = rot_delta;
    //ROS_DEBUG("ousp3 rot  %f", rot_z_);
    left2_error3_ = left2_error2_;
    left2_error2_ = left2_error1_;
    // right2_error3_ = right2_error2_;
    // right2_error2_ = right2_error1_;
    current_vel.angular.z = rot_z_;
    mCmdvelPub_.publish(current_vel);
    return false;
}

float DockController::computeDockError()
{
    int l2 = bw_status_->sensor_status.left_sensor2;
    int r2 = bw_status_->sensor_status.right_sensor2;
    float return_value = 0.0;
    if (l2 == 0)
    {
        if (r2 == 0)
            return return_value;
        return_value = 1.0;
    }
    else if (r2 == 0)
    {
        if (l2 == 0)
            return return_value;
        return_value = -1.0;
    }
    else
    {
        switch (r2)
        {
            case 1:
            case 5:
                if (l2 == 2 || l2 == 3 || l2 == 6 || l2 == 7)
                    return return_value;
                return_value = 1.0;
                break;
            case 2:
            case 3:
            case 6:
            case 7:
                if (l2 == 1 || l2 == 3 || l2 == 5 || l2 == 7)
                    return return_value;
                if (l2 == 2 || l2 == 6)
                    return_value = -1.0;
                if (l2 == 4)
                    return_value = 1.0;
                break;
            case 4:
                if (l2 == 4)
                    return return_value;
                return_value = -1.0;
                break;
        }
    }
    return return_value;
}

void DockController::setDockPid(double kp_theta_set,double  kd_theta_set,double  ki_theta_set,double kp_y_set,double  kd_y_set,double  ki_y_set,double  kp_x_set,double  kd_x_set,double  ki_x_set)
{
  kp_theta_set_ = kp_theta_set;
  kd_theta_set_ = kd_theta_set;
  ki_theta_set_ = ki_theta_set;

  kp_y_set_ = kp_y_set;
  kd_y_set_ = kd_y_set;
  ki_y_set_ = ki_y_set;

  kp_x_set_ = kp_x_set;
  kd_x_set_ = kd_x_set;
  ki_x_set_ = ki_x_set;
}

void DockController::setScaleParam(double theta_min_set,double  y_min_set,double  x_min_set,double  max_x_speed,double  max_y_speed,double  max_theta_speed,double goal_theta_error,double goal_y_error,double goal_x_error)
{
  theta_min_set_ = theta_min_set;
  y_min_set_ = y_min_set;
  x_min_set_ = x_min_set;

  max_theta_speed_ = max_theta_speed;
  max_y_speed_ = max_y_speed;
  max_x_speed_ = max_y_speed;

  goal_theta_error_ = goal_theta_error;
  goal_y_error_ = goal_y_error;
  goal_x_error_ = goal_x_error;

}

bool DockController::rotateOrigin()
{
    geometry_msgs::Pose current_pose = mRobot_pose_;
    float x, y, theta;
    x = current_pose.position.x;
    y = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;
    static float target_theta = 0;
    if (usefull_num_ == 0)
    {
        //先正转20度
        target_theta = theta + 10.f / 180.f * PI_temp;
        if (target_theta <= -PI_temp)
            target_theta += 2 * PI_temp;
        if (target_theta > PI_temp)
            target_theta -= 2 * PI_temp;
        usefull_num_++;
    }
    else if (usefull_num_ == 2)
    {
        //再反转40度
        target_theta = theta - 20.f / 180.f * PI_temp;
        if (target_theta <= -PI_temp)
            target_theta += 2 * PI_temp;
        if (target_theta > PI_temp)
            target_theta -= 2 * PI_temp;
        usefull_num_++;
    }
    else if (usefull_num_ == 4)
    {
        //最后正转20度
        target_theta = theta + 10.f / 180.f * PI_temp;
        if (target_theta <= -PI_temp)
            target_theta += 2 * PI_temp;
        if (target_theta > PI_temp)
            target_theta -= 2 * PI_temp;
        usefull_num_++;
    }
    if (usefull_num_ == 1 || usefull_num_ == 5)
    {
        //正转
        geometry_msgs::Twist current_vel;
        current_vel.linear.x = 0;
        current_vel.linear.y = 0;
        current_vel.linear.z = 0;
        current_vel.angular.x = 0;
        current_vel.angular.y = 0;
        current_vel.angular.z = 0.2;
        mCmdvelPub_.publish(current_vel);
    }
    else if (usefull_num_ == 3)
    {
        //反转
        geometry_msgs::Twist current_vel;
        current_vel.linear.x = 0;
        current_vel.linear.y = 0;
        current_vel.linear.z = 0;
        current_vel.angular.x = 0;
        current_vel.angular.y = 0;
        current_vel.angular.z = -0.2;
        mCmdvelPub_.publish(current_vel);
    }
    float theta_error = theta - target_theta;
    if (theta_error <= -PI_temp)
        theta_error += 2 * PI_temp;
    if (theta_error > PI_temp)
        theta_error -= 2 * PI_temp;
    if (fabs(theta_error) < 0.02)
    {
        //到达目标角度停止，切换方向
        geometry_msgs::Twist current_vel;
        current_vel.linear.x = 0;
        current_vel.linear.y = 0;
        current_vel.linear.z = 0;
        current_vel.angular.x = 0;
        current_vel.angular.y = 0;
        current_vel.angular.z = 0;
        mCmdvelPub_.publish(current_vel);
        usefull_num_++;
    }
    if (usefull_num_ == 6)
    {
        geometry_msgs::Twist current_vel;
        current_vel.linear.x = 0;
        current_vel.linear.y = 0;
        current_vel.linear.z = 0;
        current_vel.angular.x = 0;
        current_vel.angular.y = 0;
        current_vel.angular.z = 0;
        mCmdvelPub_.publish(current_vel);
        return true;
    }
    return false;
}

void DockController::caculatePose4()
{
    //当前角度设为目标角度，当前位置在pose3坐标系下的x轴位置，得到目标点位置
    geometry_msgs::Pose current_pose = mRobot_pose_;
    float x, y, theta, x2;
    x = current_pose.position.x;
    y = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;

    mPose4_[2] = theta;
    //转到pose3本地坐标系,得到x轴值
    x2 = cos(mPose3_[2]) * (x - mPose3_[0]) + sin(mPose3_[2]) * (y - mPose3_[1]);
    //转到全局坐标系
    mPose4_[0] = mPose3_[0] + x2 * cos(mPose3_[2]);
    mPose4_[1] = mPose3_[1] + x2 * sin(mPose3_[2]);
}

bool DockController::goToPose4()
{
    static float last_x2 = 0;
    geometry_msgs::Pose current_pose = mRobot_pose_;
    float x, y, theta, x2, y2;
    x = current_pose.position.x;
    y = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;
    //转到pose4本地坐标系
    x2 = cos(mPose4_[2]) * (x - mPose4_[0]) + sin(mPose4_[2]) * (y - mPose4_[1]);
    y2 = -sin(mPose4_[2]) * (x - mPose4_[0]) + cos(mPose4_[2]) * (y - mPose4_[1]);

    if(min_x2_4_>99) last_x2 = x2;
    if(fabs(x2)<min_x2_4_) min_x2_4_ = fabs(x2);

    //ROS_ERROR("temp error3 %f %f %f ; %f %f",x,y,yaw, x2,y2);
    //增加过零检查和发散检查
    if (fabs(x2) <= 0.03)
        return true;
    if((x2*last_x2) < 0.0001) return true; //过最小值
    if(fabs(min_x2_4_ - fabs(x2)) > 0.2) return true; //发散

    last_x2 = x2;

    geometry_msgs::Twist current_vel;
    current_vel.linear.x = 0.2;
    current_vel.linear.y = 0;
    current_vel.linear.z = 0;
    current_vel.angular.x = 0;
    current_vel.angular.y = 0;
    current_vel.angular.z = 0;
    mCmdvelPub_.publish(current_vel);
    return false;
}

geometry_msgs::Pose DockController::getRobotPose()
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    return mRobot_pose_;
}
bool DockController::getRobotPose(float (&robot_pose)[3])
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    if (!mPose_flag_)
        return false;
    geometry_msgs::Pose current_pose = mRobot_pose_;
    robot_pose[0] = current_pose.position.x;
    robot_pose[1] = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    robot_pose[2] = yaw;
    return true;
}

bool DockController::getIRPose(float (&robot_pose)[3])
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    if (!mPose_flag_)
        return false;
    geometry_msgs::Pose current_pose = mRobot_pose_;

    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    robot_pose[2] = yaw;
    robot_pose[0] = current_pose.position.x - back_distance_ * cos(yaw);
    robot_pose[1] = current_pose.position.y - back_distance_ * sin(yaw);
    return true;
}

void DockController::setDockPositionCaculate(CaculateDockPosition* dock_position_caculate)
{
    mdock_position_caculate_ = dock_position_caculate;
}

void DockController::caculateStation3()
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    mstationPose3_[0] = (mstationPose1_[0] + mstationPose2_[0])/2.0;
    mstationPose3_[1] = (mstationPose1_[1] + mstationPose2_[1])/2.0;
    return;
}

bool DockController::rotate2Station3()
{
    //旋转到正对目标点3
    geometry_msgs::Pose current_pose = mRobot_pose_;

    float x, y, theta;
    x = current_pose.position.x;
    y = current_pose.position.y;
    tf::Quaternion q1(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z,
                      current_pose.orientation.w);
    tf::Matrix3x3 m1(q1);

    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    theta = yaw;
    //ROS_ERROR("temp error %f %f %f ; %f %f %f",roll,pitch,yaw, mstationPose3_[0],mstationPose3_[1],mstationPose3_[2]);
    if (fabs(theta - mstationPose3_[2]) < 0.02)
    {
        //ROS_ERROR("temp error2 %f %f %f ; %f %f %f",x,y,yaw, mstationPose3_[0],mstationPose3_[1],mstationPose3_[2]);
        return true;
    }
    else
    {
        geometry_msgs::Twist current_vel;
        current_vel.linear.x = 0;
        current_vel.linear.y = 0;
        current_vel.linear.z = 0;
        current_vel.angular.x = 0;
        current_vel.angular.y = 0;
        current_vel.angular.z = 0;
        mCmdvelPub_.publish(current_vel);

        float delta_theta = theta - mstationPose3_[2];

        if (delta_theta > 0.001)
        {
            if (delta_theta < (PI_temp + 0.001))
            {
                //反转
                current_vel.angular.z = -0.3;
            }
            else
            {
                //正转
                current_vel.angular.z = 0.3;
            }
        }
        else
        {
            if (delta_theta < (-PI_temp - 0.001))
            {
                //反转
                current_vel.angular.z = -0.3;
            }
            else
            {
                //正转
                current_vel.angular.z = 0.3;
            }
        }
        mCmdvelPub_.publish(current_vel);
    }
    return false;
}

bool DockController::goToStation3()
{
   //todo
    // return false;
    boost::mutex::scoped_lock lock(mMutex_pose);
    if(fabs(local_station3_pose_.point.x)<= 0.05 && fabs(local_station3_pose_.point.y)<= 0.05)
    {
      //ROS_ERROR("goToStation3 %f %f ", fabs(local_station3_pose_.point.x),fabs(local_station3_pose_.point.y) );
      return true;
    }
    else
    {
      //ROS_ERROR("goToStation3 %f %f ", local_station3_pose_.point.x,local_station3_pose_.point.y );

      geometry_msgs::Twist current_vel;
      //pid 对准
      float kp_y = kp_y_set_;
      float kd_y = kp_y_set_*30*kd_y_set_;
      float ki_y = kp_y_set_/30/ki_y_set_;

      float error_temp1 = local_station3_pose_.point.y - error_y_last_;
      error_y_sum_ += local_station3_pose_.point.y;
      if(error_y_sum_>3.0) error_y_sum_ = 3.0;
      if(error_y_sum_<-3.0) error_y_sum_ = -3.0;

      current_vel.linear.y = kp_y*local_station3_pose_.point.y + kd_y*error_temp1 + ki_y*error_y_sum_;
      if(current_vel.linear.y > max_y_speed_ ) current_vel.linear.y = max_y_speed_;
      if(current_vel.linear.y < -max_y_speed_ ) current_vel.linear.y = -max_y_speed_;

      float kp_x = kp_x_set_;
      float kd_x = kp_x_set_*30*kd_x_set_;
      float ki_x = kp_x_set_/30/ki_x_set_;

      error_temp1 = local_station3_pose_.point.x - error_x_last_;
      error_x_sum_ += local_station3_pose_.point.x;
      if(error_x_sum_>3.0) error_x_sum_ = 3.0;
      if(error_x_sum_<-3.0) error_x_sum_ = -3.0;

      current_vel.linear.x = kp_x*local_station3_pose_.point.x + kd_x*error_temp1 + ki_x*error_x_sum_;
      if(current_vel.linear.x > max_x_speed_ ) current_vel.linear.x = max_x_speed_;
      if(current_vel.linear.x < -max_x_speed_ ) current_vel.linear.x = -max_x_speed_;

      usefull_num_++;
      current_vel.linear.z = 0;
      current_vel.angular.x = 0;
      current_vel.angular.y = 0;
      current_vel.angular.z = 0;
      mCmdvelPub_.publish(current_vel);

      error_y_last_ = local_station3_pose_.point.y;
      error_x_last_ = local_station3_pose_.point.x;
    }
}

}  // namespace bw_auto_dock
