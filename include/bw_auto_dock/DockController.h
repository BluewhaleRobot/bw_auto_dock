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

#ifndef __DockController_H__
#define __DockController_H__
#include <ros/ros.h>
#include "bw_auto_dock/StatusPublisher.h"
#include "bw_auto_dock/AsyncSerial.h"
#include <std_msgs/Bool.h>
#include "bw_auto_dock/AsyncSerial.h"
#include <time.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>


namespace bw_auto_dock
{
class CaculateDockPosition;
typedef enum class Dcharge_status_temp
{
    freed,
    finding0,
    finding1,
    finding2,
    finding3,
    finding4,
    docking1,
    docking2,
    docking3,
    docking4,
    charging1,
    charged1,
    temp1,
    temp2,
    temp3,
} CHARGE_STATUS_TEMP;

class DockController
{
  public:
    DockController(double back_distance, double max_linearspeed, double max_rotspeed,double crash_distance,int barDetectFlag,std::string global_frame, StatusPublisher* bw_status);
    void run();
    void dealing_status();
    void updateOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void updateChargeFlag(const std_msgs::Bool& currentFlag);
    void caculatePose3();
    bool backToPose3();
    bool backToDock();
    float computeDockError();
    void setDockPid(double kp_theta_set,double  kd_theta_set,double  ki_theta_set,double kp_y_set,double  kd_y_set,double  ki_y_set,double  kp_x_set,double  kd_x_set,double  ki_x_set);
    void setScaleParam(double theta_min_set,double  y_min_set,double  x_min_set,double  max_x_speed,double  max_y_speed,double  max_theta_speed,double goal_theta_error,double goal_y_error,double goal_x_error);
    bool rotateOrigin();
    void caculatePose4();
    bool goToPose4();
    geometry_msgs::Pose getRobotPose();
    bool getRobotPose(float (&robot_pose)[3]);
    bool getIRPose(float (&robot_pose)[3]);
    void setDockPositionCaculate(CaculateDockPosition* dock_position_caculate);
    bool goToStation3();
    bool rotate2Station3();
    void caculateStation3();
    void setPowerParam(double power_threshold);
  private:
    CaculateDockPosition* mdock_position_caculate_;
    CHARGE_STATUS mcharge_status_;
    CHARGE_STATUS_TEMP mcharge_status_temp_;
    DOCK_POSITION mdock__referenss_position_;
    geometry_msgs::Pose mRobot_pose_;
    geometry_msgs::Pose mPose1_;
    geometry_msgs::Pose mPose2_;
    float* mPose3_;
    float* mPose4_;

    bool mcurrentChargeFlag_;

    double back_distance_;
    double max_linearspeed_;
    double max_rotspeed_;
    double crash_distance_;

    StatusPublisher* bw_status_;

    ros::Publisher mCmdvelPub_;
    ros::Publisher mbarDetectPub_;
    ros::Publisher mlimitSpeedPub_;

    boost::mutex mMutex_charge;
    boost::mutex mMutex_pose;

    float left2_error1_;
    float left2_error2_;
    float left2_error3_;
    float right2_error1_;
    float right2_error2_;
    float right2_error3_;
    float rot_z_;
    int usefull_num_;
    int unusefull_num_;

    float kp_;
    float ki_;
    float kd_;

    float current_average_;
    bool mPose_flag_;
    float mstationPose1_[2];
    float mstationPose2_[2];
    float mstationPose3_[2];

    float min_x2_;
    float min_x2_4_;
    bool barDetectFlag_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_;

    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::PoseStamped global_pose_;
    geometry_msgs::PointStamped global_station3_pose_;
    geometry_msgs::PointStamped local_station3_pose_;
    std::string global_frame_;
    bool mTf_flag_;
    double power_threshold_;

    float y_min_set_; //对准过程中y轴偏差阈值
    float x_min_set_; //对准过程中x轴偏差阈值,小于这个值需要回退到参考点
    float theta_min_set_; //对准过程中角度偏差阈值

    float max_theta_speed_;
    float max_y_speed_;
    float max_x_speed_;

    float goal_theta_error_;
    float goal_y_error_;
    float goal_x_error_;

    float kp_theta_set_;
    float kd_theta_set_;
    float ki_theta_set_;
    float kp_y_set_;
    float kd_y_set_;
    float ki_y_set_;
    float kp_x_set_;
    float kd_x_set_;
    float ki_x_set_;

    float goal_position_[3];

    float error_theta_last_;
    float error_theta_sum_;
    float error_y_last_;
    float error_y_sum_;
    float error_x_last_;
    float error_x_sum_;
};

}  // namespace bw_auto_dock
#endif  // DockController_H
