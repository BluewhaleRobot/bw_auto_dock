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
namespace bw_auto_dock
{
DockController::DockController(double back_distance, double max_linearspeed, double max_rotspeed,
                             StatusPublisher* bw_status, CallbackAsyncSerial* cmd_serial)
{
    back_distance_ = back_distance;
    max_linearspeed_ = max_linearspeed;
    max_rotspeed_ = max_rotspeed;
    bw_status_ = bw_status;
    mcmd_serial_ = cmd_serial;
    mcharge_status_ = CHARGE_STATUS::freed;
    bw_status_->set_charge_status(mcharge_status_);
    mPose3_ = new float[3];
    mPose4_ = new float[3];

    mstationPose3_ = new float[3];

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
}

void DockController::run()
{
    ros::NodeHandle nodeHandler;
    mCmdvelPub_ = nodeHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    mbarDetectPub_ = nodeHandler.advertise<std_msgs::Bool>("/barDetectFlag", 1, true);
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
    if (mcurrentChargeFlag_)
    {
        //关闭红外避障
        std_msgs::Bool pub_data;
        pub_data.data = false;
        mbarDetectPub_.publish(pub_data);
    }
    //下发充电开关闭命令
    char cmd_str[6] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x4B, (char)0x00 };
    if (NULL != mcmd_serial_)
    {
        mcmd_serial_->write(cmd_str, 6);
    }
}

void DockController::updateOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(mMutex_pose);
    mRobot_pose_ = msg->pose.pose;
    mPose_flag_ = true;
}

void DockController::dealing_status()
{
    boost::mutex::scoped_lock lock1(mMutex_charge);
    boost::mutex::scoped_lock lock2(mMutex_pose);
    geometry_msgs::Twist current_vel;
    if (!mPose_flag_)
        return;  //历程计没有开启
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
        DOCK_POSITION dock_position_current = bw_status_->get_dock_position();
        switch (mcharge_status_temp_)
        {
            case CHARGE_STATUS_TEMP::finding0:
                if (usefull_num_ == 0)
                {
                    //根据充电桩位置,计算两个移动参考点
                    usefull_num_++;
                    if (mdock_position_caculate_->getDockPosition(mstationPose1_, mstationPose2_))
                    {
                        //选择距离更远的站点当成station3
                        this->caculateStation3();
                    }
                    else
                    {
                        // error,没有设置充电桩位置
                        ROS_ERROR("Can not get dock station position!");
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
                    if (this->rotate2Station3())
                    {
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
                //往前直线运动，探测到left_center或者right_center后记录当期位置，同时降低进入finding2
                if (dock_position_current == DOCK_POSITION::left_center ||
                    dock_position_current == DOCK_POSITION::right_center)
                {
                    mdock__referenss_position_ = dock_position_current;
                    mPose1_ = mRobot_pose_;
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding2;
                    usefull_num_ = 0;
                    unusefull_num_ = 0;
                    current_vel.linear.x = 0.1;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                }
                else
                {
                    if (this->goToStation3())
                    {
                        //没有捕获
                        mcharge_status_ = CHARGE_STATUS::finding;
                        mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding0;
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
                if (dock_position_current == DOCK_POSITION::back_center &&
                    (bw_status_->sensor_status.left_sensor2 == 3 || bw_status_->sensor_status.left_sensor2 == 7) &&
                    (bw_status_->sensor_status.right_sensor2 == 3 || bw_status_->sensor_status.right_sensor2 == 7))
                {
                    mPose1_ = mRobot_pose_;
                    mPose2_ = mRobot_pose_;
                    this->caculatePose3();
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding4;
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
                break;
            case CHARGE_STATUS_TEMP::finding2:
                //往前直线运动，探测不到left_center或者right_center后记录当期位置，同时进入finding3
                if (dock_position_current != DOCK_POSITION::left_center &&
                    dock_position_current != DOCK_POSITION::right_center)
                {
                    mPose2_ = mRobot_pose_;
                    this->caculatePose3();
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding3;
                    //停止前进，
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
                else
                {
                    current_vel.linear.x = 0.1;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                }
                if (dock_position_current == DOCK_POSITION::back_center)
                {
                    mPose1_ = mRobot_pose_;
                    mPose2_ = mRobot_pose_;
                    this->caculatePose3();
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding4;
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
                break;
            case CHARGE_STATUS_TEMP::finding3:
                //后退，直到到达目标pose3同时进入finding4
                if (this->backToPose3())
                {
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding4;
                    //停止前进，
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
                break;
            case CHARGE_STATUS_TEMP::finding4:
            {
                //原地旋转，直到出现DOCK_POSITION::back_center，进入docking1
                static float target_theta = 0;
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
                    target_theta = theta;
                }
                else
                {
                    float theta_error = theta - target_theta;
                    if (theta_error <= -PI)
                        theta_error += 2 * PI;
                    if (theta_error > PI)
                        theta_error -= 2 * PI;
                    if (fabs(theta_error) >= (0.9 * PI))
                    {
                        //旋转超过162度后还是没有发现，进入finding0
                        mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding0;
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
                // if(dock_position_current == DOCK_POSITION::back_center &&
                // (((bw_status_->sensor_status.left_sensor2==3||bw_status_->sensor_status.left_sensor2==7) &&
                // mdock__referenss_position_ == DOCK_POSITION::right_center
                // )||((bw_status_->sensor_status.right_sensor2==3 ||bw_status_->sensor_status.right_sensor2==7)  &&
                // mdock__referenss_position_ == DOCK_POSITION::left_center)) )
                // if(((bw_status_->sensor_status.left_sensor2==3||bw_status_->sensor_status.left_sensor2==7) &&
                // mdock__referenss_position_ == DOCK_POSITION::right_center
                // )||((bw_status_->sensor_status.right_sensor2==3 ||bw_status_->sensor_status.right_sensor2==7)  &&
                // mdock__referenss_position_ == DOCK_POSITION::left_center) )
                if (dock_position_current == DOCK_POSITION::back_center)
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
                    if (mdock__referenss_position_ == DOCK_POSITION::left_center)
                    {
                        //右转
                        current_vel.angular.z = -0.3;
                    }
                    else
                    {
                        //左转
                        current_vel.angular.z = 0.3;
                    }
                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    mCmdvelPub_.publish(current_vel);
                }
                break;
            }
            case CHARGE_STATUS_TEMP::docking1:
                // pid方式对准充电桩前进，当出现充电电压后停止，当侦测到碰上角落后也停止
                if (this->backToDock())
                {
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
                break;
            case CHARGE_STATUS_TEMP::docking2:
                if (bw_status_->sensor_status.power > 9.0)
                {
                    //已经侦测到电压，进入充电状态
                    ROS_DEBUG("docking2.1 %f", bw_status_->sensor_status.power);
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
                    //触发了碰撞传感器
                    if (bw_status_->sensor_status.left_switch1 == 1 || bw_status_->sensor_status.right_switch1 == 1)
                    {
                        //进入docking3
                        ROS_DEBUG("docking2.2");
                        mcharge_status_temp_ = CHARGE_STATUS_TEMP::docking3;
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
                        //原地旋转，如果正反旋转了20度，还是没有触发，则进入temp1
                        ROS_DEBUG("docking2.3");
                        usefull_num_++;
                        // if(this->rotateOrigin())
                        if (usefull_num_ >= 10)
                        {
                            ROS_DEBUG("docking2.4");
                            this->caculatePose4();
                            mcharge_status_temp_ = CHARGE_STATUS_TEMP::temp1;
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
                }
                break;
            case CHARGE_STATUS_TEMP::docking3:
                //往前移动，直到不会触发碰撞

                if (bw_status_->sensor_status.left_switch1 == 0 && bw_status_->sensor_status.right_switch1 == 0)
                {
                    //进入docking2
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::docking2;
                    usefull_num_ = 0;
                    unusefull_num_ = 0;
                    //停止移动
                    geometry_msgs::Twist current_vel;
                    current_vel.linear.x = 0;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                    ROS_DEBUG("docking3.1 %d", usefull_num_);
                }
                else
                {
                    usefull_num_++;
                    geometry_msgs::Twist current_vel;
                    current_vel.linear.x = 0.1;
                    current_vel.linear.y = 0;
                    current_vel.linear.z = 0;
                    current_vel.angular.x = 0;
                    current_vel.angular.y = 0;
                    current_vel.angular.z = 0;
                    mCmdvelPub_.publish(current_vel);
                    ROS_DEBUG("docking3.2 %d", usefull_num_);
                }
                break;
            case CHARGE_STATUS_TEMP::temp1:
                //运动到目标点4
                ROS_DEBUG("temp1.1 ");
                if (this->goToPose4())
                {
                    ROS_DEBUG("temp1.2 ");
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::temp2;
                    //停止前进，
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
                break;
            case CHARGE_STATUS_TEMP::temp2:
            {
                //直接进入finding0,temp1不进入，是因为旋转位置不合适，容易碰到东西
                mcharge_status_ = CHARGE_STATUS::finding;
                mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding0;
                bw_status_->set_charge_status(mcharge_status_);
                usefull_num_ = 0;
                unusefull_num_ = 0;
                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
                break;
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

                if (usefull_num_ == 0)
                {
                    mPose4_[2] = atan2(mPose3_[1] - mPose4_[1], mPose3_[0] - mPose4_[0]);
                    usefull_num_++;
                }

                if (fabs(yaw - mPose4_[2]) < 0.02)
                {
                    ROS_DEBUG("temp2.2 ");
                    mcharge_status_ = CHARGE_STATUS::finding;
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::finding0;
                    bw_status_->set_charge_status(mcharge_status_);
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
                else
                {
                    ROS_DEBUG("temp2.1");
                    if (mPose4_[2] > 0)
                    {
                        //正转
                        current_vel.linear.x = 0;
                        current_vel.linear.y = 0;
                        current_vel.linear.z = 0;
                        current_vel.angular.x = 0;
                        current_vel.angular.y = 0;
                        current_vel.angular.z = 0.2;
                        mCmdvelPub_.publish(current_vel);
                    }
                    else
                    {
                        //反转
                        current_vel.linear.x = 0;
                        current_vel.linear.y = 0;
                        current_vel.linear.z = 0;
                        current_vel.angular.x = 0;
                        current_vel.angular.y = 0;
                        current_vel.angular.z = -0.2;
                        mCmdvelPub_.publish(current_vel);
                    }
                }
            }
            break;
            case CHARGE_STATUS_TEMP::charging1:
                //触发了碰撞传感器
                if (bw_status_->sensor_status.left_switch1 == 1 || bw_status_->sensor_status.right_switch1 == 1)
                {
                    //进入docking3
                    ROS_DEBUG("docking2.2");
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::docking3;
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
                if (bw_status_->sensor_status.power < 9.0)
                {
                    //没有侦测到电压，进入temp1
                    usefull_num_++;
                    if (usefull_num_ > 10)
                    {
                        //下发充电关闭命令
                        char cmd_str[6] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x4B, (char)0x00 };
                        if (NULL != mcmd_serial_)
                        {
                            mcmd_serial_->write(cmd_str, 6);
                        }
                        this->caculatePose4();
                        mcharge_status_temp_ = CHARGE_STATUS_TEMP::temp1;
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
                else
                {
                    unusefull_num_++;
                    if (unusefull_num_ > 20)
                    {
                        //下发充电开关使能命令,进入充电状态,黄灯
                        unusefull_num_ = 21;
                        char cmd_str[6] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x4B, (char)0x01 };
                        if (NULL != mcmd_serial_)
                        {
                            mcmd_serial_->write(cmd_str, 6);
                        }
                        //根据充电电流，判断是否已经充满
                        current_average_ = current_average_ * 0.99 + bw_status_->sensor_status.current * 0.01;
                        if ((current_average_) < 0.1)
                        {
                            //进入充满状态
                            //下发充满显示状态使能命令，绿灯
                            char cmd_str[6] = {
                                (char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x4B, (char)0x02
                            };
                            if (NULL != mcmd_serial_)
                            {
                                mcmd_serial_->write(cmd_str, 6);
                            }
                            mcharge_status_temp_ = CHARGE_STATUS_TEMP::charged1;
                            mcharge_status_ = CHARGE_STATUS::charged;
                            bw_status_->set_charge_status(mcharge_status_);
                            usefull_num_ = 0;
                            unusefull_num_ = 0;
                            //开启红外避障
                            std_msgs::Bool pub_data;
                            pub_data.data = true;
                            mbarDetectPub_.publish(pub_data);
                        }
                    }
                }
                //停止移动
                current_vel.linear.x = 0;
                current_vel.linear.y = 0;
                current_vel.linear.z = 0;
                current_vel.angular.x = 0;
                current_vel.angular.y = 0;
                current_vel.angular.z = 0;
                mCmdvelPub_.publish(current_vel);
                break;
            case CHARGE_STATUS_TEMP::charged1:
                if (usefull_num_ > 18000)
                {
                    // 10分钟后转成freed
                    //进入free显示状态，下发充电开关闭命令，关闭灯
                    char cmd_str[6] = { (char)0xcd, (char)0xeb, (char)0xd7, (char)0x02, (char)0x4B, (char)0x00 };
                    if (NULL != mcmd_serial_)
                    {
                        mcmd_serial_->write(cmd_str, 6);
                    }
                    mcharge_status_temp_ = CHARGE_STATUS_TEMP::freed;
                    mcharge_status_ = CHARGE_STATUS::freed;
                    bw_status_->set_charge_status(mcharge_status_);
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
                break;
        }
    }
    else
    {
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
    ROS_DEBUG("theta12  %f  %f x1 y1 %f %f x2 y2 %f %f ", theta1, theta2, mPose1_.position.x, mPose1_.position.y,
              mPose2_.position.x, mPose2_.position.y);
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

    ROS_DEBUG("theta3  %f  %f x3 y3 %f %f x y %f %f x2 y2 %f %f ", mPose3_[2], theta, mPose3_[0], mPose3_[1], x, y, x2,
              y2);

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
    if (bw_status_->sensor_status.left_switch1 == 1 || bw_status_->sensor_status.right_switch1 == 1)
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
    ROS_DEBUG("ousp1 delta  %f  error %f %f %f sonsor %d ", rot_delta, left2_error1_, rot_error_temp1, rot_error_temp2,
              bw_status_->sensor_status.left_sensor2);
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
    ROS_DEBUG("ousp3 rot  %f", rot_z_);
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

void DockController::setDockPid(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
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
        target_theta = theta + 10.f / 180.f * PI;
        if (target_theta <= -PI)
            target_theta += 2 * PI;
        if (target_theta > PI)
            target_theta -= 2 * PI;
        usefull_num_++;
    }
    else if (usefull_num_ == 2)
    {
        //再反转40度
        target_theta = theta - 20.f / 180.f * PI;
        if (target_theta <= -PI)
            target_theta += 2 * PI;
        if (target_theta > PI)
            target_theta -= 2 * PI;
        usefull_num_++;
    }
    else if (usefull_num_ == 4)
    {
        //最后正转20度
        target_theta = theta + 10.f / 180.f * PI;
        if (target_theta <= -PI)
            target_theta += 2 * PI;
        if (target_theta > PI)
            target_theta -= 2 * PI;
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
    if (theta_error <= -PI)
        theta_error += 2 * PI;
    if (theta_error > PI)
        theta_error -= 2 * PI;
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

    if (fabs(x2) <= 0.03)
        return true;
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
    //选择距离远的点
    geometry_msgs::Pose current_pose = mRobot_pose_;
    float x, y, theta;
    x = current_pose.position.x;
    y = current_pose.position.y;

    float distance1, distance2;
    distance1 = (x - mstationPose1_[0]) * (x - mstationPose1_[0]) + (y - mstationPose1_[1]) * (y - mstationPose1_[1]);
    distance2 = (x - mstationPose2_[0]) * (x - mstationPose2_[0]) + (y - mstationPose2_[1]) * (y - mstationPose2_[1]);
    if (distance1 > distance2  )
    {
        if(distance2>(0.1*0.1))
        {//选2
           mstationPose3_[0] = mstationPose2_[0];
           mstationPose3_[1] = mstationPose2_[1];
        }
        else
        {
	   //选1
           mstationPose3_[0] = mstationPose1_[0];
            mstationPose3_[1] = mstationPose1_[1];
        }
    }
    else
    {
        if(distance1<(0.1*0.1))
        {//选2
           mstationPose3_[0] = mstationPose2_[0];
           mstationPose3_[1] = mstationPose2_[1];
        }
        else
        {
	   //选1
           mstationPose3_[0] = mstationPose1_[0];
            mstationPose3_[1] = mstationPose1_[1];
        }
    }
    theta = atan2(mstationPose3_[1] - y, mstationPose3_[0] - x);

    mstationPose3_[2] = theta;
    //  ROS_INFO("station3 %f %f %f %f %f %f %f %f
    //  %f",mstationPose1_[0],mstationPose1_[1],mstationPose2_[0],mstationPose2_[1],mstationPose3_[0],mstationPose3_[1],mstationPose3_[2],x,y);
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

    if (fabs(theta - mstationPose3_[2]) < 0.02)
    {
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
            if (delta_theta < (PI + 0.001))
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
            if (delta_theta < (-PI - 0.001))
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
    x2 = cos(mstationPose3_[2]) * (x - mstationPose3_[0]) + sin(mstationPose3_[2]) * (y - mstationPose3_[1]);
    y2 = -sin(mstationPose3_[2]) * (x - mstationPose3_[0]) + cos(mstationPose3_[2]) * (y - mstationPose3_[1]);

    if (fabs(x2) <= 0.03)
        return true;
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

}  // namespace bw_auto_dock
