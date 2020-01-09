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

#ifndef __STATUSPUBLISHER_H__
#define __STATUSPUBLISHER_H__

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <algorithm>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include "bw_auto_dock/AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265

namespace bw_auto_dock
{
typedef struct
{
    float power;    //电源电压【1.0 4.0】v
    float current;  //充电电流
    unsigned int left_sensor1;
    unsigned int left_sensor2;
    unsigned int right_sensor2;
    unsigned int right_sensor1;
    unsigned int left_switch1;
    unsigned int right_switch1;
    unsigned int time_stamp;  //时间戳
    unsigned int version;
} UPLOAD_STATUS;

typedef enum class Dcharge_status
{
    freed,
    charging,
    charged,
    finding,
    docking,
    error  //没有预设充电桩位置
} CHARGE_STATUS;

typedef enum class Ddock_position
{
    not_found,     // 0
    left,          // 1
    left_center,   // 2
    left_down,     // 3
    left_up,       // 4
    right,         // 5
    right_center,  // 6
    right_down,    // 7
    right_up,      // 8
    back,          // 9
    back_center,   // 10
    back_left,     // 11
    back_right     // 12
} DOCK_POSITION;   // 13

class StatusPublisher
{
  public:
    StatusPublisher();
    void Refresh();
    void Update(const char* data, unsigned int len);
    DOCK_POSITION get_dock_position();
    void set_charge_status(CHARGE_STATUS charge_status);
    CHARGE_STATUS get_charge_status();
    UPLOAD_STATUS sensor_status;

  private:
    DOCK_POSITION mdock_position_;
    CHARGE_STATUS mcharge_status_;

    ros::NodeHandle mNH;
    ros::Publisher mIRsensor1Pub;
    ros::Publisher mIRsensor2Pub;
    ros::Publisher mIRsensor3Pub;
    ros::Publisher mIRsensor4Pub;
    ros::Publisher mDockpostionPub;
    ros::Publisher mChargestatusPub;
    ros::Publisher mPowerPub;
    ros::Publisher mCurrentPub;
    ros::Publisher mCrashPub;
    bool mbUpdated;

    boost::mutex mMutex_sensor;
    boost::mutex mMutex_charge;
    boost::mutex mMutex_dock;
};

}  // namespace bw_auto_dock

#endif  // STATUSPUBLISHER_H
