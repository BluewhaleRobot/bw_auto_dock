#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

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

#define PI 3.14159265

namespace bw_auto_dock
{
typedef struct {
    float power;//电源电压【1.0 4.0】v
    float current;//充电电流
    unsigned int left_sensor1;
    unsigned int left_sensor2;
    unsigned int right_sensor2;
    unsigned int right_sensor1;
    unsigned int left_switch1;
    unsigned int right_switch1;
    unsigned int time_stamp;//时间戳
}UPLOAD_STATUS;

typedef enum class Dcharge_status {
freed,
charging,
charged,
finding,
docking,
error   //没有预设充电桩位置
}CHARGE_STATUS;

typedef enum class Ddock_position{
not_found, //0
left, //1
left_center, //2
left_down, //3
left_up, //4
right, //5
right_center, //6
right_down, //7
right_up, //8
back, //9
back_center, //10
back_left, //11
back_right //12
}DOCK_POSITION; //13

class StatusPublisher
{

public:
    StatusPublisher();
    void Refresh();
    void Update(const char *data, unsigned int len);
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

} //namespace xqserial_server


#endif // STATUSPUBLISHER_H
