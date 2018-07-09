#ifndef DOCKCONTROLER_H
#define DOCKCONTROLER_H
#include <ros/ros.h>
#include "bw_auto_dock/StatusPublisher.h"
#include "bw_auto_dock/AsyncSerial.h"
#include <std_msgs/Bool.h>
#include "bw_auto_dock/AsyncSerial.h"

namespace bw_auto_dock
{
class CaculateDockPosition;

typedef enum class Dcharge_status_temp {
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
}CHARGE_STATUS_TEMP;

class DockControler
{
public:
    DockControler(double back_distance,double max_linearspeed,double max_rotspeed,StatusPublisher* bw_status,CallbackAsyncSerial* cmd_serial);
    void run();
    void dealing_status();
    void updateOdom(const nav_msgs::Odometry::ConstPtr& msg);
    void updateChargeFlag(const std_msgs::Bool& currentFlag);
    void caculatePose3();
    bool backToPose3();
    bool backToDock();
    float computeDockError();
    void setDockPid(double kp,double ki,double kd);
    bool rotateOrigin();
    void caculatePose4();
    bool goToPose4();
    geometry_msgs::Pose getRobotPose();
    bool getRobotPose(float (&robot_pose)[3]);
    bool getIRPose(float (&robot_pose)[3]);
    void setDockPositionCaculate(CaculateDockPosition * dock_position_caculate);
    bool goToStation3();
    bool rotate2Station3();
    void caculateStation3();
private:

    CallbackAsyncSerial* mcmd_serial_;
    CaculateDockPosition * mdock_position_caculate_;
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

    StatusPublisher* bw_status_;

    ros::Publisher mCmdvelPub_;
    ros::Publisher mbarDetectPub_;

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
    float* mstationPose3_;
};

}
#endif // DOCKCONTROLER_H
