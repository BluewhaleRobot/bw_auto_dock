#ifndef GETDOCKPOSITION_H
#define GETDOCKPOSITION_H
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "bw_auto_dock/local_grid.hpp"

namespace bw_auto_dock
{

class CaculateDockPosition
{
public:
    CaculateDockPosition(double grid_length,std::string frame_id,std::string dock_station_filename,DockControler * dock_controler,StatusPublisher* bw_status);
    bool getDockPosition(float (&station_pose1)[2],float (&station_pose2)[2]);
    void run();
    void updateMapsaveFlag(const std_msgs::Bool& currentFlag);
    void saveDockPositon();
private:
  DockControler * mdock_controler_;
  StatusPublisher* mbw_status_;
  LocalGrid * mlocal_grid_;
  std::string mdock_station_filename_;
  float station_distance_;
  boost::mutex mMutex_;

};

}
#endif // GETDOCKPOSITION_H
