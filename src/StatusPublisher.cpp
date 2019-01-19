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

#include "bw_auto_dock/StatusPublisher.h"

#define DISABLE 0
#define ENABLE 1

namespace bw_auto_dock
{
StatusPublisher::StatusPublisher(double crash_distance)
{
    mbUpdated = false;
    mdock_position_ = DOCK_POSITION::not_found;
    mcharge_status_ = CHARGE_STATUS::freed;
    sensor_status.left_sensor1 = 0;
    sensor_status.left_sensor2 = 0;
    sensor_status.right_sensor2 = 0;
    sensor_status.right_sensor1 = 0;
    sensor_status.power = 0.0;
    sensor_status.battery = 0.0;
    sensor_status.current = 0.0;
    sensor_status.remaining_battery = 0.0;
    crash_distance_ = crash_distance;

    mIRsensor1Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor1", 1, true);
    mIRsensor2Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor2", 1, true);
    mIRsensor3Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor3", 1, true);
    mIRsensor4Pub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/IRsensor4", 1, true);
    mDockpostionPub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/Dockpostion", 1, true);
    mChargestatusPub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/Chargestatus", 1, true);

    mPowerPub = mNH.advertise<std_msgs::Float32>("bw_auto_dock/Chargepower", 1, true);
    mBatteryPowerPub = mNH.advertise<std_msgs::Float64>("bw_auto_dock/Batterypower", 1, true);
    mRemainPowerPub = mNH.advertise<std_msgs::Float32>("bw_auto_dock/Remainpower", 1, true);
    mCurrentPub = mNH.advertise<std_msgs::Float32>("bw_auto_dock/Chargecurrent", 1, true);
    mCrashPub = mNH.advertise<std_msgs::Int32>("bw_auto_dock/Crashdetector", 1, true);

    mSonar1Pub = mNH.advertise<sensor_msgs::Range>("bw_auto_dock/Sonar1", 1, true);
    mSonar2Pub = mNH.advertise<sensor_msgs::Range>("bw_auto_dock/Sonar2", 1, true);

    battery_ready_ = false;
    DockSonar1.header.frame_id = "dock_sonar1";
    DockSonar1.radiation_type = 0;
    DockSonar1.field_of_view = 0.7;
    DockSonar1.min_range = 0.01;
    DockSonar1.max_range = 0.60;

    DockSonar2.header.frame_id = "dock_sonar2";
    DockSonar2.radiation_type = 0;
    DockSonar2.field_of_view = 0.7;
    DockSonar2.min_range = 0.01;
    DockSonar2.max_range = 0.60;

}

void StatusPublisher::Update_controler(const char data[], unsigned int len)
{
    int i = 0, j = 0;
    int* receive_byte;
    static unsigned char last_str[2] = { 0x00, 0x00 };
    static unsigned char new_packed_ctr = DISABLE;  // ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len = 0;               //包的理论长度
    static int new_packed_len = 0;                  //包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str = 0x00;
    const int cmd_string_max_size = 512;
    receive_byte = (int*)&sensor_status;

    for (i = 0; i < len; i++)
    {
        current_str = data[i];
        // unsigned int temp=(unsigned int)current_str;
        // std::cout<<temp<<std::endl;
        //判断是否有新包头
        if (last_str[0] == 205 && last_str[1] == 235 && current_str == 215)  //包头 205 235 215
        {
            // std::cout<<"runup1 "<<std::endl;
            new_packed_ctr = ENABLE;
            new_packed_ok_len = 0;
            new_packed_len = new_packed_ok_len;
            last_str[0] = last_str[1];  //保存最后两个字符，用来确定包头
            last_str[1] = current_str;
            continue;
        }
        last_str[0] = last_str[1];  //保存最后两个字符，用来确定包头
        last_str[1] = current_str;
        if (new_packed_ctr == ENABLE)
        {
            //获取包长度
            new_packed_ok_len = current_str;
            if (new_packed_ok_len > cmd_string_max_size)
                new_packed_ok_len = cmd_string_max_size;  //包内容最大长度有限制
            new_packed_ctr = DISABLE;
            // std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        else
        {
            //判断包当前大小
            if (new_packed_ok_len <= new_packed_len)
            {
                // std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，后续内容无效
                continue;
            }
            else
            {
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len - 1] = current_str;
                if (new_packed_ok_len == new_packed_len && new_packed_ok_len > 0)
                {
                    // std::cout<<"runup4 "<<std::endl;
                    //当前包已经处理完成，开始处理
                    boost::mutex::scoped_lock lock(mMutex_sensor);

                    if (new_packed_ok_len == 50)
                    {
                        for (j = 0; j < 10; j++)
                        {
                            memcpy(&receive_byte[j], &cmd_string_buf[5 * j], 4);
                        }
                        mbUpdated = true;
                    }
                    if (mbUpdated)
                    {
                        for (j = 0; j < 9; j++)
                        {
                            if (cmd_string_buf[5 * j + 4] != 32)
                            {
                                sensor_status.left_sensor1 = 0;
                                sensor_status.left_sensor2 = 0;
                                sensor_status.right_sensor2 = 0;
                                sensor_status.right_sensor1 = 0;
                                mbUpdated = false;
                                break;
                            }
                        }
                    }
                    // ii++;
                    // std::cout << ii << std::endl;
                    new_packed_ok_len = 0;
                    new_packed_len = 0;
                }
            }
        }
    }
    return;
}

void StatusPublisher::Update_battery(const char data[], unsigned int len)
{
    int i = 0, j = 0;
    static unsigned char last_str[2] = { 0x00, 0x00 };
    static unsigned char new_packed_ctr = DISABLE;  // ENABLE表示新包开始，DISABLE 表示上一个包还未处理完；
    static int new_packed_ok_len = 0;               //包的理论长度
    static int new_packed_len = 0;                  //包的实际长度
    static unsigned char cmd_string_buf[512];
    unsigned char current_str = 0x00;
    const int cmd_string_max_size = 512;

    for (i = 0; i < len; i++)
    {
        current_str = data[i];
        // unsigned int temp=(unsigned int)current_str;
        // std::cout<<temp<<std::endl;
        //判断是否有新包头
        if (last_str[0] == 0xdd && last_str[1] == 0x03 && current_str == 0x00)  //包头 205 235 215
        {
            // std::cout<<"runup1 "<<std::endl;
            new_packed_ctr = ENABLE;
            new_packed_ok_len = 0;
            new_packed_len = new_packed_ok_len;
            last_str[0] = last_str[1];  //保存最后两个字符，用来确定包头
            last_str[1] = current_str;
            continue;
        }
        last_str[0] = last_str[1];  //保存最后两个字符，用来确定包头
        last_str[1] = current_str;
        if (new_packed_ctr == ENABLE)
        {
            //获取包长度
            new_packed_ok_len = current_str + 3;
            if (new_packed_ok_len > cmd_string_max_size)
                new_packed_ok_len = cmd_string_max_size;  //包内容最大长度有限制
            new_packed_ctr = DISABLE;
            // std::cout<<"runup2 "<< new_packed_len<< new_packed_ok_len<<std::endl;
        }
        else
        {
            //判断包当前大小
            if (new_packed_ok_len <= new_packed_len)
            {
                // std::cout<<"runup3 "<< new_packed_len<< new_packed_ok_len<<std::endl;
                //包长度已经大于等于理论长度，后续内容无效
                continue;
            }
            else
            {
                //获取包内容
                new_packed_len++;
                cmd_string_buf[new_packed_len - 1] = current_str;
                if (new_packed_ok_len == new_packed_len && new_packed_ok_len > 0)
                {
                    // std::cout<<"runup4 "<<std::endl;
                    //当前包已经处理完成，开始处理
                    boost::mutex::scoped_lock lock(mMutex_battery);
                    if (cmd_string_buf[new_packed_ok_len-1]==0x77)
                    {

                      std::uint16_t sum=0;
                      for(i=0;i<(new_packed_ok_len-3);i++)
                      {
                        sum += cmd_string_buf[i];
                      }
                      sum += new_packed_ok_len -3;
                      sum = ~sum + 1;
                      std::uint16_t sum_check= cmd_string_buf[new_packed_ok_len-3]<<8|cmd_string_buf[new_packed_ok_len-2];
                      if(sum==sum_check)
                      {
                        std::int16_t voltage = cmd_string_buf[0]<<8|cmd_string_buf[1];
                        std::int16_t current = cmd_string_buf[2]<<8|cmd_string_buf[3];
                        std::int8_t remaining_battery = cmd_string_buf[19];
                        sensor_status.battery = voltage/100.0;
                        sensor_status.current = current/100.0;
                        sensor_status.remaining_battery = remaining_battery;
                        battery_ready_ = true;
                      }
                    }
                    // ii++;
                    // std::cout << ii << std::endl;
                    new_packed_ok_len = 0;
                    new_packed_len = 0;
                }
            }
        }
    }
    return;
}

void StatusPublisher::Refresh()
{
    boost::mutex::scoped_lock lock1(mMutex_sensor);
    boost::mutex::scoped_lock lock2(mMutex_dock);
    boost::mutex::scoped_lock lock3(mMutex_charge);
    // std::cout<<"runR"<< mbUpdated<<std::endl;
    if (mbUpdated)
    {
        // Time
        ros::Time current_time = ros::Time::now();
        std_msgs::Int32 pub_data;
        pub_data.data = sensor_status.left_sensor1;
        mIRsensor1Pub.publish(pub_data);

        pub_data.data = sensor_status.left_sensor2;
        mIRsensor2Pub.publish(pub_data);

        pub_data.data = sensor_status.right_sensor2;
        mIRsensor3Pub.publish(pub_data);

        pub_data.data = sensor_status.right_sensor1;
        mIRsensor4Pub.publish(pub_data);

        //根据4个红外接收器状态判断dock方位
        if ((sensor_status.left_sensor1 + sensor_status.left_sensor2 + sensor_status.right_sensor2 +
             sensor_status.right_sensor1) == 0)
        {
            //没有发现
            mdock_position_ = DOCK_POSITION::not_found;
        }
        else
        {
            if ((sensor_status.left_sensor2 + sensor_status.right_sensor2) == 0)
            {
                //后面的传感器没有发现
                if (sensor_status.left_sensor1 == 0)
                {
                    //左边的传感器没有发现
                    switch (sensor_status.right_sensor1)
                    {
                        case 1:
                            mdock_position_ = DOCK_POSITION::right_up;
                            break;
                        case 2:
                            mdock_position_ = DOCK_POSITION::right_down;
                            break;
                        case 3:
                            mdock_position_ = DOCK_POSITION::right_center;
                            break;
                        case 4:
                            mdock_position_ = DOCK_POSITION::right;
                            break;
                        case 5:
                            mdock_position_ = DOCK_POSITION::right_up;
                            break;
                        case 6:
                            mdock_position_ = DOCK_POSITION::right_down;
                            break;
                        case 7:
                            mdock_position_ = DOCK_POSITION::right_center;
                            break;
                    }
                }
                else
                {
                    //右边的传感器没有发现
                    switch (sensor_status.left_sensor1)
                    {
                        case 1:
                            mdock_position_ = DOCK_POSITION::left_down;
                            break;
                        case 2:
                            mdock_position_ = DOCK_POSITION::left_up;
                            break;
                        case 3:
                            mdock_position_ = DOCK_POSITION::left_center;
                            break;
                        case 4:
                            mdock_position_ = DOCK_POSITION::left;
                            break;
                        case 5:
                            mdock_position_ = DOCK_POSITION::left_down;
                            break;
                        case 6:
                            mdock_position_ = DOCK_POSITION::left_up;
                            break;
                        case 7:
                            mdock_position_ = DOCK_POSITION::left_center;
                            break;
                    }
                }
            }
            else
            {
                if (sensor_status.left_sensor2 == 0)
                {
                    //后面的左边探测器没有探测
                    switch (sensor_status.right_sensor2)
                    {
                        case 1:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 2:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 3:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 4:
                            mdock_position_ = DOCK_POSITION::back;
                            break;
                        case 5:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 6:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                        case 7:
                            mdock_position_ = DOCK_POSITION::back_right;
                            break;
                    }
                }
                else
                {
                    //后面的右边探测器没有探测
                    switch (sensor_status.left_sensor2)
                    {
                        case 1:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 2:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0 &&
                                sensor_status.right_sensor2 != 2 && sensor_status.right_sensor2 != 6)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 3:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 4:
                            if (sensor_status.right_sensor2 != 0 && sensor_status.right_sensor2 != 4)
                            {
                                mdock_position_ = DOCK_POSITION::back_right;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back;
                            }
                            break;
                        case 5:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 6:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0 &&
                                sensor_status.right_sensor2 != 2 && sensor_status.right_sensor2 != 6)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                        case 7:
                            if (sensor_status.right_sensor2 != 4 && sensor_status.right_sensor2 != 0)
                            {
                                mdock_position_ = DOCK_POSITION::back_center;
                            }
                            else
                            {
                                mdock_position_ = DOCK_POSITION::back_left;
                            }
                            break;
                    }
                }
            }
        }
        if (sensor_status.left_sensor1 == 3||sensor_status.left_sensor1 == 7)
        {
          //ROS_ERROR(" dock : %d, %d %d",(int)mdock_position_,sensor_status.left_sensor1,sensor_status.left_sensor2);
          mdock_position_ = DOCK_POSITION::left_center;
        }
        if (sensor_status.right_sensor1 == 3||sensor_status.right_sensor1 == 7)
        {
          //ROS_ERROR(" dock : %d, %d %d",(int)mdock_position_,sensor_status.left_sensor1,sensor_status.left_sensor2);
          mdock_position_ = DOCK_POSITION::right_center;
        }
        //发布dock位置
        pub_data.data = (int)mdock_position_;
        mDockpostionPub.publish(pub_data);
        //发布充电任务状态
        pub_data.data = (int)mcharge_status_;
        mChargestatusPub.publish(pub_data);

        if (sensor_status.distance1 <= this->crash_distance_&&sensor_status.distance1>0.1)
        {
            pub_data.data = 1;
        }
        else
        {
            pub_data.data = 0;
        }
        ROS_DEBUG("distance: %f %f ,current: %f",sensor_status.distance1,sensor_status.distance2,sensor_status.current);
        mCrashPub.publish(pub_data);

        std_msgs::Float32 pub_data2;
        pub_data2.data = sensor_status.power;
        mPowerPub.publish(pub_data2);

        std_msgs::Float64 pub_data3;
        pub_data3.data = sensor_status.battery;
        mBatteryPowerPub.publish(pub_data3);

        pub_data2.data = sensor_status.current;
        mCurrentPub.publish(pub_data2);

        pub_data2.data = sensor_status.remaining_battery;
        mRemainPowerPub.publish(pub_data2);
        mbUpdated = false;

        static int num_pub=0;
        num_pub++;
        if(num_pub%3==0)
        {
             DockSonar1.header.stamp = current_time;
             DockSonar1.range = sensor_status.distance1/1000.0;
             mSonar1Pub.publish(DockSonar1);

             DockSonar2.header.stamp = current_time;
             DockSonar2.range = sensor_status.distance2/1000.0;
             mSonar2Pub.publish(DockSonar2);
        }
    }
}

DOCK_POSITION StatusPublisher::get_dock_position()
{
    boost::mutex::scoped_lock lock(mMutex_dock);
    return mdock_position_;
}
void StatusPublisher::set_charge_status(CHARGE_STATUS charge_status)
{
    boost::mutex::scoped_lock lock(mMutex_charge);
    mcharge_status_ = charge_status;
}

CHARGE_STATUS StatusPublisher::get_charge_status()
{
    boost::mutex::scoped_lock lock(mMutex_charge);
    return mcharge_status_;
}
}  // namespace bw_auto_dock
