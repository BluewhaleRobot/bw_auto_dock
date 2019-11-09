#!/usr/bin/env python
#encoding=utf-8


import time
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32, Float64
from geometry_msgs.msg import Twist
from bw_auto_dock.srv import Charge, ChargeRequest, ChargeResponse, StopCharge, StopChargeResponse
from tf.transformations import euler_from_quaternion
import threading


"""
订阅话题

odom (nav_msgs/Odometry) Used to get robot current position

发布话题

barDetectFlag (std_msgs/Bool) Turn off infrared obstacle avoidance system

cmd_vel (geometry_msgs/Twist) A stream of velocity commands meant for execution by mobile base

bw_auto_dock/Chargecurrent (std_msgs/Float32) Charging current

bw_auto_dock/Chargepower (std_msgs/Float32) Charging voltage

bw_auto_dock/Batterypower (std_msgs/Float32) battery voltage

bw_auto_dock/Chargestatus (std_msgs::Int32) Charging status

bw_auto_dock/Crashdetector (std_msgs::Int32) Collision detection status

bw_auto_dock/Dockposition (std_msgs::Int32) Dock position

bw_auto_dock/IRsensor1 (std_msgs::Int32) left sensor1 infrared receiver data

bw_auto_dock/IRsensor2 (std_msgs::Int32) left sensor2 infrared receiver data

bw_auto_dock/IRsensor3 (std_msgs::Int32) right sensor2 infrared receiver data

bw_auto_dock/IRsensor4 (std_msgs::Int32) right sensor1 infrared receiver data

提供服务

charge
stop_charge

"""

class FakeChargeNode(object):

    def __init__(self):
        super(FakeChargeNode, self).__init__()
        rospy.init_node("dock_driver")
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/xqserial_server/Power", Float64, self.power_cb)
        rospy.Service("/bw_auto_dock/charge", Charge, self.start_charge)
        rospy.Service("/bw_auto_dock/stop_charge", StopCharge, self.stop_charge)
        self.bar_detect_flag_pub = rospy.Publisher("/barDetectFlag", Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.charge_current_pub = rospy.Publisher("/bw_auto_dock/Chargecurrent", Float32, queue_size=1)
        self.charge_power_pub = rospy.Publisher("/bw_auto_dock/Chargepower", Float32, queue_size=1)
        self.battery_power_pub = rospy.Publisher("/bw_auto_dock/Batterypower", Float32, queue_size=1)
        self.charge_status_pub = rospy.Publisher("/bw_auto_dock/Chargestatus", Int32, queue_size=1)
        self.crash_detector_pub = rospy.Publisher("/bw_auto_dock/Crashdetector", Int32, queue_size=1)
        self.dock_position_pub = rospy.Publisher("/bw_auto_dock/Dockposition", Int32, queue_size=1)
        self.ir1_pub = rospy.Publisher("/bw_auto_dock/IRsensor1", Int32, queue_size=1)
        self.ir2_pub = rospy.Publisher("/bw_auto_dock/IRsensor2", Int32, queue_size=1)
        self.ir3_pub = rospy.Publisher("/bw_auto_dock/IRsensor3", Int32, queue_size=1)
        self.ir4_pub = rospy.Publisher("/bw_auto_dock/IRsensor4", Int32, queue_size=1)
        self.power_pub = rospy.Publisher("/set_power", Float64, queue_size=1)
        self.odom = None
        self.charging_flag = False
        self.charge_status = 0
        self.current_power = 0
        self.time_rate = rospy.Rate(50)
        threading.Thread(target=self.pub_status).start()
        
    def pub_status(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            charge_current = Float32()
            charge_current.data = 1
            self.charge_current_pub.publish(charge_current)
            charge_power = Float32()
            charge_power.data = 1
            self.charge_power_pub.publish(charge_power)
            battery_power = Float32()
            battery_power.data = self.current_power
            self.battery_power_pub.publish(battery_power)
            charge_status = Int32()
            charge_status.data = self.charge_status
            self.charge_status_pub.publish(charge_status)
            dock_pos = Int32()
            self.dock_position_pub.publish(dock_pos)
            ir_data = Int32()
            self.ir1_pub.publish(ir_data)
            self.ir2_pub.publish(ir_data)
            self.ir3_pub.publish(ir_data)
            self.ir4_pub.publish(ir_data)
            rate.sleep()

    def odom_cb(self, odom):
        self.odom = odom

    def power_cb(self, power):
        self.current_power = power.data

    def start_charge(self, charge_req):
        if not self.charging_flag:
            self.charging_flag = True
            threading.Thread(target=self.charge_task, args=(
                charge_req.x, charge_req.y, charge_req.theta
            )).start()
            return ChargeResponse(True)
        else:
            return ChargeResponse(False)

    def uni_angle(self, angle):
        # 把角度归一化到0 到 2pi
        while angle > 2 * math.pi or angle < 0:
            if angle > 2 * math.pi:
                angle -= math.pi * 2
            if angle < 0:
                angle += math.pi * 2
        return angle

    def charge_task(self, x, y, theta):
        self.charge_status = 3
        while self.charging_flag:
            current_x = self.odom.pose.pose.position.x
            current_y = self.odom.pose.pose.position.y
            current_oritation = self.odom.pose.pose.orientation
            current_pose_q = [current_oritation.x, current_oritation.y,
                            current_oritation.z, current_oritation.w]
            current_theta = euler_from_quaternion(current_pose_q)[2]
            # 最大位置误差
            min_x = 0.05
            min_y = 0.05
            delta_x = current_x - x
            delta_y = current_y - y
            target_theta = 0
            theta = self.uni_angle(theta)
            speed_cmd = Twist()
            # 角度已经瞄准
            if abs(delta_x) > min_x or abs(delta_y) > min_y:
                speed_cmd.linear.x = 0.2
                speed_cmd.angular.z = 0
                target_theta = math.pi + math.atan2(delta_y, delta_x)
            if abs(delta_x) < min_x and abs(delta_y) < min_y:
                speed_cmd.linear.x = 0
                speed_cmd.angular.z = 0
                target_theta = theta
            # 归一到 0 到 2pi
            target_theta = self.uni_angle(target_theta)
            current_theta = self.uni_angle(current_theta)
            min_theta = 10.0 / 180 * math.pi
            # 先瞄准角度        
            delta_theta = abs(target_theta - current_theta)
            if delta_theta > math.pi:
                delta_theta = math.pi * 2 - delta_theta
            if delta_theta > min_theta and target_theta - current_theta > 0:
                if target_theta - current_theta > math.pi:
                    speed_cmd.angular.z = -0.4
                else:
                    speed_cmd.angular.z = 0.4
                speed_cmd.linear.x = 0
            if delta_theta > min_theta and target_theta - current_theta < 0:
                if target_theta - current_theta > math.pi:
                    speed_cmd.angular.z = 0.4
                else:
                    speed_cmd.angular.z = -0.4
                speed_cmd.linear.x = 0
            
            self.cmd_vel_pub.publish(speed_cmd)
            # rospy.logwarn("############ " + str(delta_x) + " " + str(delta_y) + " " + str(abs(target_theta - current_theta)) + " " + \
            #     str(target_theta) + " " +  str(current_theta))
            if abs(delta_x) < min_x and abs(delta_y) < min_y and abs(target_theta - current_theta) < min_theta:
                # 已到达目标位置
                speed_cmd.linear.x = 0
                speed_cmd.angular.z = 0
                self.cmd_vel_pub.publish(speed_cmd)
                self.charge_status = 1
                break
            self.time_rate.sleep()

        if not self.charging_flag:
            speed_cmd = Twist()
            self.cmd_vel_pub.publish(speed_cmd)
            self.charge_status = 0
            return

        # 开始充电 设置电压
        while self.current_power < 12.0 and self.charging_flag:
            self.charge_status = 1
            power = Float64()
            if self.current_power == 0:
                power.data = 10
            if self.current_power > 0 and self.current_power < 12:
                # 手动设置充电, 每秒升高0.03V
                power.data = self.current_power + 0.03 / 50
            self.power_pub.publish(power)
            self.time_rate.sleep()

        if not self.charging_flag:
            speed_cmd = Twist()
            self.cmd_vel_pub.publish(speed_cmd)
            self.charge_status = 0
            return

        # 已经充满
        self.charge_status = 2
        time.sleep(5)
        # 向前移动，退出充电
        speed_cmd = Twist()
        speed_cmd.linear.x = 0.2
        self.cmd_vel_pub.publish(speed_cmd)
        time.sleep(5)
        speed_cmd.linear.x = 0
        self.cmd_vel_pub.publish(speed_cmd)
        self.charge_status = 0
        self.charging_flag = False

    def stop_charge(self, req):
        self.charging_flag = False
        return StopChargeResponse(True)


if __name__ == "__main__":
    FakeChargeNode()
    while not rospy.is_shutdown():
        time.sleep(1)