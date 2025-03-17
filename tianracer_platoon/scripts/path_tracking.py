#!/usr/bin/env python3
# author: @tianbot 
# repo: tianbot/abc_swarm
# license: https://github.com/tianbot/abc_swarm/blob/main/LICENSE
from importlib.resources import path
import math
import copy
import numpy as np
import tf
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped


class PathTracking(): 
    def path_cb(self, path):
        self.updated_path = path
        rospy.loginfo("接收到新路径，包含 %d 个点", len(path.poses))

    def pure_pursuit(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.path = copy.deepcopy(self.updated_path)
            self.d = []

            try:
                # 使用 rospy.Time(0) 获取最新的变换，避免等待时钟同步
                (trans, rot) = self.listener.lookupTransform("world", self.robot_name+'/odom', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("TF异常: %s", str(e))
                rate.sleep()
                continue
            rospy.loginfo_once('TF is ready')

            current_x = trans[0]
            current_y = trans[1]
            current_theta = tf.transformations.euler_from_quaternion(rot)[2]

            # 如果路径为空，则跳过
            if len(self.path.poses) == 0:
                rate.sleep()
                continue

            # find nearest points
            for i in range(len(self.path.poses)):
                dx = current_x - self.path.poses[i].pose.position.x
                dy = current_y - self.path.poses[i].pose.position.y
                self.d.append(np.hypot(dx, dy))
            
            if len(self.d):
                ind = np.argmin(self.d)
                lf_distance = 0
                while self.Lf > lf_distance and ind < len(self.path.poses) - 2:
                    delta_x = self.path.poses[ind+1].pose.position.x - self.path.poses[ind].pose.position.x
                    delta_y = self.path.poses[ind+1].pose.position.y - self.path.poses[ind].pose.position.y 
                    lf_distance = lf_distance + np.hypot(delta_x, delta_y)
                    if ind >= len(self.path.poses) - 2:
                        break
                    ind = ind + 1
            
                target_x = self.path.poses[ind].pose.position.x
                target_y = self.path.poses[ind].pose.position.y
                
                # Calculate twist
                alpha = math.atan2(target_y - current_y, target_x - current_x) - current_theta
                alpha = np.mod(alpha + math.pi, 2*math.pi) - math.pi

                twist = Twist()
                angular_z = 0.5 * alpha
                
                # publish twist
                if lf_distance > self.Lf / 2:
                    twist.linear.x = 0.2
                    if angular_z > 1.0:
                        angular_z = 1.0
                    elif angular_z < -1.0:
                        angular_z = -1.0
                    twist.angular.z = angular_z
                elif lf_distance < self.Lf / 4:
                    twist.linear.x = 0
                    twist.angular.z = 0

                # 发布速度命令并记录日志
                # self.cmd_vel_pub.publish(twist)
                # rospy.loginfo("发布速度命令: linear.x=%.2f, angular.z=%.2f", twist.linear.x, twist.angular.z)

                # publish ackermann
                if abs(twist.linear.x) > 0.001:  # 避免除以零
                    WB = 0.26
                    v_leader = twist.linear.x
                    steer = np.math.atan(angular_z * WB / v_leader)
                    ack = AckermannDriveStamped()
                    ack.header.stamp = rospy.Time.now()
                    ack.drive.speed = float(v_leader)
                    ack.drive.steering_angle = float(steer)
                    self.cmd_publisher.publish(ack)
                    rospy.loginfo("发布Ackermann命令: speed=%.2f, steering_angle=%.2f", v_leader, steer)
            
            rate.sleep()

    def __init__(self):  
        rospy.init_node('path_tracking', anonymous=False)  
        # member var
        self.path = Path()
        self.updated_path = Path()
        self.d = []
        self.Lf = 0.3
        # param
        self.robot_name = rospy.get_param('~robot_name', 'tianracer_01')
        self.plan_topic_name = rospy.get_param('~plan_topic', 'global_plan')

        rospy.loginfo("路径跟踪节点初始化，机器人名称: %s, 路径话题: %s", self.robot_name, self.plan_topic_name)

        # subs and pubs
        self.path_sub = rospy.Subscriber(self.plan_topic_name, Path, self.path_cb)
        
        # 使用绝对话题名称确保发布到正确的位置
        # self.cmd_vel_pub = rospy.Publisher('/' + self.robot_name + '/cmd_vel', Twist, queue_size=1)
        
        # 创建发布器和订阅器
        self.cmd_publisher = rospy.Publisher(
            '/' + self.robot_name + '/ackermann_cmd_stamped', AckermannDriveStamped, queue_size=1
        )  
        self.listener = tf.TransformListener()
        
        rospy.loginfo("路径跟踪节点初始化完成")

if __name__ == '__main__':  
    try:  
        tracker = PathTracking() 
        tracker.pure_pursuit()
    except rospy.ROSInterruptException:  
        rospy.loginfo("Path Tracking finished.")