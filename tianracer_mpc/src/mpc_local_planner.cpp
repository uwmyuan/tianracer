#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace tianracer_mpc {

class MPCLocalPlannerROS : public nav_core::BaseLocalPlanner {
public:
  MPCLocalPlannerROS() : initialized_(false) {}

  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if(!initialized_) {
      ros::NodeHandle private_nh("~/" + name);
      
      // 发布全局路径给Python MPC控制器
      path_pub_ = private_nh.advertise<nav_msgs::Path>("path", 1);
      goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
      
      // 订阅Python MPC控制器的输出
      cmd_sub_ = private_nh.subscribe("ackermann_cmd_stamped", 1, &MPCLocalPlannerROS::cmdCallback, this);
      
      initialized_ = true;
      ROS_INFO("MPC本地规划器已初始化");
    }
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if(!initialized_) {
      ROS_ERROR("MPC本地规划器尚未初始化");
      return false;
    }
    
    // 将全局路径发送给Python MPC控制器
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    path.poses = plan;
    path_pub_.publish(path);
    
    // 发送目标点
    if(!plan.empty()) {
      goal_pub_.publish(plan.back());
    }
    
    return true;
  }

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if(!initialized_) {
      ROS_ERROR("MPC本地规划器尚未初始化");
      return false;
    }
    
    // 使用最近接收到的控制命令
    if(has_new_cmd_) {
      cmd_vel.linear.x = latest_cmd_.linear.x;
      cmd_vel.angular.z = latest_cmd_.angular.z;
      has_new_cmd_ = false;
      return true;
    }
    
    return false;
  }

  bool isGoalReached() {
    // 简单实现，可以根据需要扩展
    return false;
  }

private:
  void cmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    // 将阿克曼控制命令转换为Twist消息
    latest_cmd_.linear.x = msg->drive.speed;
    latest_cmd_.angular.z = msg->drive.speed * tan(msg->drive.steering_angle) / 0.26; // 使用轴距
    has_new_cmd_ = true;
  }

  bool initialized_;
  ros::Publisher path_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber cmd_sub_;
  geometry_msgs::Twist latest_cmd_;
  bool has_new_cmd_ = false;
};

}  // namespace tianracer_mpc

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tianracer_mpc::MPCLocalPlannerROS, nav_core::BaseLocalPlanner)