#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include <typeinfo>
#include <ros/time.h>
#include <iostream>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO

#include <ros/ros.h>

#define WHEEL_RADIUS                     0.033           // meter
#define WHEEL_SEPARATION                 0.287           // meter (BURGER : 0.160, WAFFLE : 0.287)
#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
#define LEFT                        0
#define RIGHT                       1

char odom_header_frame_id[30];
char odom_child_frame_id[30];
bool init_encoder = true;
int32_t last_diff_tick[2];
double last_rad[2];
double last_velocity[2];
double odom_pose[3]={0,0,0};
double robot_pose[2];
tf::Quaternion q_t;
geometry_msgs::Quaternion q_t2;
double odom_vel[3];
double last_theta = 0.0;
int32_t last_tick[2] = {0, 0};
nav_msgs::Odometry odom;
bool tf_count = true;

void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  
  if (init_encoder)
  {
    for (int index = 0; index < 2; index++)
    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta, roll, pitch;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = -1*TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  //theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  //orientation = sensors.getOrientation();
  theta       = -1*atan2f(q_t2.x*q_t2.y + q_t2.w*q_t2.z, 
                0.5f - q_t2.y*q_t2.y - q_t2.z*q_t2.z);

  //theta       = atan2f(2.0f*(q_t2.x*q_t2.z + q_t2.y*q_t2.z), ( -q_t2.x*q_t2.x-q_t2.y*q_t2.y + q_t2.z*q_t2.z + q_t2.w*q_t2.w))*180/3.141592;

  //delta_theta = theta;
  // tf::Quaternion quater(q_t2.x,q_t2.y,q_t2.z,q_t2.w);
  // tf::Matrix3x3 m(quater);

  // m.getRPY(roll,pitch,theta);
  delta_theta = theta - last_theta;
  // if(delta_theta > 10*3.141592/180 && delta_theta > 90*3.141592/180){
  //   delta_theta = 0;  
  // }
  //std::cout <<"dth : "<< delta_theta/*180/3.141592*/<<std::endl<<"theta : "<< theta<<std::endl<<"last theta : "<<last_theta<<std::endl;
  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

void state_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg){
  // std::cout<<"left :"<<msg->dynamixel_state[0].present_position<<std::endl;
  // std::cout<<"right :"<<msg->dynamixel_state[1].present_position<<std::endl;

  updateMotorInfo(msg->dynamixel_state[0].present_position,msg->dynamixel_state[1].present_position);
  calcOdometry(0.01);
  // std::cout<<"x :"<<odom_pose[0]<<std::endl;
  // std::cout<<"y :"<<odom_pose[1]<<std::endl;
  // std::cout<<"theta :"<<odom_pose[2]*180/3.141592<<std::endl;
}
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  q_t2.x = msg->orientation.x;
  q_t2.y = msg->orientation.y;
  q_t2.z = msg->orientation.z;
  q_t2.w = msg->orientation.w;
}
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  // robot_pose[0]=msg->pose.pose.position.x;
  // robot_pose[1]=msg->pose.pose.position.y;
  // q_t2 = msg->pose.pose.orientation;
  // tf::Quaternion q(
  //   msg->pose.pose.orientation.x,
  //   msg->pose.pose.orientation.y,
  //   msg->pose.pose.orientation.z,
  //   msg->pose.pose.orientation.w);
  // tf::Matrix3x3 m(q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);

  // std::cout<<"amcl : "<<yaw*180/3.141592<<std::endl;
  // robot_pose[2] = yaw;
  // tf::quaternionMsgToTF(msg->pose.pose.orientation,q_t);
  //robot_pose[2]=(double)q_t.getAngle;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "deadreconing");
  ros::NodeHandle nh("");
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformBroadcaster tf_br_;
  //tf::StampedTransform tf_map_to_odom_;
  geometry_msgs::TransformStamped odom_trans;

  //tf_map_to_odom_.frame_id_ = std::string("map");
  //tf_map_to_odom_.child_frame_id_ = std::string("odom");
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",100);
  ros::Subscriber dynamixel_state_sub = nh.subscribe<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_workbench/dynamixel_state", 100, state_callback);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 100, pose_callback);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("imu", 100, imu_callback);

  ros::Rate loop_rate(10);

  while(ros::ok())
  { 
    ros::Time current_now = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_pose[2]);
    //geometry_msgs::Quaternion odom_quat = q_t2;
    
    odom_trans.header.stamp = current_now;

    odom_trans.transform.translation.x = odom_pose[0];
    odom_trans.transform.translation.y = odom_pose[1];
    //odom_trans.transform.translation.x = robot_pose[0];
    //odom_trans.transform.translation.y = robot_pose[1];
    odom_trans.transform.translation.z = 0.0;
    // odom_trans.transform.rotation.x = 0.0;
    // odom_trans.transform.rotation.y = 0.0;
    // odom_trans.transform.rotation.z = 0.0;
    // odom_trans.transform.rotation.w = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //tf_map_to_odom_.stamp_ = ros::Time::now();
    //std::cout<<"amcl : x"<<odom_pose[0]<<"  y "<<odom_pose[1]<<"  theta "<<odom_pose[2]<<std::endl;

    // tf_map_to_odom_.setOrigin(tf::Vector3(robot_pose[0], robot_pose[1], 0));
    // tf_map_to_odom_.setRotation(tf::Quaternion(q_t.getX(), q_t.getY(), q_t.getZ(), q_t.getW()));
    // if(tf_count == true){
    //   tf_br_.sendTransform(tf_map_to_odom_);
    //   tf_count = false;
    // }
    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_now;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation.x = q_t2.x;
    // odom.pose.pose.orientation.y = q_t2.y;
    // odom.pose.pose.orientation.z = q_t2.z;
    // odom.pose.pose.orientation.w = q_t2.w;
    odom.pose.pose.orientation = odom_quat;

    //std::cout<<"amcl : x"<<odom_pose[0]<<"  y "<<odom_pose[1]<<"  theta "<<odom_pose[2]*180/3.141592<<std::endl;

    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = odom_vel[0];
    odom.twist.twist.linear.y = odom_vel[1];
    odom.twist.twist.angular.z = odom_vel[2];

    odom_pub.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}