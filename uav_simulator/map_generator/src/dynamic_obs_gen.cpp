#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>

#include <dynamic_obs.h>

using namespace std;
// using namespace map_generator;

ros::Subscriber _odom_sub;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;
double _min_dist;

bool _map_ok = false;
bool _has_odom = false;

// add dynmaic obstacles -- add by zjc 10.04
bool is_dynamic = true;
pcl::PointCloud<pcl::PointXYZ> dynamic_cloudMap;
sensor_msgs::PointCloud2 dynamicObs_pcd;
ros::Publisher _dynamic_map_pub;
double _start_time;
double _last_time, _curr_time;
double _delta_time = 0, _total_time = 0;
double _oneway_time;
double _dynamic_obs_x;
double _dynamic_obs_y;
double _dynamic_obs_z;
double _dynamic_obs_width;
double _dynamic_obs_height;
int _dynamic_obs_vel; // m/s

vector<DynamicObs*> _pDynamic_obss;
bool _obs_flag = false;
bool _has_obs[6] = {false};
int _dynamic_obs_num;

pcl::PointXYZ _dynamic_obs_center[6];

ros::Subscriber _drone0_odom_sub;
ros::Subscriber _drone1_odom_sub;
ros::Subscriber _drone2_odom_sub;
ros::Subscriber _drone3_odom_sub;
ros::Subscriber _drone4_odom_sub;
ros::Subscriber _drone5_odom_sub;

ros::Publisher _dynamic_obs_odom_pub[6];
int _dynamic_obs_type;
int _dynamic_obs_y_range;

void DynamicObsGenerate() {  
  _curr_time = ros::Time::now().toSec();
  _total_time = _curr_time - _start_time;
  _delta_time = _curr_time - _last_time;
  _last_time = _curr_time;

  for (int i = 0; i < _dynamic_obs_num; i++) {
    _dynamic_obs_center[i].y += _delta_time*_dynamic_obs_vel;
    if (!_has_obs[i]) {
      DynamicObs* obs = new DynamicObs(_dynamic_obs_center[i], _dynamic_obs_width, _dynamic_obs_height, _resolution);
      _pDynamic_obss.push_back(obs);
      _has_obs[i] = true;
    }  
    else {
      _pDynamic_obss[i]->update(_dynamic_obs_center[i]);
    }
  }
  if (_total_time > _oneway_time) {
    _start_time += _oneway_time;
    _dynamic_obs_vel = -_dynamic_obs_vel;
  }
}

void pubOdom(int drone_id) {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "world";

  odom.pose.pose.position.x = _dynamic_obs_center[drone_id].x;
  odom.pose.pose.position.y = _dynamic_obs_center[drone_id].y;
  odom.pose.pose.position.z = _dynamic_obs_center[drone_id].z;

  odom.pose.pose.orientation.w = 1;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0; 

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = _dynamic_obs_vel;
  odom.twist.twist.linear.z = 0.0;

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  _dynamic_obs_odom_pub[drone_id].publish(odom);
}

void rcvDroneOdomCallbackBase(const nav_msgs::Odometry odom, int drone_id) {
  // ROS_WARN("drone_id = %d", drone_id);
  _dynamic_obs_center[drone_id].x = odom.pose.pose.position.x;
  _dynamic_obs_center[drone_id].y = odom.pose.pose.position.y;
  _dynamic_obs_center[drone_id].z = odom.pose.pose.position.z;

  if (!_has_obs[drone_id]) {
    DynamicObs* obs = new DynamicObs(_dynamic_obs_center[drone_id], _dynamic_obs_width, _dynamic_obs_height, _resolution);
    _pDynamic_obss.push_back(obs);
    _has_obs[drone_id] = true;
  } else {
    _pDynamic_obss[drone_id]->update(_dynamic_obs_center[drone_id]);
  }
}

void rcvDrone0OdomCallback(const nav_msgs::Odometry odom) {
  int drone_id = 0;
  rcvDroneOdomCallbackBase(odom, drone_id);
  return;
}

void rcvDrone1OdomCallback(const nav_msgs::Odometry odom) {
  int drone_id = 1;
  rcvDroneOdomCallbackBase(odom, drone_id);
  return;
}

void rcvDrone2OdomCallback(const nav_msgs::Odometry odom) {
  int drone_id = 2;
  rcvDroneOdomCallbackBase(odom, drone_id);
  return;
}

void rcvDrone3OdomCallback(const nav_msgs::Odometry odom) {
  int drone_id = 3;
  rcvDroneOdomCallbackBase(odom, drone_id);
  return;
}

void rcvDrone4OdomCallback(const nav_msgs::Odometry odom) {
  int drone_id = 4;
  rcvDroneOdomCallbackBase(odom, drone_id);
  return;
}

void rcvDrone5OdomCallback(const nav_msgs::Odometry odom) {
  int drone_id = 5;
  rcvDroneOdomCallbackBase(odom, drone_id);
  return;
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  // ROS_WARN("rcvOdometryCallbck");
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

void pubSensedPoints() {
  if(_dynamic_obs_type == 0){
    DynamicObsGenerate();
  }
  if (dynamic_cloudMap.width > 0)
    dynamic_cloudMap.clear();

  for (int k = 0; k < _pDynamic_obss.size(); k++) {
    DynamicObs* pObs = _pDynamic_obss[k];
    pubOdom(k);
    for (int i = 0; i < pObs->_cloudMap_ptr.width; i++)
      dynamic_cloudMap.points.push_back(pObs->_cloudMap_ptr.points[i]);
  }
  dynamic_cloudMap.width = dynamic_cloudMap.points.size();
  dynamic_cloudMap.height = 1;
  dynamic_cloudMap.is_dense = true;

  pcl::toROSMsg(dynamic_cloudMap, dynamicObs_pcd);
  dynamicObs_pcd.header.frame_id = "world";
  _dynamic_map_pub.publish(dynamicObs_pcd);
  return;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obs");
  ros::NodeHandle n("~");

  _dynamic_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/dynamic_obs", 1);
  
  _dynamic_obs_odom_pub[0] = n.advertise<nav_msgs::Odometry>("/test/dynamic0_odom", 1);
  _dynamic_obs_odom_pub[1] = n.advertise<nav_msgs::Odometry>("/test/dynamic1_odom", 1);
  _dynamic_obs_odom_pub[2] = n.advertise<nav_msgs::Odometry>("/test/dynamic2_odom", 1);
  _dynamic_obs_odom_pub[3] = n.advertise<nav_msgs::Odometry>("/test/dynamic3_odom", 1);
  _dynamic_obs_odom_pub[4] = n.advertise<nav_msgs::Odometry>("/test/dynamic4_odom", 1);
  _dynamic_obs_odom_pub[5] = n.advertise<nav_msgs::Odometry>("/test/dynamic5_odom", 1);

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  _drone0_odom_sub = n.subscribe("/drone_0_visual_slam/odom", 50, rcvDrone0OdomCallback);
  _drone1_odom_sub = n.subscribe("/drone_1_visual_slam/odom", 50, rcvDrone1OdomCallback);
  _drone2_odom_sub = n.subscribe("/drone_2_visual_slam/odom", 50, rcvDrone2OdomCallback);
  _drone3_odom_sub = n.subscribe("/drone_3_visual_slam/odom", 50, rcvDrone3OdomCallback);
  _drone4_odom_sub = n.subscribe("/drone_4_visual_slam/odom", 50, rcvDrone4OdomCallback);
  _drone5_odom_sub = n.subscribe("/drone_5_visual_slam/odom", 50, rcvDrone5OdomCallback);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/resolution", _resolution, 0.1);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);

  // param for dynamic obs
  n.param("dynamic/x", _dynamic_obs_x, 0.0);
  n.param("dynamic/z", _dynamic_obs_z, 0.5);
  n.param("dynamic/width", _dynamic_obs_width, 1.0);
  n.param("dynamic/height", _dynamic_obs_height, 0.2);
  n.param("dynamic/vel", _dynamic_obs_vel, 1);
  n.param("dynamic/num", _dynamic_obs_num, 1);
  n.param("dynamic/dynamic_obs_type", _dynamic_obs_type, 0);
  n.param("dynamic/y_range", _dynamic_obs_y_range, 5);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  ros::Rate loop_rate(_sense_rate);

  _start_time = ros::Time::now().toSec();
  _last_time = _start_time;

  _oneway_time = _dynamic_obs_y_range*2/_dynamic_obs_vel;

  for (int i = 0; i < _dynamic_obs_num; i++) {
    _dynamic_obs_center[i].x = _dynamic_obs_x;
    _dynamic_obs_center[i].y = -_dynamic_obs_y_range;
    _dynamic_obs_center[i].z = _dynamic_obs_z-i*1;   
  }

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }

  for(int i = 0, s = _pDynamic_obss.size(); i < s; i++) {
    DynamicObs* pObs = _pDynamic_obss.at(i);
    delete pObs;
  }
  vector<DynamicObs*>(_pDynamic_obss).swap(_pDynamic_obss);

}