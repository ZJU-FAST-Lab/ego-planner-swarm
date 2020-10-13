#include <iostream>
#include <fstream>
#include <vector>
//include ros dep.
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
//include pcl dep
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
 #include <pcl_conversions/pcl_conversions.h>
//include opencv and eigen
#include <Eigen/Eigen>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

//#include <cloud_banchmark/cloud_banchmarkConfig.h>
#include "depth_render.cuh"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/String.h"

using namespace cv;
using namespace std;
using namespace Eigen;

int *depth_hostptr;
cv::Mat depth_mat;

//camera param
int width, height;
double fx,fy,cx,cy;

DepthRender depthrender;
ros::Publisher pub_depth;
ros::Publisher pub_color;
ros::Publisher pub_pose;
ros::Publisher pub_pcl_wolrd;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer, estimation_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

Matrix4d cam02body;
Matrix4d cam2world;
Eigen::Quaterniond cam2world_quat;
nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate; 
double _x_size, _y_size, _z_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

// render dynamic obs -- add by zjc 10.05
ros::Subscriber _dynamic_obs_sub;
ros::Subscriber _drone1_odom_sub, _drone2_odom_sub;

vector<float> _all_cloud_data;

const int MAX_DRONE_NUM = 3;

double MAX_POSE_ERROR;
double MAX_POSE_ERROR2;
double MAX_DEPTH_ERROR;
int PIXEL_THRESHOLD;

ros::Publisher _detect_drone_pub;

Eigen::Vector4d my_pose_world;
Eigen::Quaterniond my_attitude_world;
Eigen::Vector4d my_last_pose_world;
ros::Time my_last_odom_stamp = ros::TIME_MAX;

Matrix4d drone2world[MAX_DRONE_NUM];
Eigen::Vector4d drone_pose_world[MAX_DRONE_NUM];
Eigen::Quaterniond drone_attitude_world[MAX_DRONE_NUM];
Eigen::Vector4d drone_pose_cam[MAX_DRONE_NUM];
Eigen::Vector2i drone_ref_pixel[MAX_DRONE_NUM];

ros::Publisher debug_pub;
vector<Eigen::Vector2i> hit_pixels[MAX_DRONE_NUM];
int valid_pixel_cnt[MAX_DRONE_NUM];
double _dynamic_width, _dynamic_height;

bool in_depth[MAX_DRONE_NUM] = {false};
cv::Point _lu[MAX_DRONE_NUM], _rd[MAX_DRONE_NUM];
cv::Point _bbox_lu[MAX_DRONE_NUM], _bbox_rd[MAX_DRONE_NUM];

std::stringstream _img_text_ss;

void render_currentpose();
void render_pcl_world();

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index) 
{
    Eigen::Vector3d pt;
    pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
    pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
    pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

    return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt)
{
    Eigen::Vector3i idx;
    idx(0) = std::min( std::max( int( (pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
    idx(1) = std::min( std::max( int( (pt(1) - _gl_yl) * _inv_resolution), 0), _GLY_SIZE - 1);
    idx(2) = std::min( std::max( int( (pt(2) - _gl_zl) * _inv_resolution), 0), _GLZ_SIZE - 1);              
  
    return idx;
};

inline double getDist2(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
{
    double delta_x = p1(0)-p2(0);
    double delta_y = p1(1)-p2(1);
    double delta_z = p1(2)-p2(2);
    return delta_x*delta_x+delta_y*delta_y+delta_z*delta_z;
}

inline double getDist2(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2)
{
    double delta_x = p1(0)-p2(0);
    double delta_y = p1(1)-p2(1);
    double delta_z = p1(2)-p2(2);
    return delta_x*delta_x+delta_y*delta_y+delta_z*delta_z;
}

inline Eigen::Vector4d depth2Pos(int u, int v, float depth) 
{
  Eigen::Vector4d pose_in_camera;
  pose_in_camera(0) = (u - cx) * depth / fx;
  pose_in_camera(1) = (v - cy) * depth / fy;
  pose_in_camera(2) = depth; 
  pose_in_camera(3) = 1.0;
  return pose_in_camera;
}

inline Eigen::Vector4d depth2Pos(const Eigen::Vector2i &pixel, float depth) 
{
  Eigen::Vector4d pose_in_camera;
  pose_in_camera(0) = (pixel(0) - cx) * depth / fx;
  pose_in_camera(1) = (pixel(1) - cy) * depth / fy;
  pose_in_camera(2) = depth; 
  pose_in_camera(3) = 1.0;
  return pose_in_camera;
}

inline Eigen::Vector2i pos2Depth(const Eigen::Vector4d &pose_in_camera) 
{
  float depth = pose_in_camera(2);
  Eigen::Vector2i pixel;
  pixel(0) = pose_in_camera(0) * fx / depth + cx + 0.5;
  pixel(1) = pose_in_camera(1) * fy / depth + cy + 0.5;
  return pixel;
}

inline bool isInSensorRange(const Eigen::Vector2i &pixel)
{
  if (pixel(0)>=0 && pixel(1) >= 0 && pixel(0) <= width && pixel(1) <= height) return true;
  else 
    return false;
}

bool countPixel(int drone_id, const cv::Mat &depth_image, Eigen::Vector2i &true_pixel) {
  _bbox_lu[drone_id].x = width;
  _bbox_rd[drone_id].x = 0;
  _bbox_lu[drone_id].y = height;
  _bbox_rd[drone_id].y = 0;

  valid_pixel_cnt[drone_id] = 0;
  hit_pixels[drone_id].clear();

  Eigen::Vector2i tmp_pixel;
  Eigen::Vector4d tmp_pose_cam;
  int search_radius = 2*MAX_POSE_ERROR*fx/drone_pose_cam[drone_id](2);
  float depth;
  _lu[drone_id].x = drone_ref_pixel[drone_id](0) - search_radius;
  _lu[drone_id].y = drone_ref_pixel[drone_id](1) - search_radius;
  _rd[drone_id].x = drone_ref_pixel[drone_id](0) + search_radius;
  _rd[drone_id].y = drone_ref_pixel[drone_id](1) + search_radius;
  // check the tmp_p around ref_pixel
  for(int i = -search_radius; i <= search_radius; i++)
    for(int j = -search_radius; j <= search_radius; j++)
    {
      tmp_pixel(0) = drone_ref_pixel[drone_id](0) + j;
      tmp_pixel(1) = drone_ref_pixel[drone_id](1) + i;
      if(tmp_pixel(0) < 0 || tmp_pixel(0) >= width || tmp_pixel(1) < 0 || tmp_pixel(1) >= height)
        continue;
      // depth = depth_image.at<float>(tmp_pixel(0), tmp_pixel(1));
      depth = depth_image.at<float>(tmp_pixel(1), tmp_pixel(0));
      // get tmp_pose in cam frame
      tmp_pose_cam = depth2Pos(tmp_pixel(0), tmp_pixel(1), depth);
      if (getDist2(tmp_pose_cam, drone_pose_cam[drone_id]) < MAX_POSE_ERROR2) {
        valid_pixel_cnt[drone_id]++;
        hit_pixels[drone_id].push_back(tmp_pixel);
        _bbox_lu[drone_id].x = tmp_pixel(0) < _bbox_lu[drone_id].x ? tmp_pixel(0) : _bbox_lu[drone_id].x;
        _bbox_lu[drone_id].y = tmp_pixel(1) < _bbox_lu[drone_id].y ? tmp_pixel(1) : _bbox_lu[drone_id].y;
        _bbox_rd[drone_id].x = tmp_pixel(0) > 0 ? tmp_pixel(0) : _bbox_rd[drone_id].x;
        _bbox_rd[drone_id].y = tmp_pixel(1) > 0 ? tmp_pixel(1) : _bbox_rd[drone_id].y;
      }
    } 
  PIXEL_THRESHOLD = (_dynamic_width*fx/drone_pose_cam[drone_id](2)) * (_dynamic_height*fy/drone_pose_cam[drone_id](2))*0.4; 
  if (valid_pixel_cnt[drone_id] > PIXEL_THRESHOLD) {
    int step = 1, size = (_bbox_rd[drone_id].y-_bbox_lu[drone_id].y) < (_bbox_rd[drone_id].x-_bbox_lu[drone_id].x) ? (_bbox_rd[drone_id].y-_bbox_lu[drone_id].y) : (_bbox_rd[drone_id].x-_bbox_lu[drone_id].x);
    int init_x = (_bbox_lu[drone_id].x+_bbox_rd[drone_id].x)/2, init_y = (_bbox_lu[drone_id].y+_bbox_rd[drone_id].y)/2;
    int x_flag = 1, y_flag = 1;
    int x_idx = 0, y_idx = 0, cnt = 0;
    float depth = depth_image.at<float>(init_y, init_x); 
    tmp_pose_cam = depth2Pos(init_x, init_y, depth);
    if (getDist2(tmp_pose_cam, drone_pose_cam[drone_id]) < MAX_POSE_ERROR2){
      true_pixel(0) = init_x;
      true_pixel(1) = init_y;
      return true;
    }
    while(step<size) {
        while(x_idx<step){
            init_x = init_x+x_flag;
            depth = depth_image.at<float>(init_y, init_x); 
            tmp_pose_cam = depth2Pos(init_x, init_y, depth);
            if (getDist2(tmp_pose_cam, drone_pose_cam[drone_id]) < MAX_POSE_ERROR2) {
              true_pixel(0) = init_x;
              true_pixel(1) = init_y;
              return true;
            }
            x_idx++;
        }
        x_idx = 0;
        x_flag = -x_flag;
        while(y_idx<step){
            init_y = init_y+y_flag;
            depth = depth_image.at<float>(init_y, init_x); 
            tmp_pose_cam = depth2Pos(init_x, init_y, depth);
            if (getDist2(tmp_pose_cam, drone_pose_cam[drone_id]) < MAX_POSE_ERROR2){
              true_pixel(0) = init_x;
              true_pixel(1) = init_y;
              return true;
            }
            y_idx++;
        }
        y_idx = 0;
        y_flag = -y_flag;
        step++;
    }
    while(x_idx<step-1){
        init_x = init_x+x_flag;
        depth = depth_image.at<float>(init_y, init_x); 
        tmp_pose_cam = depth2Pos(init_x, init_y, depth);
        if (getDist2(tmp_pose_cam, drone_pose_cam[drone_id]) < MAX_POSE_ERROR2){
          true_pixel(0) = init_x;
          true_pixel(1) = init_y;
          return true;
        }
        x_idx++;
    }  
  }
  return false;
}

void detect(int drone_id, const cv::Mat &depth_image, Eigen::Vector2i &true_pixel)
{
  std_msgs::String msg;
  std::stringstream ss;

  bool found = countPixel(drone_id, depth_image, true_pixel); 
}

void rcvOdometryCallback(const nav_msgs::Odometry& odom)
{
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
  Matrix4d body2world = Matrix4d::Identity();

  my_pose_world(0) = odom.pose.pose.position.x;
  my_pose_world(1) = odom.pose.pose.position.y;
  my_pose_world(2) = odom.pose.pose.position.z;
  my_pose_world(3) = 1.0;
  my_attitude_world.x() = odom.pose.pose.orientation.x;
  my_attitude_world.y() = odom.pose.pose.orientation.y;
  my_attitude_world.z() = odom.pose.pose.orientation.z;
  my_attitude_world.w() = odom.pose.pose.orientation.w;
  body2world.block<3,3>(0,0) = my_attitude_world.toRotationMatrix();
  body2world(0,3) = my_pose_world(0);
  body2world(1,3) = my_pose_world(1);
  body2world(2,3) = my_pose_world(2);

  //convert to cam pose
  cam2world = body2world * cam02body;
  cam2world_quat = cam2world.block<3,3>(0,0);

  my_last_odom_stamp = odom.header.stamp;

  my_last_pose_world(0) = odom.pose.pose.position.x;
  my_last_pose_world(1) = odom.pose.pose.position.y;
  my_last_pose_world(2) = odom.pose.pose.position.z;
  my_last_pose_world(3) = 1.0;
  //publish tf
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin( tf::Vector3(cam2world(0,3), cam2world(1,3), cam2world(2,3) ));
  // transform.setRotation(tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(), cam2world_quat.w()));
  // br.sendTransform(tf::StampedTransform(transform, my_last_odom_stamp, "world", "camera")); 
  //publish transform from world frame to quadrotor frame.
}

void rcvDroneOdomCallbackBase(const nav_msgs::Odometry& odom, int drone_id)
{
  Matrix4d drone2world = Matrix4d::Identity();
  drone_pose_world[drone_id](0) = odom.pose.pose.position.x;
  drone_pose_world[drone_id](1) = odom.pose.pose.position.y;
  drone_pose_world[drone_id](2) = odom.pose.pose.position.z;
  drone_pose_world[drone_id](3) = 1.0;

  drone_attitude_world[drone_id].x() = odom.pose.pose.orientation.x;
  drone_attitude_world[drone_id].y() = odom.pose.pose.orientation.y;
  drone_attitude_world[drone_id].z() = odom.pose.pose.orientation.z;
  drone_attitude_world[drone_id].w() = odom.pose.pose.orientation.w;
  drone2world.block<3,3>(0,0) = drone_attitude_world[drone_id].toRotationMatrix();
  
  drone2world(0,3) = drone_pose_world[drone_id](0);
  drone2world(1,3) = drone_pose_world[drone_id](1);
  drone2world(2,3) = drone_pose_world[drone_id](2);

  drone_pose_cam[drone_id] = cam2world.inverse() * drone_pose_world[drone_id];
  
  // if the drone is in sensor range
  drone_ref_pixel[drone_id] = pos2Depth(drone_pose_cam[drone_id]);
  if (isInSensorRange(drone_ref_pixel[drone_id])) {
    in_depth[drone_id] = true;
  } else {
    in_depth[drone_id] = false;
  }
}

void rcvDrone1OdomCallback(const nav_msgs::Odometry& odom)
{
  rcvDroneOdomCallbackBase(odom, 1);
}

void rcvDrone2OdomCallback(const nav_msgs::Odometry& odom)
{
  rcvDroneOdomCallbackBase(odom, 2);
}

void pubCameraPose(const ros::TimerEvent & event)
{ 
  //cout<<"pub cam pose"
  geometry_msgs::PoseStamped camera_pose;
  camera_pose.header = _odom.header;
  camera_pose.header.frame_id = "/map";
  camera_pose.pose.position.x = cam2world(0,3);
  camera_pose.pose.position.y = cam2world(1,3);
  camera_pose.pose.position.z = cam2world(2,3);
  camera_pose.pose.orientation.w = cam2world_quat.w();
  camera_pose.pose.orientation.x = cam2world_quat.x();
  camera_pose.pose.orientation.y = cam2world_quat.y();
  camera_pose.pose.orientation.z = cam2world_quat.z();
  pub_pose.publish(camera_pose);
}

void renderSensedPoints(const ros::TimerEvent & event)
{ 
  //if(! has_global_map || ! has_odom) return;
  if( !has_global_map && !has_local_map) return;
  
  if( !has_odom ) return;
  render_currentpose();
  render_pcl_world();

}

vector<float> cloud_data;
void rcvGlobalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
  if(has_global_map)
    return;

  ROS_WARN("Global Pointcloud received..");
  //load global map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  //transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);
  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    cloud_data.push_back(pt_in.x);
    cloud_data.push_back(pt_in.y);
    cloud_data.push_back(pt_in.z);
  }
  printf("global map has points: %d.\n", (int)cloud_data.size() / 3 );
  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  has_global_map = true;
}

void rcvDynamicObsCallBack(const sensor_msgs::PointCloud2 & dynamic_obs_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ> dynamic_obs_cloudIn;
  // pcl::PointXYZ dynamic_obs_pt_in;
  Eigen::Vector3d dynamic_obs_pt_in;
  pcl::fromROSMsg(dynamic_obs_pointcloud, dynamic_obs_cloudIn);

  if(has_global_map) {
    _all_cloud_data.clear();
    // step 1: add static 
    for(int i = 0; i < int(cloud_data.size()); i++){
      _all_cloud_data.push_back(cloud_data[i]);
    }    
  }
  // step 2: add dynamic
  for(int i = 0; i < int(dynamic_obs_cloudIn.points.size()); i++) {
    dynamic_obs_pt_in(0) = dynamic_obs_cloudIn.points[i].x;
    dynamic_obs_pt_in(1) = dynamic_obs_cloudIn.points[i].y;
    dynamic_obs_pt_in(2) = dynamic_obs_cloudIn.points[i].z;

    if(getDist2(dynamic_obs_pt_in, my_last_pose_world.segment(0,3)) > 0.15) {
      _all_cloud_data.push_back(dynamic_obs_pt_in(0));
      _all_cloud_data.push_back(dynamic_obs_pt_in(1));
      _all_cloud_data.push_back(dynamic_obs_pt_in(2));
    }
  }
  // printf("dynamic obs has points: %d.\n", (int)dynamic_obs_cloudIn.points.size());
  depthrender.update_data(_all_cloud_data);
}

void rcvLocalPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{
  //ROS_WARN("Local Pointcloud received..");
  //load local map
  pcl::PointCloud<pcl::PointXYZ> cloudIn;
  pcl::PointXYZ pt_in;
  //transform map to point cloud format
  pcl::fromROSMsg(pointcloud_map, cloudIn);

  if(cloudIn.points.size() == 0) return;
  for(int i = 0; i < int(cloudIn.points.size()); i++){
    pt_in = cloudIn.points[i];
    Eigen::Vector3d pose_pt(pt_in.x, pt_in.y, pt_in.z);
    //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
    cloud_data.push_back(pose_pt(0));
    cloud_data.push_back(pose_pt(1));
    cloud_data.push_back(pose_pt(2));
  }
  //printf("local map has points: %d.\n", (int)cloud_data.size() / 3 );
  //pass cloud_data to depth render
  depthrender.set_data(cloud_data);
  depth_hostptr = (int*) malloc(width * height * sizeof(int));

  has_local_map = true;
}

void render_pcl_world()
{
  //for debug purpose
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ pt_in;

  Eigen::Vector4d pose_in_camera;
  Eigen::Vector4d pose_in_world;
  Eigen::Vector3d pose_pt;

  for(int u = 0; u < width; u++)
    for(int v = 0; v < height; v++){
      float depth = depth_mat.at<float>(v,u);
      
      if(depth == 0.0)
        continue;

      pose_in_camera(0) = (u - cx) * depth / fx;
      pose_in_camera(1) = (v - cy) * depth / fy;
      pose_in_camera(2) = depth; 
      pose_in_camera(3) = 1.0;
      
      pose_in_world = cam2world * pose_in_camera;

      if( (pose_in_world.segment(0,3) - my_last_pose_world.segment(0,3)).norm() > sensing_horizon )
          continue; 

      pose_pt = pose_in_world.head(3);
      //pose_pt = gridIndex2coord(coord2gridIndex(pose_pt));
      pt_in.x = pose_pt(0);
      pt_in.y = pose_pt(1);
      pt_in.z = pose_pt(2);

      localMap.points.push_back(pt_in);
    }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, local_map_pcl);
  local_map_pcl.header.frame_id  = "/map";
  local_map_pcl.header.stamp     = my_last_odom_stamp;

  pub_pcl_wolrd.publish(local_map_pcl);

}

void render_currentpose()
{
  double this_time = ros::Time::now().toSec();

  Matrix4d cam_pose = cam2world.inverse();

  double pose[4 * 4];

  for(int i = 0; i < 4; i ++)
    for(int j = 0; j < 4; j ++)
      pose[j + 4 * i] = cam_pose(i, j);

  depthrender.render_pose(pose, depth_hostptr);

  depth_mat = cv::Mat::zeros(height, width, CV_32FC1);
  double min = 0.5;
  double max = 1.0f;
  for(int i = 0; i < height; i++)
    for(int j = 0; j < width; j++)
    {
      float depth = (float)depth_hostptr[i * width + j] / 1000.0f;
      depth = depth < 500.0f ? depth : 0;
      max = depth > max ? depth : max;
      depth_mat.at<float>(i,j) = depth;
    }
  // ROS_INFO("render cost %lf ms.", (ros::Time::now().toSec() - this_time) * 1000.0f);
  // printf("max_depth %lf.\n", max);

  // Eigen::Vector2i true_pixel[MAX_DRONE_NUM];
  // for (int i = 0; i < MAX_DRONE_NUM; i++) {
  //   if (in_depth[i]) {
  //     detect(i, depth_mat, true_pixel[i]);
  //   }
  // }  

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = my_last_odom_stamp;
  out_msg.header.frame_id = "camera";
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = depth_mat.clone();
  pub_depth.publish(out_msg.toImageMsg());

  cv::Mat adjMap;
  depth_mat.convertTo(adjMap, CV_8UC1, 255 /13.0, -min);
  cv::Mat falseColorsMap;
  cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_RAINBOW);

  // for (int i = 0; i < MAX_DRONE_NUM; i++) {
  //   if (in_depth[i]) {
  //     // add bound box in colormap
  //     // cv::Rect rect(_bbox_lu.x, _bbox_lu.y, _bbox_rd.x, _bbox_rd.y);//左上坐标（x,y）和矩形的长(x)宽(y)
  //     cv::rectangle(falseColorsMap, cv::Rect(_lu[i], _rd[i]), Scalar(0, 0, 0), 5, LINE_8, 0);
  //     cv::rectangle(falseColorsMap, cv::Rect(_bbox_lu[i], _bbox_rd[i]), Scalar(255, 0, 0), 5, LINE_8, 0);

  //     // draw true and estimate pixel
  //     cv::circle(falseColorsMap, Point(true_pixel[i](0), true_pixel[i](1)), 15, Scalar(255, 0, 0), -1);
  //     cv::circle(falseColorsMap, Point(drone_ref_pixel[i](0), drone_ref_pixel[i](1)), 15, Scalar(0, 255, 0), -1);   

  //     // draw hit pixels
  //     // for(int k = 0; k < hit_pixels.size(); k++) {
  //     //   falseColorsMap.at<Vec3b>(hit_pixels[k](1), hit_pixels[k](0))[0] = 0;
  //     //   falseColorsMap.at<Vec3b>(hit_pixels[k](1), hit_pixels[k](0))[1] = 0;
  //     //   falseColorsMap.at<Vec3b>(hit_pixels[k](1), hit_pixels[k](0))[2] = 0;
  //     // } 
  //   }
  // }

  // cv::putText(falseColorsMap, _img_text_ss.str(), Point(10, 400), cv::FONT_HERSHEY_PLAIN, 2.0, Scalar(0, 0, 0), 8);
  cv_bridge::CvImage cv_image_colored;
  cv_image_colored.header.frame_id = "depthmap";
  cv_image_colored.header.stamp = my_last_odom_stamp;
  cv_image_colored.encoding = sensor_msgs::image_encodings::BGR8;
  cv_image_colored.image = falseColorsMap;
  pub_color.publish(cv_image_colored.toImageMsg());
  _img_text_ss.str("");
  //cv::imshow("depth_image", adjMap);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("cam_width", width);
  nh.getParam("cam_height", height);
  nh.getParam("cam_fx", fx);
  nh.getParam("cam_fy", fy);
  nh.getParam("cam_cx", cx);
  nh.getParam("cam_cy", cy);
  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate",    sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);

  nh.getParam("map/x_size",     _x_size);
  nh.getParam("map/y_size",     _y_size);
  nh.getParam("map/z_size",     _z_size);

  nh.param("estimate/dynamic_width", _dynamic_width, 0.5);
  nh.param("estimate/dynamic_height", _dynamic_height, 0.1);
  nh.param("estimate/max_pose_error", MAX_POSE_ERROR, 0.3);
  nh.param("estimate/max_depth_error", MAX_DEPTH_ERROR, 0.3);
  // nh.param("estimate/max_pixel_error", MAX_PIXEL_ERROR, 80);
  // nh.param("estimate/pixel_threshold", PIXEL_THRESHOLD, 50);

  MAX_POSE_ERROR2 = MAX_POSE_ERROR*MAX_POSE_ERROR;

  depthrender.set_para(fx, fy, cx, cy, width, height);

  cam02body << 0.0, 0.0, 1.0, 0.0,
              -1.0, 0.0, 0.0, 0.0,
               0.0, -1.0,0.0, 0.0,
               0.0, 0.0, 0.0, 1.0;

  //init cam2world transformation
  cam2world = Matrix4d::Identity();

  //subscribe point cloud and dynamic obj
  global_map_sub = nh.subscribe( "global_map", 1,  rcvGlobalPointCloudCallBack);  
  local_map_sub  = nh.subscribe( "local_map",  1,  rcvLocalPointCloudCallBack);  
  odom_sub       = nh.subscribe( "odometry",   50, rcvOdometryCallback);  
  _dynamic_obs_sub = nh.subscribe("dynamic_obs", 1, rcvDynamicObsCallBack);

  //publisher depth image and color image
  pub_depth = nh.advertise<sensor_msgs::Image>("depth",1000);
  pub_color = nh.advertise<sensor_msgs::Image>("colordepth",1000);
  pub_pose  = nh.advertise<geometry_msgs::PoseStamped>("camera_pose",1000);
  pub_pcl_wolrd = nh.advertise<sensor_msgs::PointCloud2>("rendered_pcl",1);

  // subscribe other drones odom, notify by id
  _drone1_odom_sub = nh.subscribe( "/test/dynamic0_odom", 50, rcvDrone1OdomCallback); 
  _drone2_odom_sub = nh.subscribe( "/test/dynamic1_odom", 50, rcvDrone2OdomCallback); 
  
  // init other drone transformation and bounding box
  for(int i = 0; i < MAX_DRONE_NUM; i++) {
  //   drone2world[i] = Matrix4d::Identity();
    _bbox_rd[i].y = 0;
    _bbox_rd[i].x = 0;
    _bbox_lu[i].x = width;
    _bbox_lu[i].y = height;
  } 
  // _bbox_rd.y = 0;
  // _bbox_rd.x = 0;
  // _bbox_lu.x = width;
  // _bbox_lu.y = height;  
  _detect_drone_pub = nh.advertise<std_msgs::String>("/detect_drone1", 1000);
  debug_pub = nh.advertise<nav_msgs::Odometry>("/debug", 1000);

  double sensing_duration  = 1.0 / sensing_rate;
  double estimate_duration = 1.0 / estimation_rate;

  local_sensing_timer = nh.createTimer(ros::Duration(sensing_duration),  renderSensedPoints);
  estimation_timer    = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size/2.0;
  _gl_yl = -_y_size/2.0;
  _gl_zl =   0.0;
  
  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  ros::Rate rate(100);
  while(ros::ok()) 
  {
    ros::spinOnce();  
    rate.sleep();
  } 
}
