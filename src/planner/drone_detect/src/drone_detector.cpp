#include "drone_detect/drone_detector.h"

// STD
#include <string>

namespace detect
{

  DroneDetector::DroneDetector(const std::string &node_name)
      : Node(node_name)
  {
    readParameters();

    // 创建订阅器
    my_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry", 100, std::bind(&DroneDetector::rcvMyOdomCallback, this, std::placeholders::_1));

    depth_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "depth", 50, std::bind(&DroneDetector::rcvDepthImgCallback, this, std::placeholders::_1));

    droneX_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/others_odom", 100, std::bind(&DroneDetector::rcvDroneXOdomCallback, this, std::placeholders::_1));

    // 创建发布器
    new_depth_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("new_depth_image", 50);
    debug_depth_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_depth_image", 50);
    debug_info_pub_ = this->create_publisher<std_msgs::msg::String>("/debug_info", 50);

    // 初始化变换矩阵
    cam2body_ << 0.0, 0.0, 1.0, 0.0,
        -1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    // 初始化其他无人机的位姿误差发布器
    for (int i = 0; i < max_drone_num_; i++)
    {
      if (i != my_id_)
      {
        drone_pose_err_pub_[i] = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "drone" + std::to_string(i) + "to" + std::to_string(my_id_) + "_pose_err", 50);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully launched node.");
  }

  DroneDetector::~DroneDetector()
  {
  }

  void DroneDetector::readParameters()
  {
    // 声明参数并获取
    this->declare_parameter("cam_width", 640);
    this->declare_parameter("cam_height", 480);
    this->declare_parameter("cam_fx", 525.0);
    this->declare_parameter("cam_fy", 525.0);
    this->declare_parameter("cam_cx", 320.0);
    this->declare_parameter("cam_cy", 240.0);

    this->declare_parameter("debug_flag", false);
    this->declare_parameter("pixel_ratio", 0.5);
    this->declare_parameter("my_id", 0);
    this->declare_parameter("estimate/drone_width", 1.0);
    this->declare_parameter("estimate/drone_height", 0.5);
    this->declare_parameter("estimate/max_pose_error", 0.1);

    // 获取参数值
    this->get_parameter("cam_width", img_width_);
    this->get_parameter("cam_height", img_height_);
    this->get_parameter("cam_fx", fx_);
    this->get_parameter("cam_fy", fy_);
    this->get_parameter("cam_cx", cx_);
    this->get_parameter("cam_cy", cy_);

    this->get_parameter("debug_flag", debug_flag_);
    this->get_parameter("pixel_ratio", pixel_ratio_);
    this->get_parameter("my_id", my_id_);
    this->get_parameter("estimate/drone_width", drone_width_);
    this->get_parameter("estimate/drone_height", drone_height_);
    this->get_parameter("estimate/max_pose_error", max_pose_error_);

    max_pose_error2_ = max_pose_error_ * max_pose_error_;
  }

  // inline functions
  inline double DroneDetector::getDist2(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
  {
    double delta_x = p1(0) - p2(0);
    double delta_y = p1(1) - p2(1);
    double delta_z = p1(2) - p2(2);
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
  }

  inline double DroneDetector::getDist2(const Eigen::Vector4d &p1, const Eigen::Vector4d &p2)
  {
    double delta_x = p1(0) - p2(0);
    double delta_y = p1(1) - p2(1);
    double delta_z = p1(2) - p2(2);
    return delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
  }

  inline Eigen::Vector4d DroneDetector::depth2Pos(int u, int v, float depth)
  {
    Eigen::Vector4d pose_in_camera;
    pose_in_camera(0) = (u - cx_) * depth / fx_;
    pose_in_camera(1) = (v - cy_) * depth / fy_;
    pose_in_camera(2) = depth;
    pose_in_camera(3) = 1.0;
    return pose_in_camera;
  }

  inline Eigen::Vector4d DroneDetector::depth2Pos(const Eigen::Vector2i &pixel, float depth)
  {
    Eigen::Vector4d pose_in_camera;
    pose_in_camera(0) = (pixel(0) - cx_) * depth / fx_;
    pose_in_camera(1) = (pixel(1) - cy_) * depth / fy_;
    pose_in_camera(2) = depth;
    pose_in_camera(3) = 1.0;
    return pose_in_camera;
  }

  inline Eigen::Vector2i DroneDetector::pos2Depth(const Eigen::Vector4d &pose_in_camera)
  {
    float depth = pose_in_camera(2);
    Eigen::Vector2i pixel;
    pixel(0) = pose_in_camera(0) * fx_ / depth + cx_ + 0.5;
    pixel(1) = pose_in_camera(1) * fy_ / depth + cy_ + 0.5;
    return pixel;
  }

  inline bool DroneDetector::isInSensorRange(const Eigen::Vector2i &pixel)
  {
    if (pixel(0) >= 0 && pixel(1) >= 0 && pixel(0) <= img_width_ && pixel(1) <= img_height_)
      return true;
    else
      return false;
  }

  void DroneDetector::rcvMyOdomCallback(const nav_msgs::msg::Odometry &odom)
  {
    my_odom_ = odom;
    Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();

    my_pose_world_(0) = odom.pose.pose.position.x;
    my_pose_world_(1) = odom.pose.pose.position.y;
    my_pose_world_(2) = odom.pose.pose.position.z;
    my_pose_world_(3) = 1.0;
    my_attitude_world_.x() = odom.pose.pose.orientation.x;
    my_attitude_world_.y() = odom.pose.pose.orientation.y;
    my_attitude_world_.z() = odom.pose.pose.orientation.z;
    my_attitude_world_.w() = odom.pose.pose.orientation.w;
    body2world.block<3, 3>(0, 0) = my_attitude_world_.toRotationMatrix();
    body2world(0, 3) = my_pose_world_(0);
    body2world(1, 3) = my_pose_world_(1);
    body2world(2, 3) = my_pose_world_(2);

    // convert to cam pose
    cam2world_ = body2world * cam2body_;
    cam2world_quat_ = cam2world_.block<3, 3>(0, 0);

    // my_last_odom_stamp_ = odom.header.stamp;

    // my_last_pose_world_(0) = odom.pose.pose.position.x;
    // my_last_pose_world_(1) = odom.pose.pose.position.y;
    // my_last_pose_world_(2) = odom.pose.pose.position.z;
    // my_last_pose_world_(3) = 1.0;

    // publish tf
    //  static tf::TransformBroadcaster br;
    //  tf::Transform transform;
    //  transform.setOrigin( tf::Vector3(cam2world(0,3), cam2world(1,3), cam2world(2,3) ));
    //  transform.setRotation(tf::Quaternion(cam2world_quat.x(), cam2world_quat.y(), cam2world_quat.z(), cam2world_quat.w()));
    //  br.sendTransform(tf::StampedTransform(transform, my_last_odom_stamp, "world", "camera"));
    // publish transform from world frame to quadrotor frame.
  }

  void DroneDetector::rcvDepthImgCallback(const sensor_msgs::msg::Image::ConstPtr &depth_img)
  {
    /* 获取深度图像 */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(depth_img, depth_img->encoding);
    cv_ptr->image.copyTo(depth_img_);

    debug_start_time_ = this->get_clock()->now();

    Eigen::Vector2i true_pixel[max_drone_num_];
    for (int i = 0; i < max_drone_num_; i++)
    {
      if (in_depth_[i])
      {
        detect(i, true_pixel[i]);
      }
    }

    cv_bridge::CvImage out_msg;
    for (int i = 0; i < max_drone_num_; i++)
    {
      if (in_depth_[i])
      {
        // 擦除深度图像中的像素点
        for (int k = 0; k < int(hit_pixels_[i].size()); k++)
        {
          uint16_t *row_ptr;
          row_ptr = depth_img_.ptr<uint16_t>(hit_pixels_[i][k](1));
          (*(row_ptr + hit_pixels_[i][k](0))) = 0.0;
        }
      }
    }

    debug_end_time_ = this->get_clock()->now();
    // RCLCPP_WARN(this->get_logger(), "cost_total_time = %lf ms",
    //             (debug_end_time_ - debug_start_time_).seconds() * 1000.0);

    out_msg.header = depth_img->header;
    out_msg.encoding = depth_img->encoding;
    out_msg.image = depth_img_.clone();
    new_depth_img_pub_->publish(*out_msg.toImageMsg());

    std_msgs::msg::String msg;
    std::stringstream ss;
    if (debug_flag_)
    {
      for (int i = 0; i < max_drone_num_; i++)
      {
        if (in_depth_[i])
        {
          // 在调试图像中添加边界框
          cv::rectangle(depth_img_, cv::Rect(searchbox_lu_[i], searchbox_rd_[i]), cv::Scalar(0, 0, 0), 5, cv::LINE_8, 0);
          cv::rectangle(depth_img_, cv::Rect(boundingbox_lu_[i], boundingbox_rd_[i]), cv::Scalar(0, 0, 0), 5, cv::LINE_8, 0);
          if (debug_detect_result_[i] == 1)
          {
            ss << "no enough " << hit_pixels_[i].size();
          }
          else if (debug_detect_result_[i] == 2)
          {
            ss << "success";
          }
        }
        else
        {
          ss << "no detect";
        }
      }

      out_msg.header = depth_img->header;
      out_msg.encoding = depth_img->encoding;
      out_msg.image = depth_img_.clone();
      debug_depth_img_pub_->publish(*out_msg.toImageMsg());

      msg.data = ss.str();
      debug_info_pub_->publish(msg);
    }
  }

  void DroneDetector::rcvDroneOdomCallbackBase(const nav_msgs::msg::Odometry &odom, int drone_id)
  {
    if (drone_id == my_id_)
    {
      return;
    }
    Eigen::Matrix4d drone2world = Eigen::Matrix4d::Identity();
    drone_pose_world_[drone_id](0) = odom.pose.pose.position.x;
    drone_pose_world_[drone_id](1) = odom.pose.pose.position.y;
    drone_pose_world_[drone_id](2) = odom.pose.pose.position.z;
    drone_pose_world_[drone_id](3) = 1.0;

    drone_attitude_world_[drone_id].x() = odom.pose.pose.orientation.x;
    drone_attitude_world_[drone_id].y() = odom.pose.pose.orientation.y;
    drone_attitude_world_[drone_id].z() = odom.pose.pose.orientation.z;
    drone_attitude_world_[drone_id].w() = odom.pose.pose.orientation.w;
    drone2world.block<3, 3>(0, 0) = drone_attitude_world_[drone_id].toRotationMatrix();

    drone2world(0, 3) = drone_pose_world_[drone_id](0);
    drone2world(1, 3) = drone_pose_world_[drone_id](1);
    drone2world(2, 3) = drone_pose_world_[drone_id](2);

    drone_pose_cam_[drone_id] = cam2world_.inverse() * drone_pose_world_[drone_id];
    // if the drone is in sensor range
    drone_ref_pixel_[drone_id] = pos2Depth(drone_pose_cam_[drone_id]);
    if (drone_pose_cam_[drone_id](2) > 0 && isInSensorRange(drone_ref_pixel_[drone_id]))
    {
      in_depth_[drone_id] = true;
    }
    else
    {
      in_depth_[drone_id] = false;
      debug_detect_result_[drone_id] = 0;
    }
  }

  void DroneDetector::rcvDrone0OdomCallback(const nav_msgs::msg::Odometry &odom)
  {
    rcvDroneOdomCallbackBase(odom, 0);
  }

  void DroneDetector::rcvDrone1OdomCallback(const nav_msgs::msg::Odometry &odom)
  {
    rcvDroneOdomCallbackBase(odom, 1);
  }

  void DroneDetector::rcvDrone2OdomCallback(const nav_msgs::msg::Odometry &odom)
  {
    rcvDroneOdomCallbackBase(odom, 2);
  }

  void DroneDetector::rcvDroneXOdomCallback(const nav_msgs::msg::Odometry &odom)
  {
    std::string numstr = odom.child_frame_id.substr(6);
    try
    {
      int drone_id = std::stoi(numstr);
      rcvDroneOdomCallbackBase(odom, drone_id);
    }
    catch (const std::exception &e)
    {
      std::cout << e.what() << '\n';
    }
  }

  bool DroneDetector::countPixel(int drone_id, Eigen::Vector2i &true_pixel, Eigen::Vector4d &true_pose_cam)
  {
    boundingbox_lu_[drone_id].x = img_width_;
    boundingbox_rd_[drone_id].x = 0;
    boundingbox_lu_[drone_id].y = img_height_;
    boundingbox_rd_[drone_id].y = 0;

    valid_pixel_cnt_[drone_id] = 0;
    hit_pixels_[drone_id].clear();

    Eigen::Vector2i tmp_pixel;
    Eigen::Vector4d tmp_pose_cam;
    int search_radius = 2 * max_pose_error_ * fx_ / drone_pose_cam_[drone_id](2);
    float depth;
    searchbox_lu_[drone_id].x = drone_ref_pixel_[drone_id](0) - search_radius;
    searchbox_lu_[drone_id].y = drone_ref_pixel_[drone_id](1) - search_radius;
    searchbox_rd_[drone_id].x = drone_ref_pixel_[drone_id](0) + search_radius;
    searchbox_rd_[drone_id].y = drone_ref_pixel_[drone_id](1) + search_radius;
    // check the tmp_p around ref_pixel
    for (int i = -search_radius; i <= search_radius; i++)
      for (int j = -search_radius; j <= search_radius; j++)
      {
        tmp_pixel(0) = drone_ref_pixel_[drone_id](0) + j;
        tmp_pixel(1) = drone_ref_pixel_[drone_id](1) + i;
        if (tmp_pixel(0) < 0 || tmp_pixel(0) >= img_width_ || tmp_pixel(1) < 0 || tmp_pixel(1) >= img_height_)
          continue;
        // depth = depth_img_.at<float>(tmp_pixel(1), tmp_pixel(0));
        uint16_t *row_ptr;
        row_ptr = depth_img_.ptr<uint16_t>(tmp_pixel(1));
        depth = (*(row_ptr + tmp_pixel(0))) / 1000.0;
        // ROS_WARN("depth = %lf", depth);
        // get tmp_pose in cam frame
        tmp_pose_cam = depth2Pos(tmp_pixel(0), tmp_pixel(1), depth);
        double dist2 = getDist2(tmp_pose_cam, drone_pose_cam_[drone_id]);
        // ROS_WARN("dist2 = %lf", dist2);
        if (dist2 < max_pose_error2_)
        {
          valid_pixel_cnt_[drone_id]++;
          hit_pixels_[drone_id].push_back(tmp_pixel);
          boundingbox_lu_[drone_id].x = tmp_pixel(0) < boundingbox_lu_[drone_id].x ? tmp_pixel(0) : boundingbox_lu_[drone_id].x;
          boundingbox_lu_[drone_id].y = tmp_pixel(1) < boundingbox_lu_[drone_id].y ? tmp_pixel(1) : boundingbox_lu_[drone_id].y;
          boundingbox_rd_[drone_id].x = tmp_pixel(0) > boundingbox_rd_[drone_id].x ? tmp_pixel(0) : boundingbox_rd_[drone_id].x;
          boundingbox_rd_[drone_id].y = tmp_pixel(1) > boundingbox_rd_[drone_id].y ? tmp_pixel(1) : boundingbox_rd_[drone_id].y;
        }
      }
    pixel_threshold_ = (drone_width_ * fx_ / drone_pose_cam_[drone_id](2)) * (drone_height_ * fy_ / drone_pose_cam_[drone_id](2)) * pixel_ratio_;
    if (valid_pixel_cnt_[drone_id] > pixel_threshold_)
    {
      int step = 1, size = (boundingbox_rd_[drone_id].y - boundingbox_lu_[drone_id].y) < (boundingbox_rd_[drone_id].x - boundingbox_lu_[drone_id].x) ? (boundingbox_rd_[drone_id].y - boundingbox_lu_[drone_id].y) : (boundingbox_rd_[drone_id].x - boundingbox_lu_[drone_id].x);
      int init_x = (boundingbox_lu_[drone_id].x + boundingbox_rd_[drone_id].x) / 2, init_y = (boundingbox_lu_[drone_id].y + boundingbox_rd_[drone_id].y) / 2;
      int x_flag = 1, y_flag = 1;
      int x_idx = 0, y_idx = 0;
      uint16_t *row_ptr;
      row_ptr = depth_img_.ptr<uint16_t>(tmp_pixel(1));
      depth = (*(row_ptr + tmp_pixel(0))) / 1000.0;
      tmp_pose_cam = depth2Pos(init_x, init_y, depth);
      if (getDist2(tmp_pose_cam, drone_pose_cam_[drone_id]) < max_pose_error2_)
      {
        true_pixel(0) = init_x;
        true_pixel(1) = init_y;
        true_pose_cam = tmp_pose_cam;
        return true;
      }
      while (step < size)
      {
        while (x_idx < step)
        {
          init_x = init_x + x_flag;
          uint16_t *row_ptr;
          row_ptr = depth_img_.ptr<uint16_t>(tmp_pixel(1));
          depth = (*(row_ptr + tmp_pixel(0))) / 1000.0;
          tmp_pose_cam = depth2Pos(init_x, init_y, depth);
          if (getDist2(tmp_pose_cam, drone_pose_cam_[drone_id]) < max_pose_error2_)
          {
            true_pixel(0) = init_x;
            true_pixel(1) = init_y;
            true_pose_cam = tmp_pose_cam;
            return true;
          }
          x_idx++;
        }
        x_idx = 0;
        x_flag = -x_flag;
        while (y_idx < step)
        {
          init_y = init_y + y_flag;
          uint16_t *row_ptr;
          row_ptr = depth_img_.ptr<uint16_t>(tmp_pixel(1));
          depth = (*(row_ptr + tmp_pixel(0))) / 1000.0;
          tmp_pose_cam = depth2Pos(init_x, init_y, depth);
          if (getDist2(tmp_pose_cam, drone_pose_cam_[drone_id]) < max_pose_error2_)
          {
            true_pixel(0) = init_x;
            true_pixel(1) = init_y;
            true_pose_cam = tmp_pose_cam;
            return true;
          }
          y_idx++;
        }
        y_idx = 0;
        y_flag = -y_flag;
        step++;
      }
      while (x_idx < step - 1)
      {
        init_x = init_x + x_flag;
        uint16_t *row_ptr;
        row_ptr = depth_img_.ptr<uint16_t>(tmp_pixel(1));
        depth = (*(row_ptr + tmp_pixel(0))) / 1000.0;
        tmp_pose_cam = depth2Pos(init_x, init_y, depth);
        if (getDist2(tmp_pose_cam, drone_pose_cam_[drone_id]) < max_pose_error2_)
        {
          true_pixel(0) = init_x;
          true_pixel(1) = init_y;
          true_pose_cam = tmp_pose_cam;
          return true;
        }
        x_idx++;
      }
    }
    return false;
  }

  void DroneDetector::detect(int drone_id, Eigen::Vector2i &true_pixel)
  {
    Eigen::Vector4d true_pose_cam, pose_error;
    bool found = countPixel(drone_id, true_pixel, true_pose_cam);
    if (found)
    {
      // ROS_WARN("FOUND");
      pose_error = cam2world_ * true_pose_cam - drone_pose_world_[drone_id];
      debug_detect_result_[drone_id] = 2;

      geometry_msgs::msg::PoseStamped out_msg;
      out_msg.header.stamp = my_last_camera_stamp_;
      out_msg.header.frame_id = "/drone_detect";
      out_msg.pose.position.x = pose_error(0);
      out_msg.pose.position.y = pose_error(1);
      out_msg.pose.position.z = pose_error(2);
      drone_pose_err_pub_[drone_id]->publish(out_msg);
    }
    else
    {
      // ROS_WARN("NOT FOUND");
      debug_detect_result_[drone_id] = 1;
    }
  }

  void DroneDetector::test()
  {
    RCLCPP_WARN(this->get_logger(), "my_id = %d", my_id_);
  }

} /* namespace */