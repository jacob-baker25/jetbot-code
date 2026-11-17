/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.uma.es/papersrepo/2016/2016_Jaimez_ICRA_RF2O.pdf
*
* Maintainer: Javier G. Monroy
* MAPIR group: https://mapir.isa.uma.es
*
* Modifications: Jeremie Deray & (see contributons on github)
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2DNode.hpp"
#include "rmw/rmw.h"
#include <iostream>

using namespace rf2o;

CLaserOdometry2DNode::CLaserOdometry2DNode(): Node("CLaserOdometry2DNode")
{
  RCLCPP_INFO(get_logger(), "Initializing RF2O node...");

  // Read Parameters
  //----------------
  this->declare_parameter<std::string>("laser_scan_topic", "/scan");
  this->get_parameter("laser_scan_topic", laser_scan_topic);
  this->declare_parameter<std::string>("odom_topic", "/odom_rf2o");
  this->get_parameter("odom_topic", odom_topic);
  this->declare_parameter<std::string>("base_frame_id", "base_link");
  this->get_parameter("base_frame_id", base_frame_id);
  this->declare_parameter<std::string>("odom_frame_id", "odom");
  this->get_parameter("odom_frame_id", odom_frame_id);
  this->declare_parameter<bool>("publish_tf", true);
  this->get_parameter("publish_tf", publish_tf);
  this->declare_parameter<std::string>("init_pose_from_topic", "");
  this->get_parameter("init_pose_from_topic", init_pose_from_topic);
  this->declare_parameter<double>("freq", 10.0);
  this->get_parameter("freq", freq);
  this->declare_parameter<bool>("timing_diagnostics", true);
  this->get_parameter("timing_diagnostics", timing.enabled);
  this->declare_parameter<bool>("suppress_small_motion", true);
  this->get_parameter("suppress_small_motion", rf2o_ref.suppress_small_motion);
  std::cerr << "RF2O_ESTIMATE_POLICY version=1 reference_max_s=0.35 deadband="
            << (rf2o_ref.suppress_small_motion ? "legacy" : "off") << std::endl;
  std::cerr << "RF2O_PYRAMID_POLICY version=1 skip_rank_deficient_coarse=true require_finest=true" << std::endl;
  if (timing.enabled) {
    std::cerr << "RF2O_TIMING_ENABLED version=1 rmw="
              << rmw_get_implementation_identifier() << std::endl;
  }

  // Init Publishers and Subscribers
  //---------------------------------
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
  odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  odom_pub  = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
      std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  
  // Initialize pose
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic,rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    // init to 0
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  // Init variables
  rf2o_ref.module_initialized = false;
  rf2o_ref.first_laser_scan   = true;
}


/**
 * Keeps the last scan from the 2D lidar to be latter processed
 * On the first laser scan, the node is initialized.
*/
CLaserOdometry2DNode::~CLaserOdometry2DNode()
{
  std::cerr << "RF2O_VALIDITY_SUMMARY {\"version\":1,\"accepted\":"
            << rf2o_ref.accepted_estimates << ",\"rejected\":"
            << rf2o_ref.rejected_estimates << ",\"last_reason\":\""
            << rf2o_ref.failure_reason << "\"}" << std::endl;
  // Capture enough input to investigate numerical failures without another
  // full-rate recorder or per-scan I/O. Nonfinite missing returns encode null.
  auto scan_json = [](const sensor_msgs::msg::LaserScan &scan) {
    std::cerr << "{\"sec\":" << scan.header.stamp.sec << ",\"nanosec\":"
      << scan.header.stamp.nanosec << ",\"angle_increment\":";
    if (std::isfinite(scan.angle_increment)) std::cerr << scan.angle_increment;
    else std::cerr << "null";
    std::cerr << ",\"ranges\":[";
    for (size_t i=0;i<scan.ranges.size();++i) {
      if (i) std::cerr << ',';
      if (std::isfinite(scan.ranges[i])) std::cerr << scan.ranges[i];
      else std::cerr << "null";
    }
    std::cerr << "]}";
  };
  for (const auto &failure : failure_inputs) {
    std::cerr << "RF2O_REJECT_INPUT {\"reason\":\"" << failure.reason << "\",\"reference\":";
    scan_json(failure.reference);
    std::cerr << ",\"candidate\":"; scan_json(failure.candidate);
    std::cerr << ",\"motion_prior\":[";
    for (int i=0;i<3;++i) {
      if (i) std::cerr << ',';
      if (std::isfinite(failure.prior(i))) std::cerr << failure.prior(i);
      else std::cerr << "null";
    }
    std::cerr << "]}" << std::endl;
  }
}

void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  TimingScope callback_timing(timing, TimingDiagnostics::CALLBACK);
  if (timing.enabled) {
    timing.scan(double(new_scan->header.stamp.sec)+double(new_scan->header.stamp.nanosec)*1e-9,
                this->get_clock()->now().seconds(), TimingDiagnostics::now());
  }
  if (GT_pose_initialized)
  {
    // Keep in memory the last received laser_scan
    last_scan = *new_scan;
    rf2o_ref.current_scan_time = last_scan.header.stamp;
    
    if (rf2o_ref.first_laser_scan == false)
    {
      // Geometry is validated in odometryCalculation before indexing ranges.
      new_scan_available = true;
    }
    else
    {
      // Initialize module on first scan (from laser params)
      if (!setLaserPoseFromTf()) return;
      rf2o_ref.init(last_scan, initial_robot_pose.pose.pose);
      last_valid_scan = last_scan;
      rf2o_ref.first_laser_scan = false;
    }
  }
}


/** 
   * Gets the laser pose with respect the base_link (through TF)
   * This allow estimation of the odometry with respect to the robot base reference system.
   */
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{  
  bool retrieved = false;  
  geometry_msgs::msg::TransformStamped tf_laser;

  try
  {
    tf_laser = buffer_->lookupTransform(base_frame_id, last_scan.header.frame_id, tf2::TimePointZero);
    retrieved = true;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(get_logger(), "%s",ex.what());
    return false;
  }

  // Keep this transform as Eigen Matrix3d
  tf2::Transform transform;
  tf2::convert(tf_laser.transform, transform);
  const tf2::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf2::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  // Sets this transform in rf2o 
  rf2o_ref.setLaserPose(laser_tf);

  return retrieved;
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


/**
 * Process the last scans to estimate the current odometry
*/
void CLaserOdometry2DNode::process()
{
  TimingScope process_timing(timing, TimingDiagnostics::PROCESS);
  // Do only run when a new scan is ready 
  if( rf2o_ref.is_initialized() && scan_available() )
  {
    // Process odometry estimation
    bool valid = false;
    {
      TimingScope compute_timing(timing, TimingDiagnostics::COMPUTE);
      valid = rf2o_ref.odometryCalculation(last_scan);
    }
    // Consume this attempt even when rejected. Never publish its stale pose.
    new_scan_available = false;
    if (valid) {
      last_valid_scan = last_scan;
      publish();
    } else {
      if (failure_inputs.size() < 3) {
        failure_inputs.push_back({rf2o_ref.failure_reason, last_valid_scan,
                                  last_scan, rf2o_ref.kai_loc_old_});
      }
      const auto rejected = rf2o_ref.rejected_estimates;
      // Bounded log rate: first three failures, then powers of two.
      if (rejected <= 3 || (rejected & (rejected-1)) == 0)
        RCLCPP_WARN(get_logger(), "RF2O_REJECT reason=%s count=%llu",
                    rf2o_ref.failure_reason, static_cast<unsigned long long>(rejected));
      if (std::string(rf2o_ref.failure_reason) == "reference_expired")
        throw std::runtime_error("RF2O_REFERENCE_EXPIRED: restart from a stationary pose");
    }
  }
  else
  {
    // This is a warning. We depend on laser scans, so no meaning running faster than scan freq.
    RCLCPP_WARN(get_logger(), "Waiting for laser_scans....");
  }
}


/**
 * This function is used to initialize the robot pose before estimating its odometry.
 * By default the odometry will start from pose_0, but when comparing different methods
 * it may be necessary to start from a different pose.
*/
void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  // Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


/**
 * Publish current odocmetry estimation over ROS
 * According to the node parameters it will publish over tf and/or especified topic
*/
void CLaserOdometry2DNode::publish()
{
  // 1. publish odom as a topic (no harm!)
  RCLCPP_DEBUG(get_logger(), "Publishing odom over topic:[%s]", odom_topic.c_str());
  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(0.0, 0.0, rf2o::getYaw(rf2o_ref.robot_pose_.rotation()));
  geometry_msgs::msg::Quaternion quaternion = tf2::toMsg(tf_quaternion);
  
  // compose odom msg
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = rf2o_ref.robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_ref.robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = quaternion;
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_ref.lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_ref.ang_speed;   //angular speed
  //publish the message
  {
    TimingScope publish_timing(timing, TimingDiagnostics::ODOM_PUBLISH);
    odom_pub->publish(odom);
  }

  // 2. publish over tf? (one one node should publish this transform!)
  if (publish_tf)
  {
    RCLCPP_DEBUG(get_logger(), "Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rf2o_ref.last_odom_time;    // the time of the last scan used!
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_ref.robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_ref.robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quaternion;
    //send the transform
    {
      TimingScope tf_timing(timing, TimingDiagnostics::TF_PUBLISH);
      odom_broadcaster->sendTransform(odom_trans);
    }
  }
}

} /* namespace rf2o */


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto myLaserOdomNode = std::make_shared<rf2o::CLaserOdometry2DNode>();

  // set desired loop rate
  rclcpp::Rate rate(myLaserOdomNode->freq);

  // Keep the original rate, executor and processing order. Measure elapsed
  // stages without periodic diagnostics I/O or new subscriptions.
  auto &timing=myLaserOdomNode->timing;
  double previous_loop=0;
  try {
    while (rclcpp::ok()) {
      if (timing.enabled) {
        const double now=rf2o::TimingDiagnostics::now();
        if (previous_loop!=0) timing.record(rf2o::TimingDiagnostics::LOOP_GAP,now-previous_loop,now);
        previous_loop=now;
      }
      {
        rf2o::TimingScope spin_timing(timing, rf2o::TimingDiagnostics::SPIN);
        rclcpp::spin_some(myLaserOdomNode);
      }
      myLaserOdomNode->process();
      {
        rf2o::TimingScope sleep_timing(timing, rf2o::TimingDiagnostics::SLEEP);
        rate.sleep();
      }
    }
  } catch (const std::exception &error) {
    if (timing.enabled) timing.write(std::cerr);
    std::cerr << "RF2O_FATAL " << error.what() << std::endl;
    // Return normally with failure so the node destructor saves bounded inputs.
    // Rethrowing out of main need not unwind locals before std::terminate.
    return 1;
  } catch (...) {
    if (timing.enabled) timing.write(std::cerr);
    std::cerr << "RF2O_FATAL unknown exception" << std::endl;
    return 1;
  }
  if (timing.enabled) timing.write(std::cerr);
  return 0;
}
