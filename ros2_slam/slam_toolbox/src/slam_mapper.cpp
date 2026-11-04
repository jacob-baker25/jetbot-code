/*
 * slam_mapper
 * Copyright (c) 2018, Simbe Robotics
 * Copyright (c) 2018, Steve Macenski
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include <memory>
#include <stdexcept>
#include "slam_toolbox/slam_mapper.hpp"

namespace mapper_utils
{
namespace
{

// declare_parameter returns a value; it does not modify its default argument.
// Read explicitly so overrides, predeclared parameters, false and zero work.
template<typename T>
T readMapperParameter(
  const rclcpp::Node::SharedPtr & node, const std::string & name, T default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  T value = default_value;
  if (!node->get_parameter(name, value)) {
    throw std::runtime_error("Missing mapper parameter: " + name);
  }
  return value;
}

}  // namespace


/*****************************************************************************/
SMapper::SMapper()
/*****************************************************************************/
{
  mapper_ = std::make_unique<karto::Mapper>();
}

/*****************************************************************************/
SMapper::~SMapper()
/*****************************************************************************/
{
  mapper_.reset();
}

/*****************************************************************************/
karto::Mapper * SMapper::getMapper()
/*****************************************************************************/
{
  return mapper_.get();
}

/*****************************************************************************/
void SMapper::setMapper(karto::Mapper * mapper)
/*****************************************************************************/
{
  mapper_.reset(mapper);
}

/*****************************************************************************/
void SMapper::clearLocalizationBuffer()
/*****************************************************************************/
{
  mapper_->ClearLocalizationBuffer();
}

/*****************************************************************************/
karto::OccupancyGrid * SMapper::getOccupancyGrid(const double & resolution)
/*****************************************************************************/
{
  karto::OccupancyGrid * occ_grid = nullptr;
  return karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(),
           resolution);
}

/*****************************************************************************/
tf2::Transform SMapper::toTfPose(const karto::Pose2 & pose) const
/*****************************************************************************/
{
  tf2::Transform new_pose;
  new_pose.setOrigin(tf2::Vector3(pose.GetX(), pose.GetY(), 0.));
  tf2::Quaternion q;
  q.setRPY(0., 0., pose.GetHeading());
  new_pose.setRotation(q);
  return new_pose;
}

/*****************************************************************************/
karto::Pose2 SMapper::toKartoPose(const tf2::Transform & pose) const
/*****************************************************************************/
{
  karto::Pose2 transformed_pose;
  transformed_pose.SetX(pose.getOrigin().x());
  transformed_pose.SetY(pose.getOrigin().y());
  transformed_pose.SetHeading(tf2::getYaw(pose.getRotation()));
  return transformed_pose;
}

/*****************************************************************************/
void SMapper::configure(const rclcpp::Node::SharedPtr & node)
/*****************************************************************************/
{
  const bool use_scan_matching = readMapperParameter<bool>(node,
    "use_scan_matching", true);
  mapper_->setParamUseScanMatching(use_scan_matching);

  const bool use_scan_barycenter = readMapperParameter<bool>(node,
    "use_scan_barycenter", true);
  mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  const double minimum_travel_distance = readMapperParameter<double>(node,
    "minimum_travel_distance", 0.5);
  mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  const double minimum_travel_heading = readMapperParameter<double>(node,
    "minimum_travel_heading", 0.5);
  mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  const int scan_buffer_size = readMapperParameter<int>(node,
    "scan_buffer_size", 10);
  mapper_->setParamScanBufferSize(scan_buffer_size);

  const double scan_buffer_maximum_scan_distance = readMapperParameter<double>(node,
    "scan_buffer_maximum_scan_distance", 10);
  mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  const double link_match_minimum_response_fine = readMapperParameter<double>(node,
    "link_match_minimum_response_fine", 0.1);
  mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  const double link_scan_maximum_distance = readMapperParameter<double>(node,
    "link_scan_maximum_distance", 1.5);
  mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  const double loop_search_maximum_distance = readMapperParameter<double>(node,
    "loop_search_maximum_distance", 3.0);
  mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  const bool do_loop_closing = readMapperParameter<bool>(node,
    "do_loop_closing", true);
  mapper_->setParamDoLoopClosing(do_loop_closing);

  const int loop_match_minimum_chain_size = readMapperParameter<int>(node,
    "loop_match_minimum_chain_size", 10);
  mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  const double loop_match_maximum_variance_coarse = readMapperParameter<double>(node,
    "loop_match_maximum_variance_coarse", 3.0);
  mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  const double loop_match_minimum_response_coarse = readMapperParameter<double>(node,
    "loop_match_minimum_response_coarse", 0.35);
  mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  const double loop_match_minimum_response_fine = readMapperParameter<double>(node,
    "loop_match_minimum_response_fine", 0.45);
  mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters
  const double correlation_search_space_dimension = readMapperParameter<double>(node,
    "correlation_search_space_dimension", 0.5);
  mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  const double correlation_search_space_resolution = readMapperParameter<double>(node,
    "correlation_search_space_resolution", 0.01);
  mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  const double correlation_search_space_smear_deviation = readMapperParameter<double>(node,
    "correlation_search_space_smear_deviation", 0.1);
  mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters
  const double loop_search_space_dimension = readMapperParameter<double>(node,
    "loop_search_space_dimension", 8.0);
  mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  const double loop_search_space_resolution = readMapperParameter<double>(node,
    "loop_search_space_resolution", 0.05);
  mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  const double loop_search_space_smear_deviation = readMapperParameter<double>(node,
    "loop_search_space_smear_deviation", 0.03);
  mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters
  const double distance_variance_penalty = readMapperParameter<double>(node,
    "distance_variance_penalty", 0.5);
  mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

  const double angle_variance_penalty = readMapperParameter<double>(node,
    "angle_variance_penalty", 1.0);
  mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

  const double fine_search_angle_offset = readMapperParameter<double>(node,
    "fine_search_angle_offset", 0.00349);
  mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  const double coarse_search_angle_offset = readMapperParameter<double>(node,
    "coarse_search_angle_offset", 0.349);
  mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  const double coarse_angle_resolution = readMapperParameter<double>(node,
    "coarse_angle_resolution", 0.0349);
  mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  const double minimum_angle_penalty = readMapperParameter<double>(node,
    "minimum_angle_penalty", 0.9);
  mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  const double minimum_distance_penalty = readMapperParameter<double>(node,
    "minimum_distance_penalty", 0.05);
  mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  const bool use_response_expansion = readMapperParameter<bool>(node,
    "use_response_expansion", true);
  mapper_->setParamUseResponseExpansion(use_response_expansion);
  // Read back Karto, not the ROS parameter store. Normalize legacy getters
  // to the units consumed by setters and declared in YAML.
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG use_scan_matching=%.12g",
    static_cast<double>(mapper_->getParamUseScanMatching()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG use_scan_barycenter=%.12g",
    static_cast<double>(mapper_->getParamUseScanBarycenter()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG minimum_travel_distance=%.12g",
    static_cast<double>(mapper_->getParamMinimumTravelDistance()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG minimum_travel_heading=%.12g",
    static_cast<double>(karto::math::DegreesToRadians(mapper_->getParamMinimumTravelHeading())));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG scan_buffer_size=%.12g",
    static_cast<double>(mapper_->getParamScanBufferSize()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG scan_buffer_maximum_scan_distance=%.12g",
    static_cast<double>(mapper_->getParamScanBufferMaximumScanDistance()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG link_match_minimum_response_fine=%.12g",
    static_cast<double>(mapper_->getParamLinkMatchMinimumResponseFine()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG link_scan_maximum_distance=%.12g",
    static_cast<double>(mapper_->getParamLinkScanMaximumDistance()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_search_maximum_distance=%.12g",
    static_cast<double>(mapper_->getParamLoopSearchMaximumDistance()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG do_loop_closing=%.12g",
    static_cast<double>(mapper_->getParamDoLoopClosing()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_match_minimum_chain_size=%.12g",
    static_cast<double>(mapper_->getParamLoopMatchMinimumChainSize()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_match_maximum_variance_coarse=%.12g",
    static_cast<double>(mapper_->getParamLoopMatchMaximumVarianceCoarse()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_match_minimum_response_coarse=%.12g",
    static_cast<double>(mapper_->getParamLoopMatchMinimumResponseCoarse()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_match_minimum_response_fine=%.12g",
    static_cast<double>(mapper_->getParamLoopMatchMinimumResponseFine()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG correlation_search_space_dimension=%.12g",
    static_cast<double>(mapper_->getParamCorrelationSearchSpaceDimension()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG correlation_search_space_resolution=%.12g",
    static_cast<double>(mapper_->getParamCorrelationSearchSpaceResolution()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG correlation_search_space_smear_deviation=%.12g",
    static_cast<double>(mapper_->getParamCorrelationSearchSpaceSmearDeviation()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_search_space_dimension=%.12g",
    static_cast<double>(mapper_->getParamLoopSearchSpaceDimension()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_search_space_resolution=%.12g",
    static_cast<double>(mapper_->getParamLoopSearchSpaceResolution()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG loop_search_space_smear_deviation=%.12g",
    static_cast<double>(mapper_->getParamLoopSearchSpaceSmearDeviation()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG distance_variance_penalty=%.12g",
    static_cast<double>(mapper_->getParamDistanceVariancePenalty()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG angle_variance_penalty=%.12g",
    static_cast<double>(mapper_->getParamAngleVariancePenalty()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG fine_search_angle_offset=%.12g",
    static_cast<double>(mapper_->getParamFineSearchAngleOffset()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG coarse_search_angle_offset=%.12g",
    static_cast<double>(mapper_->getParamCoarseSearchAngleOffset()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG coarse_angle_resolution=%.12g",
    static_cast<double>(mapper_->getParamCoarseAngleResolution()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG minimum_angle_penalty=%.12g",
    static_cast<double>(mapper_->getParamMinimumAnglePenalty()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG minimum_distance_penalty=%.12g",
    static_cast<double>(mapper_->getParamMinimumDistancePenalty()));
  RCLCPP_INFO(node->get_logger(), "KARTO_CONFIG use_response_expansion=%.12g",
    static_cast<double>(mapper_->getParamUseResponseExpansion()));

}

/*****************************************************************************/
void SMapper::Reset()
/*****************************************************************************/
{
  mapper_->Reset();
}

}  // namespace mapper_utils
