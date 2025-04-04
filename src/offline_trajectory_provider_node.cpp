
/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "csv_reader.h"

#include <iomanip>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

namespace utilities {

/**
 * @brief The OfflineTrajectoryProviderNode class.
 * 
 * This class provides a ROS2 node that reads a trajectory from a CSV file and
 * publishes the trajectory as AccelStamped and Odometry messages.
*/
class OfflineTrajectoryProviderNode : public rclcpp::Node {
 public:
  // Constructor
  OfflineTrajectoryProviderNode(std::shared_ptr<CsvReader> pCsvReader);

  // Destructor
  ~OfflineTrajectoryProviderNode();

 private:
  // Pointer to CSV reader class instance
  std::shared_ptr<CsvReader> pCsvReader_;

  // File line count variables
  int file_line_count_;
  int current_line_count_;

  // Verbose level for console output
  int verbose_level_;

  std::string node_namespace_;

  // Timer
  rclcpp::TimerBase::SharedPtr pTimer_;

  // Callback function
  void trajectoryProvideLoopCallback();

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr
      pAccelStampedPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pOdometryPublisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
      pOdometryEnuRosPublisher_;

  // tf2 broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> pStaticTf2Broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> pDynamicTf2Broadcaster_;

  // tf2 broadcaster callback
  void publishStaticTf2Transform();
};

/**
 * @brief Constructor of the OfflineTrajectoryProviderNode class.
 * 
 * The constructor initializes the node, reads the CSV file, and sets up the
 * publishers and timer. The constructor also prints the file line count and
 * trajectory time to the console.
 * 
 * @param[in] pCsvReader Pointer to the CSV reader class instance.
 * 
 * @return None
*/
OfflineTrajectoryProviderNode::OfflineTrajectoryProviderNode(
    std::shared_ptr<CsvReader> pCsvReader)
    : Node("offline_trajectory_provider_node"),
      pCsvReader_(pCsvReader),
      file_line_count_(0),
      current_line_count_(0),
      verbose_level_(1),
      node_namespace_(this->get_namespace()) {  // 10000
  // Print to console that node has started
  RCLCPP_INFO(this->get_logger(), "Offline Trajectory Provider Node started.");

  // Declare parameters
  this->declare_parameter("topic_name_odom", "/odometry");
  this->declare_parameter("topic_name_accel", "/accel");
  this->declare_parameter("base_link_frame_id", "base_link_sname");
  this->declare_parameter("csv_file_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("csv_file_name", rclcpp::PARAMETER_STRING);
  this->declare_parameter("trajectory_rate", 100.0);
  this->declare_parameter("verbose_level", 1);

  // Get the parameters from the parameter server
  std::string topic_name_odom;
  std::string topic_name_accel;
  std::string base_link_frame_id;
  std::string csv_file_path;
  std::string csv_file_name;
  double trajectory_rate;

  this->get_parameter("topic_name_odom", topic_name_odom);
  this->get_parameter("topic_name_accel", topic_name_accel);
  this->get_parameter("base_link_frame_id", base_link_frame_id);
  this->get_parameter("csv_file_path", csv_file_path);
  this->get_parameter("csv_file_name", csv_file_name);
  this->get_parameter("trajectory_rate", trajectory_rate);
  this->get_parameter("verbose_level", verbose_level_);

  // Print to console
  RCLCPP_INFO(this->get_logger(), "Reading trajectory from path: %s",
              (csv_file_path + std::string("/data/") + csv_file_name).c_str());
  RCLCPP_INFO(this->get_logger(), "Topic name odometry: %s",
              node_namespace_ + topic_name_odom.c_str());
  RCLCPP_INFO(this->get_logger(), "Topic name accel: %s",
              node_namespace_ + topic_name_accel.c_str());
  // RCLCPP_INFO(this->get_logger(), "Base link frame id: %s", base_link_frame_id);
  RCLCPP_INFO(this->get_logger(), "Trajectory rate: %f Hz", trajectory_rate);
  RCLCPP_INFO(this->get_logger(), "Verbose level: %i", verbose_level_);

  // Try to open file
  try {
    // Open file
    pCsvReader_->openFile(csv_file_path + "/data/" + csv_file_name);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", e.what());

    // Shutdown node if file cannot be opened
    rclcpp::shutdown();
  }

  // Get line count
  file_line_count_ = pCsvReader_->getCountFileLines();

  // Print line count and trajectory time to console
  RCLCPP_INFO(this->get_logger(), "Trajectory file line count: %i.",
              file_line_count_);
  RCLCPP_INFO(this->get_logger(), "Trajectory time length: %dmin.",
              file_line_count_ * 100 / 60);

  // Define publishers
  pOdometryPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
      node_namespace_ + topic_name_odom, 10);

  pAccelStampedPublisher_ =
      this->create_publisher<geometry_msgs::msg::AccelStamped>(
          node_namespace_ + topic_name_accel, 10);

  // Calculate trajectory rate in ms
  int trajectory_rate_ms = 1 / trajectory_rate * 1000;  // Hz to ms

  // Define timer
  pTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(trajectory_rate_ms),
      std::bind(&OfflineTrajectoryProviderNode::trajectoryProvideLoopCallback,
                this));

  // Define static transform broadcaster
  pStaticTf2Broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Publish static transform
  publishStaticTf2Transform();

  // Define dynamic transform broadcaster
  pDynamicTf2Broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

OfflineTrajectoryProviderNode::~OfflineTrajectoryProviderNode() {

  // Close file if open
  pCsvReader_->closeFile();

  // Print to console
  RCLCPP_INFO(this->get_logger(), "Trajectory file closed.");
}

/**
 * @brief Callback function for the trajectory provide loop.
 * 
 * This function is called by the timer and reads the next line from the CSV
 * file. The line is then published as AccelStamped and Odometry messages.
 * 
 * @param[in] None
 * 
 * @return None
*/
void OfflineTrajectoryProviderNode::trajectoryProvideLoopCallback() {
  // Check if end of file is reached
  if (current_line_count_ >= file_line_count_) {
    RCLCPP_INFO(this->get_logger(), "Trajectory file closed. Stopping timer.");
    pTimer_->cancel();
    return;
  }

  // Read next line from CSV file and write to Eigen vector
  Eigen::VectorXd lineAsVector =
      pCsvReader_->readLineAsVector(current_line_count_);

  // Split vector into separate trajectory variables
  double time = lineAsVector.coeff(0);
  Eigen::Vector3d p_nb_n =
      lineAsVector.segment(1, 3) + Eigen::Vector3d(0, 0, -0.5);
  Eigen::Vector3d v_nb_b = lineAsVector.segment(4, 3);
  Eigen::Quaterniond q_b_n;
  q_b_n.w() = lineAsVector.coeff(7);
  q_b_n.x() = lineAsVector.coeff(8);
  q_b_n.y() = lineAsVector.coeff(9);
  q_b_n.z() = lineAsVector.coeff(10);
  Eigen::Vector3d f_ib_b = lineAsVector.segment(11, 3);
  Eigen::Vector3d w_ib_b = lineAsVector.segment(14, 3);

  // Increment line count
  current_line_count_++;

  if (verbose_level_ > 0) {
    // Output to console
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(),
        1000,  // Limit console output to every 1 second
        "Progress:                  %.2f%%",
        static_cast<double>(current_line_count_) * 100 / file_line_count_);
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(),
        1000,  // Limit console output to every 1 second
        "Remaining Time:            %.1f min ",
        ((file_line_count_ - current_line_count_) * 10e-3 - 10e-3) / 60);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Limit console output to every 1 second
                         "Trajectory idx:            %i", current_line_count_);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Limit console output to every 1 second
                         "Time:                      %.3f s",
                         time);  // Assuming 3 decimal places
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Limit console output to every 1 second
                         "Position (p_nb_n):         %.3f %.3f %.3f m",
                         p_nb_n(0), p_nb_n(1), p_nb_n(2));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Limit console output to every 1 second
                         "Velocity (v_nb_b):         %.3f %.3f %.3f m/s",
                         v_nb_b(0), v_nb_b(1), v_nb_b(2));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Limit console output to every 1 second
                         "Quaternion (q_b_n):        %.3f %.3f %.3f %.3f",
                         q_b_n.w(), q_b_n.x(), q_b_n.y(), q_b_n.z());
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Limit console output to every 1 second
                         "Specific force (f_ib_b):   %.3f %.3f %.3f m/s²",
                         f_ib_b(0), f_ib_b(1), f_ib_b(2));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                         1000,  // Limit console output to every 1 second
                         "Angular Velocity (w_ib_b): %.3f %.3f %.3f rad/s\n",
                         w_ib_b(0), w_ib_b(1), w_ib_b(2));
  }

  // Get base link frame id from parameter server
  std::string base_link_frame_id;
  this->get_parameter_or("base_link_frame_id", base_link_frame_id,
                         std::string("base_link_sname"));

  // Publish AccelStamped message
  geometry_msgs::msg::AccelStamped accelStamped;

  accelStamped.header.stamp = this->now();
  accelStamped.header.frame_id = base_link_frame_id;

  accelStamped.accel.linear.x = f_ib_b(0);
  accelStamped.accel.linear.y = f_ib_b(1);
  accelStamped.accel.linear.z = f_ib_b(2);

  accelStamped.accel.angular.x = 0.0;
  accelStamped.accel.angular.y = 0.0;
  accelStamped.accel.angular.z = 0.0;

  pAccelStampedPublisher_->publish(accelStamped);

  // Publish Odometry message
  nav_msgs::msg::Odometry odometry;

  odometry.header.stamp = this->now();
  odometry.header.frame_id = "world_ned";
  odometry.child_frame_id = base_link_frame_id;

  odometry.pose.pose.position.x = p_nb_n(0);
  odometry.pose.pose.position.y = p_nb_n(1);
  odometry.pose.pose.position.z = p_nb_n(2);

  odometry.pose.pose.orientation.w = q_b_n.w();
  odometry.pose.pose.orientation.x = q_b_n.x();
  odometry.pose.pose.orientation.y = q_b_n.y();
  odometry.pose.pose.orientation.z = q_b_n.z();

  odometry.twist.twist.linear.x = v_nb_b(0);
  odometry.twist.twist.linear.y = v_nb_b(1);
  odometry.twist.twist.linear.z = v_nb_b(2);

  odometry.twist.twist.angular.x = w_ib_b(0);
  odometry.twist.twist.angular.y = w_ib_b(1);
  odometry.twist.twist.angular.z = w_ib_b(2);

  pOdometryPublisher_->publish(odometry);

  // Publish dynamic transform
  geometry_msgs::msg::TransformStamped tf2Msg;

  tf2Msg.header.stamp = this->now();
  tf2Msg.header.frame_id = "world_ned";
  tf2Msg.child_frame_id = base_link_frame_id;

  tf2Msg.transform.translation.x = p_nb_n(0);
  tf2Msg.transform.translation.y = p_nb_n(1);
  tf2Msg.transform.translation.z = p_nb_n(2);

  tf2Msg.transform.rotation.w = q_b_n.w();
  tf2Msg.transform.rotation.x = q_b_n.x();
  tf2Msg.transform.rotation.y = q_b_n.y();
  tf2Msg.transform.rotation.z = q_b_n.z();

  pDynamicTf2Broadcaster_->sendTransform(tf2Msg);
}

/**
 * @brief Publish static tf2 transform.
 * 
 * This function publishes a static tf2 transform from world (ENU) to world_ned
 * (NED) and from base_link (FLU) to base_link_sname (FRD). The function is
 * called once at the beginning of the node.
 * 
 * @param[in] None
 * 
 * @return None
*/
void OfflineTrajectoryProviderNode::publishStaticTf2Transform() {
  // Get base link frame id from parameter server
  std::string base_link_frame_id;
  this->get_parameter_or("base_link_frame_id", base_link_frame_id,
                         std::string("base_link_sname"));

  // Define static transform
  geometry_msgs::msg::TransformStamped staticTransformStamped;

  staticTransformStamped.header.stamp = this->now();
  staticTransformStamped.header.frame_id = "world";     // ENU
  staticTransformStamped.child_frame_id = "world_ned";  // NED

  staticTransformStamped.transform.translation.x = 0.0;
  staticTransformStamped.transform.translation.y = 0.0;
  staticTransformStamped.transform.translation.z = 0.0;

  // Set the transformation rotation for ENU to NED
  staticTransformStamped.transform.rotation.w = 0.0;
  staticTransformStamped.transform.rotation.x = sqrt(2) / 2;
  staticTransformStamped.transform.rotation.y = sqrt(2) / 2;
  staticTransformStamped.transform.rotation.z = 0.0;

  // Publish static transform for world
  pStaticTf2Broadcaster_->sendTransform(staticTransformStamped);

  // Define static transform
  geometry_msgs::msg::TransformStamped staticTransformStampedBaseLink;

  staticTransformStampedBaseLink.header.stamp = this->now();
  staticTransformStampedBaseLink.header.frame_id = "base_link";        // FLU
  staticTransformStampedBaseLink.child_frame_id = base_link_frame_id;  // FRD

  staticTransformStampedBaseLink.transform.translation.x = 0.0;
  staticTransformStampedBaseLink.transform.translation.y = 0.0;
  staticTransformStampedBaseLink.transform.translation.z = 0.0;

  // Set the transformation rotation from FLU to FRD
  staticTransformStampedBaseLink.transform.rotation.w = 0.0;
  staticTransformStampedBaseLink.transform.rotation.x = 1.0;
  staticTransformStampedBaseLink.transform.rotation.y = 0.0;
  staticTransformStampedBaseLink.transform.rotation.z = 0.0;

  // Publish static transform for base_link
  pStaticTf2Broadcaster_->sendTransform(staticTransformStampedBaseLink);
}

}  // namespace utilities

/**
 * @brief Main function.
 * 
 * This function is the main function of the offline_trajectory_provider_node.
 * 
 * @param[in] argc The number of command line arguments.
 * @param[in] argv The command line arguments.
 * 
 * @return 0
*/
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  // Create CSV reader object instance
  auto pCsvReader = std::make_shared<utilities::CsvReader>();

  // auto pOfflineTrajectoryProviderNode = ;

  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<utilities::OfflineTrajectoryProviderNode>(pCsvReader));
  rclcpp::shutdown();

  return 0;
}
