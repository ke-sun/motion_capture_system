/*
 * Copyright [2015] [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mocap_base/MoCapDriverBase.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std;
using namespace Eigen;

namespace mocap {

 Subject::Subject(std::shared_ptr<rclcpp::Node> nptr, const string& sub_name,
     const std::string& p_frame):
   name         (sub_name),
   status       (LOST),
   nh_ptr       (nptr),
   parent_frame (p_frame){
 
   pub_filter = nh_ptr->create_publisher<nav_msgs::msg::Odometry>(name+"/odom", 10);
   pub_raw = nh_ptr->create_publisher<geometry_msgs::msg::PoseStamped>(name+"/pose", 10);
   pub_points_raw = nh_ptr->create_publisher<geometry_msgs::msg::PoseArray>(name+"/individual_marker_array", 10);
   return;
 }
 
 // Get and set name of the subject
 const string& Subject::getName() {
   boost::shared_lock<boost::shared_mutex> read_lock(mtx);
   return name;
 }
 void Subject::setName(const string& sub_name) {
   boost::unique_lock<boost::shared_mutex> write_lock(mtx);
   name = sub_name;
 }
 
 // Enable or diable the subject
 const Subject::Status& Subject::getStatus() {
   boost::shared_lock<boost::shared_mutex> read_lock(mtx);
   return status;
 }
 void Subject::enable() {
   boost::unique_lock<boost::shared_mutex> write_lock(mtx);
   status = INITIALIZING;
 }
 void Subject::disable() {
   boost::unique_lock<boost::shared_mutex> write_lock(mtx);
   kFilter.reset();
   status = LOST;
 }
 
 // Get the state of the subject
 const Quaterniond& Subject::getAttitude() {
   boost::shared_lock<boost::shared_mutex> read_lock(mtx);
   return kFilter.attitude;
 }
 const Vector3d& Subject::getPosition() {
   boost::shared_lock<boost::shared_mutex> read_lock(mtx);
   return kFilter.position;
 }
 const Vector3d& Subject::getAngularVel() {
   boost::shared_lock<boost::shared_mutex> read_lock(mtx);
   return kFilter.angular_vel;
 }
 const Vector3d& Subject::getLinearVel() {
   boost::shared_lock<boost::shared_mutex> read_lock(mtx);
   return kFilter.linear_vel;
 }
 
 // Set the noise parameter for the kalman filter
 bool Subject::setParameters(
     const Matrix<double, 12, 12>& u_cov,
     const Matrix<double, 6, 6>& m_cov,
     const int& freq) {
   boost::unique_lock<boost::shared_mutex> write_lock(mtx);
   return kFilter.init(u_cov, m_cov, freq);
 }
 
 // Process the new measurement
 void Subject::processNewMeasurement(
     const double& time,
     const Quaterniond& m_attitude,
     const Vector3d& m_position) {
 
   boost::unique_lock<boost::shared_mutex> write_lock(mtx);
 
   // Publish raw data from mocap system
   geometry_msgs::msg::PoseStamped pose_raw;
   // time is a double that was converted from Time to Double with toSec()
   pose_raw.header.stamp = rclcpp::Time(time * 1e9);
   pose_raw.header.frame_id = parent_frame;
   pose_raw.pose.orientation = tf2::toMsg(m_attitude);
   pose_raw.pose.position = tf2::toMsg(m_position);
   pub_raw->publish(pose_raw);
 
   if (!kFilter.isReady()) {
     status = INITIALIZING;
     kFilter.prepareInitialCondition(time, m_attitude, m_position);
     return;
   }
 
   status = TRACKED;
   // Perfrom the kalman filter
   kFilter.prediction(time);
   kFilter.update(m_attitude, m_position);
 
   // Publish the new state
   nav_msgs::msg::Odometry odom_filter;
   // time is a double that was converted from Time to Double with toSec()
   odom_filter.header.stamp = rclcpp::Time(time * 1e9);
   odom_filter.header.frame_id = parent_frame;
   odom_filter.child_frame_id = name + "/base_link";
   odom_filter.pose.pose.orientation = tf2::toMsg(kFilter.attitude);
   odom_filter.pose.pose.position = tf2::toMsg(kFilter.position);
   // cannot use toMsg, only converts Vector3d -> Point, not Vector
   // odom_filter.twist.twist.angular = tf2::toMsg(kFilter.angular_vel);
   // odom_filter.twist.twist.linear = tf2::toMsg(kFilter.linear_vel);
   odom_filter.twist.twist.angular.x = kFilter.angular_vel.x();
   odom_filter.twist.twist.angular.y = kFilter.angular_vel.y();
   odom_filter.twist.twist.angular.z = kFilter.angular_vel.z();

   odom_filter.twist.twist.linear.x = kFilter.linear_vel.x();
   odom_filter.twist.twist.linear.y = kFilter.linear_vel.y();
   odom_filter.twist.twist.linear.z = kFilter.linear_vel.z();

   // tf::quaternionEigenToMsg(kFilter.attitude, odom_filter.pose.pose.orientation);
   // tf::pointEigenToMsg(kFilter.position, odom_filter.pose.pose.position);
   // tf::vectorEigenToMsg(kFilter.angular_vel, odom_filter.twist.twist.angular);
   // tf::vectorEigenToMsg(kFilter.linear_vel, odom_filter.twist.twist.linear);
   
   // To be compatible with the covariance in ROS, we have to do some shifting
   Map<Matrix<double, 6, 6, RowMajor> > pose_cov(odom_filter.pose.covariance.begin());
   Map<Matrix<double, 6, 6, RowMajor> > vel_cov(odom_filter.twist.covariance.begin());
   pose_cov.topLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(3, 3);
   pose_cov.topRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(3, 0);
   pose_cov.bottomLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(0, 3);
   pose_cov.bottomRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(0, 0);
   vel_cov.topLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(9, 9);
   vel_cov.topRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(9, 6);
   vel_cov.bottomLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(6, 9);
   vel_cov.bottomRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(6, 6);
 
   pub_filter->publish(odom_filter);
 
   return;
 }
 
 void Subject::publishMarkerPoints(
     const double& time,
     const std::vector<std::array<double,3>> marker_pos) {
   geometry_msgs::msg::PoseArray poses;
 
   for (auto i: marker_pos) {
 
     // time is a double that was converted from Time to Double with toSec()
     poses.header.stamp = rclcpp::Time(time * 1e9);
     poses.header.frame_id = parent_frame;
     geometry_msgs::msg::Pose pose;
     pose.position.x = i[0];
     pose.position.y = i[1];
     pose.position.z = i[2];
     pose.orientation.x = 0;
     pose.orientation.y = 0;
     pose.orientation.z = 0;
     pose.orientation.w = 1;
     poses.poses.push_back(pose);
 
   }
   pub_points_raw->publish(poses);
 }

} // namespace
