/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
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

#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// #include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <mocap_vicon/ViconDriver.h>

using namespace std;
using namespace Eigen;
namespace ViconSDK = ViconDataStreamSDK::CPP;

namespace mocap {

bool ViconDriver::init() {

  server_address = this->nh->declare_parameter<string>("server_address", string("mocap.perch"));
  model_list = this->nh->declare_parameter<vector<string>>("model_list", vector<string>(0));
  frame_rate = this->nh->declare_parameter<int>("frame_rate", 100);
  max_accel = this->nh->declare_parameter<float>("max_accel", 10.0);
  publish_tf = this->nh->declare_parameter<bool>("publish_tf", false);
  publish_pts = this->nh->declare_parameter<bool>("publish_pts", true);
  fixed_frame_id = this->nh->declare_parameter<string>("fixed_frame_id", string("mocap"));

  this->nh->get_parameter("server_address", server_address);
  this->nh->get_parameter("model_list", model_list);
  this->nh->get_parameter("frame_rate", frame_rate);
  this->nh->get_parameter("max_accel", max_accel);
  this->nh->get_parameter("publish_tf", publish_tf);
  this->nh->get_parameter("publish_pts", publish_pts);
  this->nh->get_parameter("fixed_frame_id", fixed_frame_id);

  frame_interval = 1.0 / static_cast<double>(frame_rate);
  double& dt = frame_interval;
  process_noise.topLeftCorner<6, 6>() =
    0.5*Matrix<double, 6, 6>::Identity()*dt*dt*max_accel;
  process_noise.bottomRightCorner<6, 6>() =
    Matrix<double, 6, 6>::Identity()*dt*max_accel;
  process_noise *= process_noise; // Make it a covariance
  measurement_noise =
    Matrix<double, 6, 6>::Identity()*1e-3;
  measurement_noise *= measurement_noise; // Make it a covariance
  model_set.insert(model_list.begin(), model_list.end());

  timespec ts_sleep;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100000000;

  // Connect to the server
  RCLCPP_INFO(this->nh->get_logger(), "Connecting to Vicon Datastream server at %s", server_address.c_str());
  bool is_connected = false;
  for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) {
    client->Connect(server_address);
    if(client->IsConnected().Connected) {
      is_connected = true;
      break;
    }
    else
      nanosleep(&ts_sleep, NULL);
  }

  // Report if cannot connect
  if (!is_connected) {
    RCLCPP_WARN(this->nh->get_logger(), "Cannot Connect to Vicon server at %s", server_address.c_str());
    return false;
  }

  // Configure the connection
  RCLCPP_INFO(this->nh->get_logger(), "Successfully Connect to Vicon server at %s", server_address.c_str());
  client->SetStreamMode(ViconSDK::StreamMode::ClientPull);
  client->SetAxisMapping(ViconSDK::Direction::Forward,
      ViconSDK::Direction::Left, ViconSDK::Direction::Up);
  client->EnableSegmentData();
  if(!client->IsSegmentDataEnabled().Enabled) {
    RCLCPP_WARN(this->nh->get_logger(), "Segment data cannot be enabled.");
    return false;
  }
  RCLCPP_INFO(this->nh->get_logger(), "Successfully configure Vicon server at %s", server_address.c_str());

  // Need to wait for some time after enabling data else you get junk frames
  //struct timespec ts_sleep;
  ts_sleep.tv_sec = 0;
  ts_sleep.tv_nsec = 100000000;
  nanosleep(&ts_sleep, NULL);

  return true;
}

void ViconDriver::run() {
  ViconSDK::Result::Enum result = client->GetFrame().Result;
  if (result != ViconSDK::Result::Success)
    return;
  handleFrame();
  return;
}

void ViconDriver::disconnect() {
  RCLCPP_INFO_STREAM(this->nh->get_logger(), "Disconnected with the server at "
      << server_address);
  client->Disconnect();
  return;
}

void ViconDriver::handleFrame() {
  int body_count = client->GetSubjectCount().SubjectCount;
  // Assign each subject with a thread
  vector<boost::thread> subject_threads;
  subject_threads.reserve(body_count);

  for (int i = 0; i< body_count; ++i) {
    string subject_name =
      client->GetSubjectName(i).SubjectName;

    // in ROS2, you cannot have an empty list, meaning the way we have to define
    // one is by setting model_list = ['']
    
    if (model_set.size() == 1 ) {
      if (auto search = model_set.find(""); search != model_set.end()) {
        model_set.erase("");
      }
    }


    // Process the subject if required
    if (model_set.empty() || model_set.count(subject_name)) {
      // Create a new subject if it does not exist
      if (subjects.find(subject_name) == subjects.end()) {
        subjects[subject_name] = Subject::SubjectPtr(
            new Subject(nh, subject_name, fixed_frame_id));
        subjects[subject_name]->setParameters(
            process_noise, measurement_noise, frame_rate);
      }
      // Handle the subject in a different thread
      subject_threads.emplace_back(&ViconDriver::handleSubject, this, i);
      //handleSubject(i);
    }
  }

  // Wait for all the threads to stop
  for(auto &thread : subject_threads) {
    thread.join();
  }

  // Send out warnings
  for (auto it = subjects.begin();
      it != subjects.end(); ++it) {
    Subject::Status status = it->second->getStatus();
    if (status == Subject::LOST)
      RCLCPP_WARN_THROTTLE(this->nh->get_logger(), *this->nh->get_clock(), 1, "Lose track of subject %s", (it->first).c_str());
    else if (status == Subject::INITIALIZING)
      RCLCPP_WARN(this->nh->get_logger(), "Initialize subject %s", (it->first).c_str());
  }

  return;
}

void ViconDriver::handleSubject(const int& sub_idx) {

  boost::unique_lock<boost::shared_mutex> write_lock(mtx);
  // We assume each subject has only one segment
  string subject_name = client->GetSubjectName(sub_idx).SubjectName;
  double time = this->nh->get_clock()->now().seconds();

  // Publish individual points of each marker
  if (publish_pts) {
    client->EnableMarkerData();
    ViconDataStreamSDK::CPP::Output_GetMarkerCount marker_count = 
      client->GetMarkerCount(subject_name);


    // Vector of each point, which contains X,Y,Z data
    std::vector<std::array<double, 3>> marker_points;
    marker_points.reserve(marker_count.MarkerCount);

    for (unsigned int i = 0; i < marker_count.MarkerCount; i++) {
      // Get the Marker Name
      ViconDataStreamSDK::CPP::Output_GetMarkerName marker_name = 
        client->GetMarkerName(subject_name, i);
      
      // Get the position of that marker in mm
      if (marker_name.Result != ViconDataStreamSDK::CPP::Result::InvalidSubjectName) {
        ViconDataStreamSDK::CPP::Output_GetMarkerGlobalTranslation marker_pos = 
          client->GetMarkerGlobalTranslation(subject_name, marker_name.MarkerName);

        if (marker_pos.Result == ViconDataStreamSDK::CPP::Result::Success) {
          std::array<double, 3> position = {marker_pos.Translation[0],
                                            marker_pos.Translation[1],
                                            marker_pos.Translation[2]};
          marker_points.emplace_back(position);
        }
      }
    }
    // Publish to ROS
    subjects[subject_name]->publishMarkerPoints(time, marker_points);
  }

  string segment_name = client->GetSegmentName(subject_name, 0).SegmentName;
  // Get the pose for the subject
  ViconSDK::Output_GetSegmentGlobalTranslation trans =
      client->GetSegmentGlobalTranslation(subject_name, segment_name);
  ViconSDK::Output_GetSegmentGlobalRotationQuaternion quat =
      client->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
  write_lock.unlock();

  //boost::shared_lock<boost::shared_mutex> read_lock(mtx);
  if(trans.Result != ViconSDK::Result::Success ||
     quat.Result != ViconSDK::Result::Success ||
     trans.Occluded || quat.Occluded) {
    subjects[subject_name]->disable();
    return;
  }

  // Convert the msgs to Eigen type
  Eigen::Quaterniond m_att(quat.Rotation[3],
      quat.Rotation[0], quat.Rotation[1], quat.Rotation[2]);
  Eigen::Vector3d m_pos(trans.Translation[0]/1000,
      trans.Translation[1]/1000, trans.Translation[2]/1000);

  // Re-enable the object if it is lost previously
  if (subjects[subject_name]->getStatus() == Subject::LOST) {
    subjects[subject_name]->enable();
  }

  // Feed the new measurement to the subject
  subjects[subject_name]->processNewMeasurement(time, m_att, m_pos);
  //read_lock.unlock();


  // Publish tf if requred
  if (publish_tf &&
      subjects[subject_name]->getStatus() == Subject::TRACKED) {

    Quaterniond att = subjects[subject_name]->getAttitude();
    Vector3d pos = subjects[subject_name]->getPosition();
    tf2::Quaternion att_tf;
    tf2::Vector3 pos_tf;
    att_tf.setX(att.x());
    att_tf.setY(att.y());
    att_tf.setZ(att.z());
    att_tf.setW(att.w());

    pos_tf.setX(pos.x());
    pos_tf.setY(pos.y());
    pos_tf.setZ(pos.z());
    pos_tf.setW(pos.w());


    // tf2::StampedTransform stamped_transform =
    //   tf2::StampedTransform(tf2::Transform(att_tf, pos_tf),
    //     this->nh->get_clock()->now(), fixed_frame_id, subject_name);

    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = this->nh->get_clock()->now();
    stamped_transform.header.frame_id = fixed_frame_id;
    stamped_transform.child_frame_id = subject_name;
    stamped_transform.transform.translation.x = pos_tf.x();
    stamped_transform.transform.translation.y = pos_tf.y();
    stamped_transform.transform.translation.z = pos_tf.z();
    stamped_transform.transform.rotation.x = att_tf.x();
    stamped_transform.transform.rotation.y = att_tf.y();
    stamped_transform.transform.rotation.z = att_tf.z();

    
    
    write_lock.lock();
    static tf2_ros::StaticTransformBroadcaster static_broadcaster(nh);
    static_broadcaster.sendTransform(stamped_transform);
    //tf_publisher.sendTransform(stamped_transform);
    write_lock.unlock();
  }

  return;
}

}
