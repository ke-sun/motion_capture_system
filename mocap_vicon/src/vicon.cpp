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

#include "rclcpp/rclcpp.hpp"
#include <mocap_vicon/ViconDriver.h>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> nh = std::make_shared<rclcpp::Node>("vicon_node");

  mocap::ViconDriver driver(nh);
  if(!driver.init()) {
    RCLCPP_ERROR(nh->get_logger(), "Initialization of the Vicon driver failed");
    return -1;
  }
  RCLCPP_INFO(nh->get_logger(), "Successfully initialize Vicon connection!");

  while(rclcpp::ok())
  {
    driver.run();
    rclcpp::spin_some(nh);
  }

  // ROS_INFO("Shutting down");
  driver.disconnect();

  return 0;
}
