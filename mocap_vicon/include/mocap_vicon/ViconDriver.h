#ifndef VICON_DRIVER_H
#define VICON_DRIVER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <set>
#include <cmath>
#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/static_transform_broadcaster.h>

#include <mocap_base/MoCapDriverBase.h>
#include "ViconDataStreamSDK_CPP/DataStreamClient.h"



namespace mocap
{
    class ViconDriver: public MoCapDriverBase
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            // constructor
            ViconDriver():  client            (new ViconDataStreamSDK::CPP::Client()),
                            max_accel         (10.0),
                            frame_interval    (0.01),
                            process_noise     (Eigen::Matrix<double, 12, 12>::Zero()),
                            measurement_noise (Eigen::Matrix<double, 6, 6>::Zero()) 
            {
                tf_publisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
                return;
            }


            // destructor
            ~ViconDriver() 
            {
                disconnect();
                delete client;
                return;
            }

            // init Initialize the object & return True if successfully initialized
            bool init();

            // run Start acquiring data from the server
            void run();

            // disconnect Disconnect to the server. The function is called 
            // automatically when the destructor is called.
            void disconnect();

        private:
            // Disable the copy constructor and assign operator
            ViconDriver(const ViconDriver& );
            ViconDriver& operator=(const ViconDriver& );

            // Handle a frame which contains the info of all subjects
            void handleFrame();

            // Handle a the info of a single subject
            void handleSubject(const int& sub_idx);

            // Portal to communicate with the server
            ViconDataStreamSDK::CPP::Client* client;

            // Max acceleration
            double max_accel;

            // Average time interval between two frames
            double frame_interval;

            // Time stamp of the last frame  
            double last_time;

            // Counter for the computing frame frequency 
            int counter = 0;

            // Fixed window size  
            int window_size = 50;

            // A set to hold the model names
            std::set<std::string> model_set;

            // Convariance matrices for initializing kalman filters
            Eigen::Matrix<double, 12, 12> process_noise;
            Eigen::Matrix<double,  6,  6> measurement_noise;

            // For multi-threading
            boost::shared_mutex mtx;

            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher;
    };
}

#endif