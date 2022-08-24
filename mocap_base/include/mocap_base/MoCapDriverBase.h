#ifndef MOCAP_DRIVER_BASE_H
#define MOCAP_DRIVER_BASE_H

#include <map>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mocap_base/KalmanFilter.h>

namespace mocap
{

  class Subject : public rclcpp::Node
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      enum Status 
      {
        LOST,
        INITIALIZING,
        TRACKED
      };

      // Constructor 
      Subject(const std::string& sub_name,
              const std::string& p_frame) : Node("vicon"),
                                            name(sub_name),
                                            status(LOST),
                                            parent_frame(p_frame)
      {
        pub_filter = this->create_publisher<nav_msgs::msg::Odometry>(name+"/odom", 10);
        pub_raw = this->create_publisher<geometry_msgs::msg::PoseStamped>(name+"/pose", 10);
      }

      // destructor 
      ~Subject() {}

      
      // @brief getName setName. Get and set the name of the object
      const std::string& getName();
      void setName(const std::string& sub_name);

      
      // @brief isActive Tells if the object is still active or not
      const Status& getStatus();
      void enable();
      void disable();


      // @brief getAttitude getPosition getAngularVel getLinearVel 
      //        Returns the state of the object
      const Eigen::Quaterniond& getAttitude();
      const Eigen::Vector3d& getPosition();
      const Eigen::Vector3d& getAngularVel();
      const Eigen::Vector3d& getLinearVel();


      // @brief setNoiseParameter Set noise parameters for the kalman filter
      //  param u_cov Input noise, m_cov Measurement noise
      //  return True if success
      bool setParameters(const Eigen::Matrix<double, 12, 12>& u_cov,
                        const Eigen::Matrix<double, 6, 6>& m_cov,
                        const int& freq);


      // processNewMeasurement Process new measurements from the mocap system
      //   m_attitude Measured attitude, m_position Measured position
      void processNewMeasurement(const double& time,
                                const Eigen::Quaterniond& m_attitude,
                                const Eigen::Vector3d& m_position);

      typedef boost::shared_ptr<Subject> SubjectPtr;
      typedef const boost::shared_ptr<Subject> SubjectConstPtr;

    private:
      // Disable copy constructor and assign operator
      Subject(const Subject&);
      Subject& operator=(const Subject&);

      // Name of the subject
      std::string name;

      // Error state Kalman filter
      KalmanFilter kFilter;

      // Tells the status of the object
      Status status;

      // Prevent cocurrent reading and writing of the class
      boost::shared_mutex mtx;

      std::string parent_frame;

      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_filter;

      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_raw;
  };

  // MoCapDriverBase Base class for the drivers of different motion capture 
  // system (e.g. vicon and qualisys)
  class MoCapDriverBase : public rclcpp::Node
  {
    public:
      // constructor
      MoCapDriverBase():  Node("vicon_driver"),
                          frame_rate(100),
                          model_list(std::vector<std::string>(0)),
                          publish_tf(false),
                          fixed_frame_id("mocap")
      {return;}

      // destructor
      ~MoCapDriverBase() {return;}

      // init Initialize the object & return True if successfully initialized
      virtual bool init() = 0;

      // run Start acquiring data from the server once
      virtual void run() = 0;

      // disconnect Disconnect to the server. The function is called automatically 
      // when the destructor is called.
      virtual void disconnect() = 0;

    private:
      // Disable the copy constructor and assign operator
      MoCapDriverBase(const MoCapDriverBase& );
      MoCapDriverBase& operator=(const MoCapDriverBase& );

    protected:
      // handleFrame handles a whole data package from the motion capture system
      virtual void handleFrame() = 0;

      // handSubject Handles a single subject (rigid body) from the motion capture system
      // sub_idx Index of the subject to be handled
      virtual void handleSubject(const int& sub_idx) = 0;

      // Address of the server
      std::string server_address;

      // Frame rate of the mocap system
      int frame_rate;

      // Rigid body to be tracked
      // Empty string if all the objects in the arena are to be tracked
      std::vector<std::string> model_list;

      // Observed rigid bodies (contains those lose tracking)
      std::map<std::string, Subject::SubjectPtr> subjects;

      bool publish_tf;
      std::string fixed_frame_id;
  };

}

#endif
