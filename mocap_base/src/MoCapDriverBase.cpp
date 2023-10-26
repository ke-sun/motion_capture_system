#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <mocap_base/MoCapDriverBase.h>

using namespace std;
using namespace Eigen;

namespace mocap
{  
    // get name of subject
    const string& Subject::getName() 
    {
        boost::shared_lock<boost::shared_mutex> read_lock(mtx);
        return name;
    }

    
    // set name of subject
    void Subject::setName(const string& sub_name) 
    {
        boost::unique_lock<boost::shared_mutex> write_lock(mtx);
        name = sub_name;
    }

    
    // Enable or diable the subject
    const Subject::Status& Subject::getStatus() 
    {
        boost::shared_lock<boost::shared_mutex> read_lock(mtx);
        return status;
    }

    void Subject::enable() 
    {
        boost::unique_lock<boost::shared_mutex> write_lock(mtx);
        status = INITIALIZING;
    }

    void Subject::disable() 
    {
        boost::unique_lock<boost::shared_mutex> write_lock(mtx);
        kFilter.reset();
        status = LOST;
    }


    // get the state of subject
    const Quaterniond& Subject::getAttitude() 
    {
        boost::shared_lock<boost::shared_mutex> read_lock(mtx);
        return kFilter.attitude;
    }

    const Vector3d& Subject::getPosition() 
    {
        boost::shared_lock<boost::shared_mutex> read_lock(mtx);
        return kFilter.position;
    }

    const Vector3d& Subject::getAngularVel() 
    {
        boost::shared_lock<boost::shared_mutex> read_lock(mtx);
        return kFilter.angular_vel;
    }

    const Vector3d& Subject::getLinearVel() 
    {
        boost::shared_lock<boost::shared_mutex> read_lock(mtx);
        return kFilter.linear_vel;
    }


    // Set the noise parameter for the kalman filter
    bool Subject::setParameters(const Matrix<double, 12, 12>& u_cov,
                                const Matrix<double, 6, 6>& m_cov,
                                const int& freq) 
    {
        boost::unique_lock<boost::shared_mutex> write_lock(mtx);
        return kFilter.init(u_cov, m_cov, freq);
    }


    // Process the new measurement
    void Subject::processNewMeasurement(const double& time,
                                        const Quaterniond& m_attitude,
                                        const Vector3d& m_position) 
    {        
        boost::unique_lock<boost::shared_mutex> write_lock(mtx);

        // Publish raw data from mocap system
        auto pose_raw = geometry_msgs::msg::PoseStamped();
        rclcpp::Time time_stamp = rclcpp::Time(static_cast<int64_t>(time * 1e9));
        pose_raw.header.stamp = time_stamp;

        pose_raw.header.frame_id = parent_frame;

        pose_raw.pose.orientation.w = m_attitude.w();
        pose_raw.pose.orientation.x = m_attitude.x();
        pose_raw.pose.orientation.y = m_attitude.y();
        pose_raw.pose.orientation.z = m_attitude.z();

        pose_raw.pose.position.x = m_position(0);
        pose_raw.pose.position.y = m_position(1);
        pose_raw.pose.position.z = m_position(2);

        pub_raw->publish(pose_raw);
 
        if (!kFilter.isReady()) 
        {
            status = INITIALIZING;
            kFilter.prepareInitialCondition(time, m_attitude, m_position);
            // in KF file it is mentioned to use it twise once for pose and 
            // second time for twist
            return;
        }

        status = TRACKED;
        // Perfrom the kalman filter
        kFilter.prediction(time);
        kFilter.update(m_attitude, m_position);

        auto odom_filter = nav_msgs::msg::Odometry();

        odom_filter.header.stamp = time_stamp;
        odom_filter.header.frame_id = parent_frame;
        odom_filter.child_frame_id = name + "/base_link";

        odom_filter.pose.pose.orientation.w = kFilter.attitude.w();
        odom_filter.pose.pose.orientation.x = kFilter.attitude.x();
        odom_filter.pose.pose.orientation.y = kFilter.attitude.y();
        odom_filter.pose.pose.orientation.z = kFilter.attitude.z();

        odom_filter.pose.pose.position.x = kFilter.position(0);
        odom_filter.pose.pose.position.y = kFilter.position(1);
        odom_filter.pose.pose.position.z = kFilter.position(2);

        odom_filter.twist.twist.angular.x = kFilter.angular_vel(0);
        odom_filter.twist.twist.angular.y = kFilter.angular_vel(1);
        odom_filter.twist.twist.angular.z = kFilter.angular_vel(2);

        odom_filter.twist.twist.linear.x = kFilter.linear_vel(0);
        odom_filter.twist.twist.linear.y = kFilter.linear_vel(1);
        odom_filter.twist.twist.linear.z = kFilter.linear_vel(2);


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

}