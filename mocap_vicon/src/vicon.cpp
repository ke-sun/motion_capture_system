#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <mocap_vicon/ViconDriver.h>

using namespace std::chrono_literals;

class ViconNode : public rclcpp::Node
{
    public:
        ViconNode() : Node("vicon_node")
        {
            this->declare_parameter("frame_rate", 100);
            this->get_parameter("frame_rate", frame_rate);

            double time_interval = 1.0/static_cast<double>(frame_rate);
            time_interval = time_interval*1000;
            //int time = (int)time_interval;
            
            if(!driver.init())
            {
                RCLCPP_ERROR(this->get_logger(), "Initialization of the Vicon driver failed");
                exit(1);
            }
            RCLCPP_INFO(this->get_logger(), "Successfully initialize Vicon connection!");
            
	    while (rclcpp::ok())
	    {
		    driver.run();
		    //RCLCPP_INFO(this->get_logger(), "Running");
	    }

	    RCLCPP_INFO(this->get_logger(), "Shutting Down Vicon");
	    driver.disconnect();
	    
	    // timer = this->create_wall_timer(std::chrono::milliseconds(time), std::bind(&ViconNode::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            driver.run();
        }
        int frame_rate;
        mocap::ViconDriver driver;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("vicon node"), "Vicon Node Started ");
    auto vicon_node = std::make_shared<ViconNode>();
    
    rclcpp::spin(vicon_node);
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("vicon node"),"Vicon Node Shutting Down");

    return 0;
}
