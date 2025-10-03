# pragma  once

#include <SpaceVecAlg/EigenTypedef.h>
#include <mc_control/GlobalPlugin.h>
#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/ros.h>
#include <mc_rbdyn/BodySensor.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp" //including ros2
#include <mc_rtc_ros/ros.h>
#include <thread>

namespace mc_plugin
{
    struct NailSensorPlugin : public mc_control::GlobalPlugin
    {
        void init(mc_control::MCGlobalController & controller,const mc_rtc::Configuration & config) override;

        void reset(mc_control::MCGlobalController & controller) override;

        void before(mc_control::MCGlobalController &) override;

        void after(mc_control::MCGlobalController & controller) override;

        mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

        ~NailSensorPlugin() override;

        void spin_node() const;
        void stop_thread();
        void start_thread();


        private:
        //     ros::Subscriber _subForce;
            rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _subForce;
            mc_rbdyn::BodySensor _nailBodySensor;

            void callback(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force);

            Eigen::Vector3d _force;

            std::thread _ros_thread;
            mc_rtc::NodeHandlePtr nh_ = nullptr;
            mc_rtc::Configuration _config;
            bool _running = true;
            bool _sensor_added = false;

    };
}