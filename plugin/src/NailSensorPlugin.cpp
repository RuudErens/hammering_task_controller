#include "NailSensorPlugin.h"
#include <rclcpp/utilities.hpp>


// Nail sensor plugin
// Add "Plugins: [NailSensorPlugin]" to the mc_rtc config file 
// Then open the results with the command 
// mc_log_ui /tmp/mc-control-CONTROLLER_NAME-lastest.bin
// Then search for nail -> force 

namespace mc_plugin
{

    NailSensorPlugin::~NailSensorPlugin()
    {
        std::cout << "Running NailSensorPlugin destructor" << std::endl;
        // if(_ros_thread.joinable()) 
        // {
        //     _running = false;
        //     _ros_thread.join();
        // }
    }


    void mc_plugin::NailSensorPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
    {
        _config.load(config);
        mc_rtc::log::info("NailSensorPlugin::init called with configuration: \n{}", config.dump(true, true));
  

        mc_rtc::log::success("NailSensorPlugin Running");
        auto nh = mc_rtc::ROSBridge::get_node_handle();
    
        // Not the cleanest but at leat mc_mujoco does not crash
        if(nh != nullptr)
        {
            _subForce = nh->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                "/nail_force_sensor", 
                1000,
                std::bind(&NailSensorPlugin::callback, this, std::placeholders::_1));
    
    
            if(_ros_thread.joinable()) 
            {
                _running = false;
                _ros_thread.join();
            }
            _running =  true;
            _ros_thread = std::thread([nh, this]
                                        {
                                            while (this->_running)
                                            {
                                                rclcpp::spin(nh);
                                            }
                                        }
                                        );
        
            mc_rtc::log::info("NailSensorPlugin::init waiting for force on nail head on topic /nail_force_sensor");
        }
        else
        {
            mc_rtc::log::error("Run roscore before running NailSensorPlugin");
        }
    
    }

    void NailSensorPlugin::reset(mc_control::MCGlobalController &controller)
    {
        
        mc_rtc::log::info("NailSensorPlugin::reset called");

        auto sensor_config = _config.find(controller.robot().module().name);
    
        if(sensor_config)
        {
            std::string name = (*sensor_config)("sensorName");
            std::string bodyname = (*sensor_config)("sensorBody");
            sva::PTransformd transform = (*sensor_config)("sensorTransform");
    
            _nailBodySensor = mc_rbdyn::BodySensor(name, bodyname, transform);
    

            // IT'S THIS LINE OF CODE THAT MAKES MUJOCO CRASH ON RESET ???
            std::cout << "Before addBodySensor" << std::endl;
            controller.controller().robot("nail").addBodySensor(_nailBodySensor);
            std::cout << "After addBodySensor" << std::endl;

        }
    
        controller.controller().logger().addLogEntry("nail_force_sensor", [this](){return _force;});

    }

    void NailSensorPlugin::before(mc_control::MCGlobalController &controller)
    {
    }

    void NailSensorPlugin::after(mc_control::MCGlobalController &controller)
    {
    }

    void NailSensorPlugin::callback(const std::shared_ptr<const geometry_msgs::msg::Vector3Stamped> &force){
        _force = {force->vector.x, force->vector.y, force->vector.z};
    }

    mc_control::GlobalPlugin::GlobalPluginConfiguration NailSensorPlugin::configuration()
    {
        mc_control::GlobalPlugin::GlobalPluginConfiguration out;
        out.should_run_before = true;
        out.should_run_after = true;
        out.should_always_run = true;
        return out;
    }

}

EXPORT_MC_RTC_PLUGIN("NailSensorPlugin", mc_plugin::NailSensorPlugin)