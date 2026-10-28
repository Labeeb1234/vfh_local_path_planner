#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <thread>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

// cell certainty grid is mostly considered as occupancy grid of the local costmap here for data representation purposes

class CCVDataNode: public rclcpp_lifecycle::LifecycleNode{
private:

    // publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ccv_grid_pub_;

    // timers and its params    
    rclcpp::CallbackGroup::SharedPtr timer_grp_;// timer_callbackgrp_definition for parallely running # STOPPED HERE
    rclcpp::TimerBase::SharedPtr timer1_, timer2_;

    // tf listener params
    uint64_t timer_period = 100; // in ms --> 10Hz
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // local costmap init objects params read from config file feed to the program via parameters
    rclcpp::CallbackGroup::SharedPtr costmap_callback_grp_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap2d_ros_;

    void getCostVal(nav2_costmap_2d::Costmap2D& costmap)const{
        std_msgs::msg::Float32MultiArray ccv_msg;
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock1(*(costmap.getMutex()));

        unsigned char* costmap_array = costmap.getCharMap(); // main master layer of costmap array
        int height = costmap.getSizeInCellsY();
        int width  = costmap.getSizeInCellsX();
        // adding metadata for ref
        ccv_msg.layout.dim.resize(2);
        ccv_msg.layout.dim[0].label = "height";
        ccv_msg.layout.dim[0].size = height;
        ccv_msg.layout.dim[0].stride = height*width;
        ccv_msg.layout.dim[1].label = "width";
        ccv_msg.layout.dim[1].size = width;
        ccv_msg.layout.dim[1].stride = width;
        ccv_msg.data.assign(costmap_array, costmap_array+height*width);
        ccv_grid_pub_->publish(ccv_msg);
    }

    void timer1Callback(){
        getCostVal(*costmap2d_ros_->getCostmap());
    }
    void timer2Callback(){
        RCLCPP_INFO(this->get_logger(), "timer-2 logging here(sanity check)");
    }



public:
    CCVDataNode(const rclcpp::NodeOptions& node_options): LifecycleNode("ccv_data_node", node_options){

        // QoS profile
        rclcpp::QoS custom_qos(rclcpp::KeepLast(10));
        custom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        custom_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        // Sensor Default QoS profile
        rclcpp::QoS sensor_qos{rclcpp::SensorDataQoS()};

        // init costmap2d_ros constructor
        costmap2d_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", this->get_namespace(), "local_costmap");
        RCLCPP_INFO(this->get_logger(), "Initializing Local Costmap.....!");
        RCLCPP_INFO(this->get_logger(), "current FSM state: %s", this->get_current_state().label().c_str());    
    }
    ~CCVDataNode(){}

    CallbackReturn on_configure(const rclcpp_lifecycle::State& prev_state){
        RCLCPP_INFO(this->get_logger(), "previous state: %s", prev_state.label().c_str());

        // create a check for TF(odom->base_footprint exist or not) using TF listeners just before configuring costmap2d_ros
        rclcpp::Duration tf_check_timeout = rclcpp::Duration::from_seconds(10.0);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        rclcpp::Time initial_transform_timeout_point = this->get_clock()->now() + tf_check_timeout;
        rclcpp::Rate r(2);
        while(rclcpp::ok() && !tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero)){
            if(this->get_clock()->now() > initial_transform_timeout_point){
                return CallbackReturn::FAILURE;
            }
            r.sleep(); // loop control rate
        }
        RCLCPP_INFO(this->get_logger(), "Required transforms exists!");

        // configure local costmap state to inactive state from unconfigured state
        const auto costmap2d_state = costmap2d_ros_->configure();
        RCLCPP_INFO(this->get_logger(), "current costmap state: %s", costmap2d_state.label().c_str());
        if(costmap2d_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
            return CallbackReturn::FAILURE;
        }

        // initialize required attributes of the node here
        ccv_grid_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/cell_certainty_grid/data", rclcpp::SystemDefaultsQoS()); // adding the publisher
        timer_grp_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        return CallbackReturn::SUCCESS;

    }
    
    CallbackReturn on_activate(const rclcpp_lifecycle::State& prev_state){
        RCLCPP_INFO(this->get_logger(), "\nprevious state: %s", prev_state.label().c_str());

        // activate costmap2d 
        const auto costmap2d_state = costmap2d_ros_->activate();
        RCLCPP_INFO(this->get_logger(), "current costmap state: %s", costmap2d_state.label().c_str());
        if(costmap2d_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
            return CallbackReturn::FAILURE;
        }

        timer1_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_period), 
            std::bind(&CCVDataNode::timer1Callback,this), 
            timer_grp_
        );
        // timer2_ = this->create_wall_timer(
        //     std::chrono::milliseconds(timer_period),
        //     std::bind(&CCVDataNode::timer2Callback, this)
        // );

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& prev_state){        
        RCLCPP_INFO(this->get_logger(), "previous state: %s", prev_state.label().c_str());
        RCLCPP_INFO(this->get_logger(), "Deactivating node....!");
        const auto costmap2d_state = costmap2d_ros_->deactivate();
        if(costmap2d_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully in [%s] state now!", costmap2d_state.label().c_str());
        timer1_.reset();
        tf_listener_.reset();
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_shutdown(){
        return CallbackReturn::SUCCESS;
    }
    CallbackReturn on_cleanup(){return CallbackReturn::SUCCESS;}
    
    
    
    CallbackReturn on_error(){return CallbackReturn::SUCCESS;}

};



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CCVDataNode>(rclcpp::NodeOptions());
    // Customizing via ExecutorOptions
    //rclcpp::executors::SingleThreadedExecutor executor{rclcpp::ExecutorOptions()};
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}