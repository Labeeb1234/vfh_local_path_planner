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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/executors.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_array.hpp"


#include "vfh_algo_plan/local_planner.hpp"
#include "vfh_algo_plan/histogram.hpp"
#include "vfh_algo_plan/utils.h"


// EL-Plan : run the costmapviewer lifecycle node in main thread and run costmap_lifecycle node in a parallel different thread (and check)

class LocalCostmapTestViewer: public rclcpp_lifecycle::LifecycleNode{
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cartesian_aw_pub_; // for publishing cartesian coords of the active window cells
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr polar_aw_pub_; // for publishing the 2D polar coordinates of the active window cells
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr histogram_pub_; // for publishing the 1D Histogram Grid which represents the local env model

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap2d_ros_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /*
        timer funcs
    */
   uint16_t debug_period_ = 3000; // in ms
   rclcpp::TimerBase::SharedPtr debug_timer_;
   rclcpp::CallbackGroup::SharedPtr costmap_group_;
    
    /*
        some helper attributes
    */
    float MAP_RESOLUTION = 0.05; // [m/cell]
    std::unique_ptr<nav2_util::NodeThread> costmap2d_ros_thread_{nullptr};
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_msg_;
    rclcpp::Time last_costmap_update_{0, 0};
    std::mutex costmap2d_mutex_;
    
    

    /*
        1D-Polar histogram based attributes
    */
    Histogram polar_histogram_ = Histogram(ALPHA_RES);




    /*
        some helper methods
    */
    /* unused will use later on */
    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        std::lock_guard<std::mutex> lock_guard(costmap2d_mutex_);
        latest_costmap_msg_ = msg;
        last_costmap_update_ = msg->header.stamp;
    }

    void modelLocalEnv(){
        std::lock_guard<std::mutex> lock_guard(costmap2d_mutex_);
        // get costmap msg
        if(!latest_costmap_msg_){return;}
        // get costmap res
        if(!latest_costmap_msg_->info.resolution){
            MAP_RESOLUTION = MAP_RESOLUTION;
        }else{
            MAP_RESOLUTION = latest_costmap_msg_->info.resolution;
        }
        // convert to matrix
        Eigen::MatrixXf costmap_matrix = costmapArrayToMatrix(latest_costmap_msg_);
        
        // publish data
        geometry_msgs::msg::PoseArray cartesian_msg;
        cartesian_msg.poses.resize(latest_costmap_msg_->info.width*latest_costmap_msg_->info.height);
        geometry_msgs::msg::PoseArray polar_msg;
        polar_msg.poses.resize(latest_costmap_msg_->info.width*latest_costmap_msg_->info.height);

        cartesian_msg.header.stamp = this->get_clock()->now();
        cartesian_msg.header.frame_id = "base_footprint";
        polar_msg.header.stamp = this->get_clock()->now();
        polar_msg.header.frame_id = "base_footprint";

        for(int row=0; row<static_cast<int>(latest_costmap_msg_->info.height); ++row){
            for(int col=0; col<static_cast<int>(latest_costmap_msg_->info.width); ++col){
                if(costmap_matrix(row, col) < OCCUPANCY_THRESHOLD){continue;}
                // convert costmap_matrix pixels to cartesian coordinates based on our frame axis convention (ENU) (axis issue needs to be looked at)
                // TODO:= record orientation of bot if sensor angular range is less than 360 degrees  
                Eigen::Vector2f cell_position = costmapMatrixToCartesianMap(
                    row, col, MAP_RESOLUTION, 
                    static_cast<int>(latest_costmap_msg_->info.height), static_cast<int>(latest_costmap_msg_->info.width)
                ); // pos wrt bot centre pose
                // convert obstacle pose to polar form (using local costmap orientation if the sensor angular range is less than 360 degrees)
                PolarPoint polar_point = cartesianToPolarPoint(cell_position);
                // wrap polar obstacle vector angle between +-pi
                wrapAngleToPlusMinusPi(polar_point.z);
                int idx = row*latest_costmap_msg_->info.width+col;
                cartesian_msg.poses[idx].position.x = cell_position.x();
                cartesian_msg.poses[idx].position.y = cell_position.y();
                cartesian_msg.poses[idx].position.z = 0.0;
                polar_msg.poses[idx].position.x = polar_point.r;
                polar_msg.poses[idx].position.y = polar_point.z*RAD_TO_DEG; // convert to degrees
                polar_msg.poses[idx].position.z = 0.0;
            }
        }

        // reduce data rep further to polar histogram (that is out Model Representation in probabilitic mode)
        generateNewPolarHistogram(polar_histogram_, costmap_matrix, MAP_RESOLUTION); // descretization of input sensor data for local env representation for planning
        // publish data
        geometry_msgs::msg::PoseArray histogram_msg;
        histogram_msg.poses.resize(GRID_LENGTH_Z);
        histogram_msg.header.stamp = this->get_clock()->now();
        histogram_msg.header.frame_id = "base_footprint";
        for(int k=0; k<GRID_LENGTH_Z; ++k){
            float hist_val = polar_histogram_.get_dist(k);
            histogram_msg.poses[k].position.x = static_cast<float>(k);
            histogram_msg.poses[k].position.y = hist_val;
            histogram_msg.poses[k].position.z = 0.0;
        }
        this->cartesian_aw_pub_->publish(cartesian_msg);
        this->polar_aw_pub_->publish(polar_msg);
        this->histogram_pub_->publish(histogram_msg);
    }



public:
    LocalCostmapTestViewer(const rclcpp::NodeOptions& node_options): 
        LifecycleNode("local_costmap_viewer_node", node_options)
    {
        // callback_group settings
        costmap_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        // declare parameters for costmap node (if any)
        costmap2d_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "local_costmap",
            std::string{get_namespace()},
            "local_costmap"
        );
    }

    ~LocalCostmapTestViewer(){
        costmap2d_ros_thread_.reset();
    }


    CallbackReturn on_configure(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Configuring node....");
        costmap2d_ros_->configure(); // configuring costmap 

        // start a separate thread to run costmap2d_ros lifecycle node
        costmap2d_ros_thread_ = std::make_unique<nav2_util::NodeThread>(costmap2d_ros_);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Activating node...");
        
        const auto costmap_state = costmap2d_ros_->activate();        
        if(costmap_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
            return CallbackReturn::FAILURE;
        }

        // after the costmap2d is in active state create a sub node to local costmap topic
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = costmap_group_;
        local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "local_costmap/costmap",
            rclcpp::QoS(10),
            std::bind(&LocalCostmapTestViewer::localCostmapCallback, this, std::placeholders::_1),
            sub_options
        ); // replace with costmap apis instead of creating a subscription accessing the costmaps (later-on)
        
        
        // ----------------------------------- FOR DEBUGGING & VIS ----------------------------------
        // create publishers
        cartesian_aw_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "local_costmap/active_window/pos",
            rclcpp::QoS(10)
        );
        polar_aw_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "local_costmap/active_window/polar",
            rclcpp::QoS(10)
        );
        histogram_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "local_costmap/polar_histogram_grid",
            rclcpp::QoS(10)
        );
        // timer funcs construction
        debug_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(debug_period_),
            std::bind(&LocalCostmapTestViewer::debugTimerCallback, this),
            costmap_group_
        );
        // -------------------------------------------------------------------------------------------

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Deactivating node...");
        costmap2d_ros_->deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Cleaning up node...");
        costmap2d_ros_->cleanup();
        costmap2d_ros_thread_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Shuting Down node...");
        return CallbackReturn::SUCCESS;
    }


    void debugTimerCallback(){
        this->modelLocalEnv();
    }

    void testThreadFunction(){
        // configure tf listener node
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
        //     *tf_buffer_,
        //     this->get_node_base_interface(),
        //     this->get_node_logging_interface(),
        //     this->get_node_clock_interface(),
        //     this->get_node_topics_interface(),
        //     false   // don't spin a dedicated thread just for tf_listener
        // ); // --> for newer distro
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_,
            this,
            false   // don't spin a dedicated thread just for tf_listener
        );
        RCLCPP_INFO(this->get_logger(), "setting up transform listener for this lifecycle node....");
    }



};



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalCostmapTestViewer>(rclcpp::NodeOptions());
    // rclcpp::executors::SingleThreadedExecutor executor;    
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2
    );
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}