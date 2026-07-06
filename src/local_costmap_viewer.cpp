#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <limits>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
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

#include "vfh_algo_plan/histogram.hpp"
#include "vfh_algo_plan/utils.h"


class LocalCostmapTestViewer: public rclcpp_lifecycle::LifecycleNode{
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr local_costmap_matrix_image_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap2d_ros_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    /*
        some helper attributes
    */
    float MAP_RESOLUTION = 0.05; // [m/cell]
    nav2_costmap_2d::Costmap2D* local_costmap; // unused as of now
    
    /*
        1D-Polar histogram based attributes
    */
    Histogram polar_histogram_ = Histogram(ALPHA_RES);
    std::vector<uint8_t> histogram_image_data_;


    /*
        temp
    */
   rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
   std::vector<uint8_t> img_data_;


    /*
        some helper methods
    */
    void printCostmapMatrix(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(),
            "local costmap properties\n:" 
            "  Size (height, width): (%u, %u)\n"
            "  Resolution: %.3f m/cell\n"
            "  Origin: (%.3f, %.3f, %.3f)",
            msg->info.height, msg->info.width, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y, msg->info.origin.position.z
        );
        
        // print/display costmap matrix
        // Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> costmap_matrix(
        //     msg->data.data(), msg->info.height, msg->info.width
        // ); // alternative to keep in mind
        if(msg->info.resolution){
            MAP_RESOLUTION = msg->info.resolution;
        }else{
            MAP_RESOLUTION = MAP_RESOLUTION;
        }
        Eigen::MatrixXf costmap_matrix = costmapArrayToMatrix(msg);
        // test conversion funcs based obstacle threshold probability
        // for(int row=0; row<msg->info.height; ++row){
        //     for(int col=0; col<msg->info.width; ++col){
        //         if(static_cast<uint8_t>(costmap_matrix(row, col)) < OCCUPANCY_THRESHOLD){continue;}
        //         Eigen::Vector2f dpos = costmapMatrixToCartesianMap(row, col, MAP_RESOLUTION, msg->info.height, msg->info.width);
        //         PolarPoint pol_point = cartesianToPolarPoint(dpos);
        //         RCLCPP_INFO(this->get_logger(), "Cell(%d,%d) cartesian pos(x,y): [%f, %f]", row, col, dpos.x(), dpos.y());
        //         RCLCPP_INFO(this->get_logger(), "Cell(%d, %d), polar point(r,theta_deg): [%f, %f]", row, col, pol_point.r, pol_point.z*RAD_TO_DEG);
        //     }
        // }

        // histogram generation pipeline
        





        // for(int y=0; y<costmap_matrix.rows(); ++y){
        //     for(int x=0; x<costmap_matrix.cols(); ++x){
        //         // print obstacle distance vectors if within a certain min obstacle probabily density
        //         float cell_certainty = costmap_matrix(y,x);
        //         if(cell_certainty > OCCUPANCY_THRESHOLD){
        //             // convert coords to world frame wrt
        //             // convert the (x,y) cell coords to m scale wrt VCP (local_costmap already in VCP) // the costmap is always in square shape (ws,ws) as per requirement as well as nav2 convention
        //             float dx = (x-(static_cast<int>(msg->info.width)/2))*msg->info.resolution;
        //             float dy = ((static_cast<int>(msg->info.height)/2)-y)*msg->info.resolution;
        //             float beta_yx = std::atan2(dy, dx); // obstacle distance vector (angle in rads)
        //             Eigen::Vector2f cell_vec(1,2);
        //             cell_vec << dx, dy;
        //             // RCLCPP_INFO(this->get_logger(), "Cell Obstacle Distance Vector: [%f, %f, %f]", cell_vec.norm(), beta_yx*RAD_TO_DEG, cell_certainty);
        //             // testing the data(sensor data/costmap obstacle cell data representation derived from 2d lidar scans) descretization logic/flow
        //             // generate histogram which causes the descretization
        //             float r = cell_vec.norm();
        //             PolarPoint pol_point(beta_yx, r);
        //             int pol_ind = polarToHistogramIndex(pol_point, ALPHA_RES); // binned azimuthal angle
        //             RCLCPP_INFO(this->get_logger(), "(Binned) Cell Obstacle Distance Vector: [%f, %d]", cell_vec.norm(), pol_ind);
        //             // out_ << "(Binned) Cell Obstacle Distance Vector: " << cell_vec.norm() << ", " << pol_ind << "\n";
        //         }
        //     }
        // }

    }

    Eigen::MatrixXf costmapArrayToMatrix(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        Eigen::MatrixXf costmap_matrix(msg->info.height, msg->info.width); // costmap_matrix axis set origin at (0,0) index btw
        for(int row=0; row<static_cast<int>(msg->info.height); ++row){
            for(int col=0; col<static_cast<int>(msg->info.width); ++col){
                costmap_matrix(row,col)=static_cast<float>(msg->data[row*msg->info.width+col]);
            }
        }
        return costmap_matrix;
    }

    // converting units/sub-frame from costmap-matrix to world units [m]
    Eigen::Vector2f costmapMatrixToCartesianMap(int mx, int my, const float map_res, const int map_height, const int map_width){
        std::pair<float, float> cartesian_coords(0.0f, 0.0f);
        // aligning map matrix (0,0) index to the map's bottom left along ENU-convention
        float x = (mx-(map_height/2.0f))*map_res;
        float y = (my-(map_width/2.0f))*map_res;
        Eigen::Vector2f p(1,2);
        p.x() = x;
        p.y() = y;
        return p;
    }
    // converting cartesian coords to polar coords
    PolarPoint cartesianToPolarPoint(const Eigen::Vector2f& p){
        // p contains 2d vector{x,y} pos values of cells in cartesian convention which is wrt the VCP/bot centre
        double r = p.norm(); // in [m]
        double beta_xy = std::atan2(p.y(), p.x()); // in [rads] this will be wrapped between +-pi but we can make sure this happens in pipelines downstream just in case
        PolarPoint polar_point(beta_xy, r); // float (double to float precision loss happens here probably)
        return polar_point;
    }


public:
    LocalCostmapTestViewer(const rclcpp::NodeOptions& node_options): 
        LifecycleNode("local_costmap_viewer_node", node_options)
    {
        // declare parameters for costmap node (if any)
        costmap2d_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "local_costmap",
            std::string{get_namespace()},
            "local_costmap"
        );
    }

    ~LocalCostmapTestViewer(){}


    CallbackReturn on_configure(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Configuring node....");
        
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
        costmap2d_ros_->configure(); // configuring costmap
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Activating node...");

        const auto costmap_state = costmap2d_ros_->activate();
        if(costmap_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
            return CallbackReturn::FAILURE;
        }

        // nav2_costmap_2d::Costmap2D* local_costmap = costmap2d_ros_->getCostmap();
        // unsigned char* costmap_ptr = local_costmap->getCharMap();
        // after the costmap2d is in active state create a sub node to local costmap topic
        local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "local_costmap/costmap",
            rclcpp::QoS(10),
            std::bind(&LocalCostmapTestViewer::printCostmapMatrix, this, std::placeholders::_1)
        ); // replace with costmap apis instead of creating a subscription accessing the costmaps (later-on)

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
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state){
        RCLCPP_INFO(this->get_logger(), "State ID: [%d]", state.id());
        RCLCPP_INFO(this->get_logger(), "Shuting Down node...");
        costmap2d_ros_->shutdown();
        return CallbackReturn::SUCCESS;
    }
};



int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalCostmapTestViewer>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}