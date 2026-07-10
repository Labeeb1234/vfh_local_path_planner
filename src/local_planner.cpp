#include "vfh_algo_plan/local_planner.hpp"

#include <iostream>

Eigen::MatrixXf costmapArrayToMatrix(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    int map_width = static_cast<int>(msg->info.width);
    int map_height = static_cast<int>(msg->info.height);
    Eigen::MatrixXf costmap_matrix(map_height, map_width);
    costmap_matrix.fill(0.0f);
    // using the std ROS ENU conventions and data array starting pixel
    for(int row=0; row<map_height; ++row){
        for(int col=0; col<map_width; ++col){
            costmap_matrix(row,col) = static_cast<float>(msg->data[row*map_width+col]); // should I normalize to { [0,1] and -1 } from { [0,100] and -1 }
        }
    }
    return costmap_matrix;
}


/*
    histogram element func mij = cij^2(a-b*dij), dij->r (wrt VCP)
    for initial tests let: b = 1 and  a = ||<MAP_HEIGHT>-<MAP_WIDTH>||_2
    H(k) = summation(mij){i,j [of kth sector]}

*/
void generateNewPolarHistogram(Histogram& histogram, const Eigen::MatrixXf& local_costmap, const float map_resolution){
    // generating 1D Polar Histogram of resolution ALPHA_RES on the azimuthal angles
    // also the azimuthal angle descretization based on ALPHA_RES takes place here and make sure the angles are wrapped around +-pi and then convert to DEG from RAD units
    const int MAP_HEIGHT = local_costmap.rows(); const int MAP_WIDTH = local_costmap.cols();
    float a = std::sqrt(MAP_HEIGHT*MAP_HEIGHT + MAP_WIDTH*MAP_WIDTH)/2;
    Eigen::VectorXi counter(GRID_LENGTH_Z);
    counter.fill(0);
    for(int row=0; row<MAP_HEIGHT; row++){
        for(int col=0; col<MAP_WIDTH; col++){
            if(static_cast<uint8_t>(local_costmap(row,col)) < OCCUPANCY_THRESHOLD){continue;} // ignore non-threat cells in the active window (maybe offload to a pre-processing func later on)
            Eigen::Vector2f cell_position = costmapMatrixToCartesianMap(row, col, map_resolution, MAP_HEIGHT, MAP_WIDTH); // wrt VCP 
            PolarPoint polar_point = cartesianToPolarPoint(cell_position);
            int polar_ind = polarToHistogramIndex(polar_point, ALPHA_RES);            
            float mag_xy = (local_costmap(row,col)*local_costmap(row,col))*(a-polar_point.r); // mji-->magnitude of obstacle distance vector func 
            histogram.set_dist(polar_ind, histogram.get_dist(polar_ind)+mag_xy);
            counter(polar_ind)++;
        }
    }
    for(int i=0; i<GRID_LENGTH_Z; ++i){
        if(counter(i) > 0){
            histogram.set_dist(i, histogram.get_dist(i)/counter(i)); // should I normalize using mean (i already have the count of data points of each sector plus the sigma)
        }else{
            histogram.set_dist(i, 0.0f);
        }
    }
}


// smoothing func for polar histogram func