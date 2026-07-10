#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "vfh_algo_plan/histogram.hpp"
#include "vfh_algo_plan/utils.h"


Eigen::MatrixXf costmapArrayToMatrix(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
/*
    histogram element func mij = cij^2(a-b*dij), dij->r (wrt VCP)
    for initial tests let: b = 1 and  a = ||<MAP_HEIGHT>-<MAP_WIDTH>||_2

*/
void generateNewPolarHistogram(Histogram& histogram, const Eigen::MatrixXf& local_costmap, const float map_resolution);


#endif