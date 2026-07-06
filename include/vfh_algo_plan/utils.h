#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <utility>

#include "vfh_algo_plan/histogram.hpp"

#define M_PI_F 3.14159265358979323846f
const float DEG_TO_RAD = M_PI_F / 180.f;
const float RAD_TO_DEG = 180.0f / M_PI_F;
// -----------------------------------------------------------------------------------------------------------
const float OCCUPANCY_THRESHOLD = 50.0; // user set costmap occupancy params (temporary testing setup for now)
// -----------------------------------------------------------------------------------------------------------

struct PolarPoint{
  PolarPoint(float z_, float r_) : z(z_), r(r_){};
  PolarPoint() : z(0.0f), r(0.0f){};
  float z;
  float r;
};

void wrapAngleToPlusMinusPi(float& angle_rad){
    angle_rad = std::fmod((angle_rad+M_PI_F), 2*M_PI_F);
    if(angle_rad < 0){
        angle_rad += 2*M_PI_F;
    }
    angle_rad = angle_rad - M_PI_F;
}

// binning azimuthal angles using this func mapping [0-(360/res-1)] --> [-180,180](in res of ALPHA_RES)
int polarToHistogramIndex(PolarPoint& polar_point, int res){
    float z = polar_point.z; // azimuth angle in ---> rads
    wrapAngleToPlusMinusPi(z); // wrap to +- pi here
    z = z*RAD_TO_DEG; // convert to degrees from RAD for binning (bin in degrees format for easiness also ALPHA_RES is in degrees) lets stick to degrees
    int z_ind = static_cast<int>(std::floor(z/res + 180.0f/res));
    // edge cases due to floating point errors
    if(z_ind > 360/res){z_ind = 360/res-1;}
    if(z_ind < 0){z_ind = 0;}
    return z_ind;
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
    PolarPoint polar_point(beta_xy, r);
    return polar_point;
}


#endif