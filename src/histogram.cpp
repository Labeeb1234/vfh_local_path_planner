#include "vfh_algo_plan/histogram.hpp"


Histogram::Histogram(const int res): resolution_(res), z_dim_(360/res), dist_(z_dim_){
    setZero();
}

void Histogram::setZero(){
    dist_.fill(0.0f);
}

bool Histogram::isEmpty()const{
    int counter = 0;
    for(int i=0; i<z_dim_; i++){
        if(dist_(i) > FLT_MIN){
            counter++;
        }
    }
    return counter == 0;
}

// increase the resolution of data representation (polar histogram by 2) --> binning increasing
void Histogram::upsample(){
    if(resolution_ != 2*ALPHA_RES){
        throw std::logic_error("Invalid use of function upsample(). This function can only be used on a half resolution histogram.");
    }
    resolution_ = resolution_/2;
    z_dim_ = z_dim_*2;
    Eigen::VectorXf temp_dist(z_dim_); 
    for(int i=0; i<z_dim_; i++){
        int i_lowres = std::floor(i/2);
        temp_dist(i) = dist_(i_lowres);
    }
    dist_ = temp_dist;
}

// reduce histogram resolution by factor of 2
void Histogram::downsample(){
    if(resolution_ != 2*ALPHA_RES){
        throw std::logic_error("Invalid use of function downsample(). This function can only be used on a full resolution histogram.");
    }
    resolution_ = resolution_*2;
    z_dim_ = z_dim_/2;
    Eigen::VectorXf temp_dist(z_dim_);
    for(int i=0; i<z_dim_; i++){
        int i_highres = 2*i;
        temp_dist(i) = dist_.block(0, i_highres, 1, 2).mean();
    }
    dist_ = temp_dist;
}