#pragma once

#include <vector>
#include "math.h"
#include <eigen3/Eigen/Dense>

double LOGS_ODDS_RATIO(double x);
double PROB(double x);
double POINT_DIS_SQ(double x0,double y0,double x1,double y1);
int get_sign(double x);

template <typename T, size_t N>
void scan_data_to_point_cloud(std::array<T,N> generated_distance_data ,std::array<T,N> generated_angle_data,std::array<Eigen::Vector2d,N> &point_cloud);

template <typename T, size_t N>
void scan_data_to_point_cloud(std::array<T,N> generated_distance_data ,std::array<T,N> generated_angle_data,std::array<Eigen::Vector2d,N> &point_cloud){

    for(size_t i = 0;i<N;i++)
    {
        point_cloud[i] = (Eigen::Vector2d(generated_distance_data[i]*sin(generated_angle_data[i]),generated_distance_data[i]*cos(generated_angle_data[i])));

    }


}