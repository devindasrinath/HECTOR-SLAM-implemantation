#pragma once

#include <vector>
#include "math.h"
#include <eigen3/Eigen/Dense>

double LOGS_ODDS_RATIO(double x);
double PROB(double x);
double POINT_DIS_SQ(double x0,double y0,double x1,double y1);
int get_sign(double x);
std::vector<Eigen::Vector2d> scan_data_to_point_cloud(std::vector<double> generated_distance_data ,std::vector<double> generated_angle_data);