#pragma once


#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

class DatasetGenerator {
private:
    std::vector<double> data;
    int _num_data;
    double radius;

    double solveQuadratic(double a, double b, double c) ;

    double circle_inside_distance(std::pair<double, double> robot_pos, double beam_angle, double radius) ;

public:
    DatasetGenerator(int num_data, double radius) : _num_data(num_data), data(num_data), radius(radius) {}

    std::vector<double> generateData(std::pair<double, double> center);

};