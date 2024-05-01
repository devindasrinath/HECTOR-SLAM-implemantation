#pragma once


#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>


template <size_t N>
class DatasetGenerator {
private:
    std::array<double,N> distanceData;
    std::array<double,N> angleData;
    size_t _num_data;
    double radius;

    double solveQuadratic(double a, double b, double c);
    double circle_inside_distance(std::pair<double, double> robot_pos, double beam_angle, double radius);

public:
    DatasetGenerator(double radius) : _num_data(N), radius(radius) {}
    std::array<double,N> getDistanceData();
    std::array<double,N> getAngleData();
    void generateData(std::pair<double,double> center, double angle_deg);
};