#pragma once

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_map>

#define NO_CELL -1

struct Cell{
    int x;
    int y;
    double log_odds_ratio;
    double pre_log_odds_ratio;
    double prob_occupied;
};

struct SensorProbabilities{
    double PROB_OCCUPIED_N;
    double PROB_PRIOR_N;
    double PROB_FREE_N;
};

class HectorSLAM {


public:
    HectorSLAM(std::vector<Cell> *cells,SensorProbabilities sensorProbabilities);
    Eigen::Vector3d runLocalization(Eigen::Vector3d robot_pos , std::vector<Eigen::Vector2d> scan_endpoints);
    Eigen::Vector3d runLocalizationLoop(Eigen::Vector3d robot_pos_old, std::vector<Eigen::Vector2d> point_cloud, size_t num_iterations);

private:
    Eigen::Matrix<double,2, 3> D_map(double robot_angle ,Eigen::Vector2d scan_point);
    Eigen::Vector2d get_world_coordinates(Eigen::Vector3d robot_pos ,Eigen::Vector2d scan_point);
    
    double D_Y(double x,double y);
    double D_X(double x,double y);
    double M_P(double x,double y);
    int findCellIndexByXAndY(int x, int y);
    bool isInteger(double value);
    double DIFF(double a,double b);

    std::vector<Cell> *_p_cells;
    SensorProbabilities _sensorProbabilities;

};
