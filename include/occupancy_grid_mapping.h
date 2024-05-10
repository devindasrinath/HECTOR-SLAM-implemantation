#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include "hector_slam.h"
#include "common.h"
#include <eigen3/Eigen/Dense>

class OccupancyGridMap{

    public:
        OccupancyGridMap(SensorProbabilities sensorProbabilities, uint32_t width, uint32_t height);
        OccupancyGridMap()= default;;
        ~OccupancyGridMap();
        void occupancy_grid_mapping();
        std::pair<int,int> find_occupied_cell_coordinates(double x1, double y1);
        void filter_detect_cells(double x0,double y0,double x1,double y1,std::vector<std::pair<int,int>> &points);
        void runOccupancyGridMap(std::vector<double> distanceDataSet,std::vector<double> angleDataSet,Eigen::Vector3d robot_pos);
        std::vector<Cell>* get_cells();
        std::vector<std::pair<int,int>> _cells_detected;
        std::pair<int,int> _occupied_cell={};

    private:
        void plotLineLow(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert);
        void plotLineHigh(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert);
        void plotLine(double x0, double y0, double x1, double y1, std::vector<std::pair<int,int>> &points);
        void init_cells();
        std::vector<std::pair<double,double>> _points_float;
        std::vector<Cell> *_p_cells;
        SensorProbabilities _sensorProbabilities;
        uint32_t _width;
        uint32_t _height;
        double log_prob_occ ;
        double log_prob_free ;
        double log_prob_prior ;



};