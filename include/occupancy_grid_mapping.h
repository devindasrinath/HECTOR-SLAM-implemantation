#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include "hector_slam.h"
#include "common.h"

class OccupancyGridMap{

    public:
        OccupancyGridMap(std::vector<Cell> *cells,SensorProbabilities sensorProbabilities);
        void occupancy_grid_mapping();
        std::pair<int,int> find_occupied_cell_coordinates(double x0, double y0, double x1, double y1);
        void filter_detect_cells(double x0,double y0,double x1,double y1,std::vector<std::pair<int,int>> &points);
        std::vector<std::pair<int,int>> _cells_detected;
        std::pair<int,int> _occupied_cell={};

    private:
        void plotLineLow(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert);
        void plotLineHigh(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert);
        void plotLine(double x0, double y0, double x1, double y1, std::vector<std::pair<int,int>> &points);
        std::vector<std::pair<double,double>> _points_float;
        std::vector<Cell> *_p_cells;
        SensorProbabilities _sensorProbabilities;



};