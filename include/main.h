#pragma once

#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <thread>




#define NUM_X_CELLS(GRID_STEP_SIZE) (GRID_WIDTH/GRID_STEP_SIZE)
#define NUM_Y_CELLS(GRID_STEP_SIZE) (GRID_HEIGHT/GRID_STEP_SIZE)
#define NUM_CELLS NUM_X_CELLS*NUM_Y_CELLS
#define PROB_OCCUPIED 0.9
#define PROB_PRIOR 0.5
#define PROB_FREE 0.1

#define NO_CELL -1
#define NUM_MAPS 3


void plotLine(double x0, double y0, double x1, double y1, std::vector<std::pair<int,int>> &points);
Eigen::Vector3d hector_slam(Eigen::Vector3d robot_pos , std::vector<Eigen::Vector2d> scan_endpoints);




