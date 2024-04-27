#pragma once

#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>



#define NUM_X_CELLS GRID_WIDTH/GRID_STEP_SIZE
#define NUM_Y_CELLS GRID_HEIGTH/GRID_STEP_SIZE
#define NUM_CELLS NUM_X_CELLS*NUM_Y_CELLS
#define PROB_OCCUPIED 0.9
#define PROB_PRIOR 0.5
#define PROB_FREE 0.1

