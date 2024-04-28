
#include "hector_slam.h"

HectorSLAM::HectorSLAM(std::vector<Cell> *cells, SensorProbabilities sensorProbabilities):
 _p_cells(cells),_sensorProbabilities(sensorProbabilities) {
}

/*
* Calculate the difference between 2 values
*/
double HectorSLAM::DIFF(double a,double b) {
    return (a-b);
}

/*
* Check weather the given value is integer or not
*/
bool HectorSLAM::isInteger(double value) {
    return ceil(value) == floor(value);
}

/*
* Check and retrieve cell index from the cells array according to the x and y coordinates
*/
int HectorSLAM::findCellIndexByXAndY(int x, int y) {
  
    for (auto cell_index = 0; cell_index<_p_cells->size(); cell_index++) {
        if (((*_p_cells)[cell_index].x == x) && ((*_p_cells)[cell_index].y == y)) {
            return cell_index;
        }
    }
    return NO_CELL;
}

/*
* Calculate map value of given point
*/
double HectorSLAM::M_P(double x, double y){

    int x0 = floor(x);
    int x1 = ceil(x);
    int y0 = floor(y);
    int y1 = ceil(y);

    int cell_index_00 = findCellIndexByXAndY(x0, y0);
    double M_P_00 = (cell_index_00 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_00].prob_occupied;

    int cell_index_01 = findCellIndexByXAndY(x0, y1);
    double M_P_01 = (cell_index_01 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_01].prob_occupied;

    int cell_index_10 = findCellIndexByXAndY(x1, y0);
    double M_P_10 = (cell_index_10 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_10].prob_occupied;

    int cell_index_11 = findCellIndexByXAndY(x1, y1);
    double M_P_11 = (cell_index_11 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_11].prob_occupied;

    if((x0!=x1) && (y0!=y1)){
        return (DIFF(y,y0)/DIFF(y1,y0))*(( (DIFF(x,x0)/DIFF(x1,x0))*M_P_11) + ((DIFF(x1,x)/DIFF(x1,x0))*M_P_01) ) +
                (DIFF(y1,y)/DIFF(y1,y0))*( ((DIFF(x,x0)/DIFF(x1,x0))*M_P_10) + ((DIFF(x1,x)/DIFF(x1,x0))*M_P_00) ) ;
    }
    else if(x0!=x1 ){
        return (DIFF(x,x0)/DIFF(x1,x0))*M_P_10 + (DIFF(x1,x)/DIFF(x1,x0))*M_P_00;
    }
    else if(y0!=y1 ){
        return (DIFF(y,y0)/DIFF(y1,y0))*M_P_01 + (DIFF(y1,y)/DIFF(y1,y0))*M_P_00;
    }
    else{
        return M_P_00;
    }

}

/*
* Calculate X derivative value of given point
*/
double HectorSLAM::D_X(double x,double y){

    int x0 = floor(x);
    int x1 = ceil(x);
    int y0 = floor(y);
    int y1 = ceil(y);

    int cell_index_00 = findCellIndexByXAndY(x0, y0);
    double M_P_00 = (cell_index_00 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_00].prob_occupied;

    int cell_index_01 = findCellIndexByXAndY(x0, y1);
    double M_P_01 = (cell_index_01 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_01].prob_occupied;

    int cell_index_10 = findCellIndexByXAndY(x1, y0);
    double M_P_10 = (cell_index_10 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_10].prob_occupied;

    int cell_index_11 = findCellIndexByXAndY(x1, y1);
    double M_P_11 = (cell_index_11 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_11].prob_occupied;

    if(y0!=y1 && x0!=x1){
        return (DIFF(y,y0)/DIFF(y1,y0))*((M_P_11 - M_P_01 )/DIFF(x1,x0)) +
                (DIFF(y1,y)/DIFF(y1,y0))*((M_P_10 - M_P_00 )/DIFF(x1,x0)) ;
    }
    else if(x0!=x1){
        return (M_P_10 - M_P_00)/DIFF(x1,x0);
    }
    else{
        return 0;
    }

}

/*
* Calculate Y derivative value of given point
*/
double HectorSLAM::D_Y(double x,double y){
    int x0 = floor(x);
    int x1 = ceil(x);
    int y0 = floor(y);
    int y1 = ceil(y);

    int cell_index_00 = findCellIndexByXAndY(x0, y0);
    double M_P_00 = (cell_index_00 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_00].prob_occupied;

    int cell_index_01 = findCellIndexByXAndY(x0, y1);
    double M_P_01 = (cell_index_01 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_01].prob_occupied;

    int cell_index_10 = findCellIndexByXAndY(x1, y0);
    double M_P_10 = (cell_index_10 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_10].prob_occupied;

    int cell_index_11 = findCellIndexByXAndY(x1, y1);
    double M_P_11 = (cell_index_11 == -1) ? _sensorProbabilities.PROB_PRIOR_N : (*_p_cells)[cell_index_11].prob_occupied;

    if(y0!=y1 && x0!=x1){
        return (DIFF(x,x0)/DIFF(x1,x0))*(M_P_11 - M_P_10 ) +
            (DIFF(x1,x)/DIFF(x1,x0))*(M_P_01 - M_P_00 ) ;
    }
    else if(y0!=y1){
        return (M_P_01 - M_P_00)/DIFF(y1,y0);
    }
    else{
        return 0;
    }

}

/*
* Get position vector of the given point with respect to the world coordinate frame
*/
Eigen::Vector2d HectorSLAM::get_world_coordinates(Eigen::Vector3d robot_pos ,Eigen::Vector2d scan_point){

    Eigen::Matrix<double, 2, 2> rot_matrix;
    
    double cos_theta = std::cos(robot_pos(2));
    double sin_theta = std::sin(robot_pos(2));

    rot_matrix(0,0) = cos_theta;
    rot_matrix(0,1) = -sin_theta;
    rot_matrix(1,0) = sin_theta;
    rot_matrix(1,1) = cos_theta;

    /* coord = Rotational_matrix * scan_endpoint + robot_pos*/
    return ((rot_matrix*scan_point) + Eigen::Vector2d(robot_pos(0),robot_pos(1)));

}

/*
* Get gradient of map value of the position vector of the given point with respect to the world coordinate frame
*/
Eigen::Matrix<double,2, 3> HectorSLAM::D_map(double robot_angle ,Eigen::Vector2d scan_point){

    Eigen::Matrix<double,2, 3> world_coordinate_derivative_matrix;

    double cos_theta = std::cos(robot_angle);
    double sin_theta = std::sin(robot_angle);

    world_coordinate_derivative_matrix(0,0) = 1;
    world_coordinate_derivative_matrix(0,1) = 0;
    world_coordinate_derivative_matrix(0,2) = (-sin_theta*scan_point(0)-cos_theta*scan_point(1));
    world_coordinate_derivative_matrix(1,0) = 0;
    world_coordinate_derivative_matrix(1,1) = 1;
    world_coordinate_derivative_matrix(1,2) = (cos_theta*scan_point(0)-sin_theta*scan_point(1));

    return world_coordinate_derivative_matrix;

}

/*
* HECTOR SLAM algorithm for calculate mosts probable deviation from the previous point
*/
Eigen::Vector3d HectorSLAM::hector_slam(Eigen::Vector3d robot_pos , std::vector<Eigen::Vector2d> scan_endpoints){

    Eigen::Matrix<double, 3, 1> sum_of_scaled_jacobian_matrixes = Eigen::Matrix<double, 3, 1>::Zero();

    Eigen::Matrix<double, 3, 3> sum_of_hessian_matrixes = Eigen::Matrix<double, 3, 3>::Zero();

    for(auto scan_endpoint : scan_endpoints){

        /* M (S_i(ξ))]*/
        auto scan_world_coordinates = get_world_coordinates(robot_pos, scan_endpoint);

        /* ∇M(S_i(ξ)) */
        Eigen::Matrix<double, 1, 2> gradient_of_scan_world_coordinates(D_X(scan_world_coordinates(0),scan_world_coordinates(1)) , D_Y(scan_world_coordinates(0),scan_world_coordinates(1)));

        /* (∂S_i(ξ)/∂ξ)*/
        Eigen::Matrix<double,2, 3> gradient_of_map = D_map(robot_pos(2) ,scan_endpoint);

        /* ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ)*/
        Eigen::Matrix<double, 1, 3> jacobian_matrix = (gradient_of_scan_world_coordinates*gradient_of_map);

        // auto occupied_cell = find_occupied_cell_coordinates(0,0,scan_world_coordinates(0),scan_world_coordinates(1));
        // sum_of_scaled_jacobian_matrixes +=jacobian_matrix.transpose()* (1 - M_P(occupied_cell.first,occupied_cell.second));
        
        /* sigma { ( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )' * [1 − M(S_i(ξ))]} */
        sum_of_scaled_jacobian_matrixes +=jacobian_matrix.transpose()* (1 - M_P(scan_world_coordinates(0),scan_world_coordinates(1)));
        
        /* sigma {( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )' * ( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )}*/
        sum_of_hessian_matrixes  += jacobian_matrix.transpose()*jacobian_matrix;

    }

    return (sum_of_hessian_matrixes.inverse()*sum_of_scaled_jacobian_matrixes);

}


