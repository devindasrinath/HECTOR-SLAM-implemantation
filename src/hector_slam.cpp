
#include "hector_slam.h"
#include <chrono>

HectorSLAM::HectorSLAM(std::vector<Cell> *cells, SensorProbabilities sensorProbabilities, int width,int height):
 _p_cells(cells),_sensorProbabilities(sensorProbabilities), _width(width), _height(height) {
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
  
    auto index = (x-1) + (y-1)*_width;
    if ((index <=(_width*_height)) && ((*_p_cells)[index].x == x) && ((*_p_cells)[index].y == y)) {
        return index;
    }
    else{
        //throw std::runtime_error("given cell coordinates are not located in the gird");
        return NO_CELL;
    }
    
}

/*
* Check and return the given cell can be process
*/
bool HectorSLAM::validScanPoint(Eigen::Vector2d point) {
  
    int x0 = floor(point(0));
    int x1 = ceil(point(0));
    int y0 = floor(point(1));
    int y1 = ceil(point(1));

    if ((findCellIndexByXAndY(x0,y0) == NO_CELL)||
    (findCellIndexByXAndY(x0,y1) == NO_CELL)||
    (findCellIndexByXAndY(x1,y0) == NO_CELL)||
    (findCellIndexByXAndY(x1,y1) == NO_CELL)) {
        return false;
    }
    return true;

    
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

Eigen::Vector2d HectorSLAM::D_X_D_Y(double x,double y){

    double d_x =0;
    double d_y =0;

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
        d_x =  (DIFF(y,y0)/DIFF(y1,y0))*((M_P_11 - M_P_01 )/DIFF(x1,x0)) +
                (DIFF(y1,y)/DIFF(y1,y0))*((M_P_10 - M_P_00 )/DIFF(x1,x0)) ;
    }
    else if(x0!=x1){
        d_x =  (M_P_10 - M_P_00)/DIFF(x1,x0);
    }

    if(y0!=y1 && x0!=x1){
        d_y =  (DIFF(x,x0)/DIFF(x1,x0))*(M_P_11 - M_P_10 ) +
            (DIFF(x1,x)/DIFF(x1,x0))*(M_P_01 - M_P_00 ) ;
    }
    else if(y0!=y1){
        d_y = (M_P_01 - M_P_00)/DIFF(y1,y0);
    }

    return Eigen::Vector2d(d_x,d_y);

}
/*
* Get position vector of the given point with respect to the world coordinate frame
*/
Eigen::Vector2d HectorSLAM::get_world_coordinates(Eigen::Vector3d robot_pos ,Eigen::Vector2d scan_point){
    
    double cos_theta = std::cos(robot_pos(2));
    double sin_theta = std::sin(robot_pos(2));

    /* coord = Rotational_matrix * scan_endpoint + robot_pos*/
    return Eigen::Vector2d(cos_theta*scan_point(0) + sin_theta*scan_point(1) + robot_pos(0),
                           cos_theta*scan_point(1) - sin_theta*scan_point(0) + robot_pos(1));

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
    world_coordinate_derivative_matrix(0,2) = (-sin_theta*scan_point(0)+cos_theta*scan_point(1));
    world_coordinate_derivative_matrix(1,0) = 0;
    world_coordinate_derivative_matrix(1,1) = 1;
    world_coordinate_derivative_matrix(1,2) = (-cos_theta*scan_point(0)-sin_theta*scan_point(1));

    return world_coordinate_derivative_matrix;

}

/*
* Calculate Jacobian matrix
*/
Eigen::Vector3d HectorSLAM::Jac(Eigen::Matrix<double, 1, 2> map_gradient,Eigen::Matrix<double, 2, 3>scan_derivative){

    Eigen::Vector3d jac;

    jac(0) = map_gradient(0,0);
    jac(1) = map_gradient(0,1);
    jac(2) = map_gradient(0,0)*scan_derivative(0,2) + map_gradient(0,1)*scan_derivative(1,2);


    return jac;

}

/*
* Calculate Hessian matrix
*/
Eigen::Matrix<double, 3, 3> HectorSLAM::Hes(Eigen::RowVector3d jacobian_matrix){

    Eigen::Matrix<double, 3, 3> hes;

    hes(0,0) = jacobian_matrix(0) * jacobian_matrix(0);
    hes(1,1) = jacobian_matrix(1) * jacobian_matrix(1);
    hes(2,2) = jacobian_matrix(2) * jacobian_matrix(2);

    hes(0,1) = jacobian_matrix(0) * jacobian_matrix(1);
    hes(0,2) = jacobian_matrix(0) * jacobian_matrix(2);
    hes(1,2) = jacobian_matrix(1) * jacobian_matrix(2);

    hes(1,0) = hes(0,1);
    hes(2,0) = hes(0,2);
    hes(2,1) = hes(1,2);

    return hes;

}
/*
* HECTOR SLAM algorithm for calculate mosts probable deviation from the previous point
*/
Eigen::Vector3d HectorSLAM::runLocalization(Eigen::Vector3d robot_pos , std::vector<Eigen::Vector2d> scan_endpoints){

    Eigen::Matrix<double, 3, 1> sum_of_scaled_jacobian_matrixes = Eigen::Matrix<double, 3, 1>::Zero();

    Eigen::Matrix<double, 3, 3> sum_of_hessian_matrixes = Eigen::Matrix<double, 3, 3>::Zero();

    for(auto scan_endpoint : scan_endpoints){

        if((scan_endpoint(0)==0) && (scan_endpoint(1)==0)){
            continue;
        }
        //auto start1 = std::chrono::high_resolution_clock::now();
        /* M (S_i(ξ))]*/
        auto scan_world_coordinates = get_world_coordinates(robot_pos, scan_endpoint);

        if(!validScanPoint(scan_world_coordinates)){
            continue;
        }

        //auto start2 = std::chrono::high_resolution_clock::now();

        /* ∇M(S_i(ξ)) */
        Eigen::Matrix<double, 1, 2> gradient_of_scan_world_coordinates(D_X_D_Y(scan_world_coordinates(0),scan_world_coordinates(1)));

        //auto start3 = std::chrono::high_resolution_clock::now();
        /* (∂S_i(ξ)/∂ξ)*/
        Eigen::Matrix<double,2, 3> gradient_of_map = D_map(robot_pos(2) ,scan_endpoint);

        //auto start4 = std::chrono::high_resolution_clock::now();

        /* ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ)*/
        Eigen::Matrix<double, 1, 3> jacobian_matrix = Jac(gradient_of_scan_world_coordinates,gradient_of_map);

        //auto start5 = std::chrono::high_resolution_clock::now();
                
        /* sigma { ( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )' * [1 − M(S_i(ξ))]} */
        sum_of_scaled_jacobian_matrixes +=jacobian_matrix.transpose()* (1 - M_P(scan_world_coordinates(0),scan_world_coordinates(1)));

        //auto start6 = std::chrono::high_resolution_clock::now();
        
        /* sigma {( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )' * ( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )}*/
        sum_of_hessian_matrixes  += Hes(jacobian_matrix);

        //auto start7 = std::chrono::high_resolution_clock::now();


        // auto duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(start2 - start1).count();
        // auto duration2 = std::chrono::duration_cast<std::chrono::nanoseconds>(start3 - start2).count();
        // auto duration3 = std::chrono::duration_cast<std::chrono::nanoseconds>(start4 - start3).count();
        // auto duration4 = std::chrono::duration_cast<std::chrono::nanoseconds>(start5 - start4).count();
        // auto duration5 = std::chrono::duration_cast<std::chrono::nanoseconds>(start6 - start5).count();
        // auto duration6 = std::chrono::duration_cast<std::chrono::nanoseconds>(start7 - start6).count();

        // std::cout << "Execution time 1: " << duration1 << " ns" << std::endl;
        // std::cout << "Execution time 2: " << duration2 << " ns" << std::endl;
        // std::cout << "Execution time 3: " << duration3 << " ns" << std::endl;
        // std::cout << "Execution time 4: " << duration4 << " ns" << std::endl;
        // std::cout << "Execution time 5: " << duration5 << " ns" << std::endl;
        // std::cout << "Execution time 6: " << duration6 << " ns" << std::endl;
        // std::cout << std::endl;

    }

    return (sum_of_hessian_matrixes.inverse()*sum_of_scaled_jacobian_matrixes);

}

Eigen::Vector3d HectorSLAM::runLocalizationLoop(Eigen::Vector3d robot_pos_old, std::vector<Eigen::Vector2d> point_cloud, size_t num_iterations){
    
    Eigen::Vector3d total_change_value = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Vector3d robot_pos(robot_pos_old);

   for(size_t i=0;i<num_iterations;i++)
    {
        auto change_value = runLocalization(robot_pos, point_cloud);
        total_change_value+=change_value;
        robot_pos += change_value;

        // std::cout<<"\nchange_value : " <<total_change_value.transpose()<<std::endl;
        
        // Eigen::Vector3d new_pos_deg(robot_pos(0),robot_pos(1),robot_pos(2)*360/(2*M_PI));
        // std::cout<<"new_pos : " <<new_pos_deg.transpose()<<std::endl;
    }

    return robot_pos;

}


void HectorSLAM::getCompleteHessianDerivs(Eigen::Vector3d robot_pos_old, std::vector<Eigen::Vector2d> point_cloud, Eigen::Matrix3d& H, Eigen::Vector3d& dTr)
  {
    int size = point_cloud.size();

    Eigen::Affine2d transform(getTransformForState(robot_pos_old));

    float sinRot = sin(robot_pos_old[2]);
    float cosRot = cos(robot_pos_old[2]);

    H = Eigen::Matrix3d::Zero();
    dTr = Eigen::Vector3d::Zero();

    for (int i = 0; i < size; ++i) {

      const Eigen::Vector2d& currPoint(point_cloud[i]);

    auto transform_curr = transform * currPoint;
      Eigen::Vector3d transformedPointData(M_P(transform_curr(0),transform_curr(1)) , D_X(transform_curr(0),transform_curr(1)),D_Y(transform_curr(0),transform_curr(1)));

      float funVal = 1.0f - transformedPointData[0];

      dTr[0] += transformedPointData[1] * funVal;
      dTr[1] += transformedPointData[2] * funVal;

      float rotDeriv = ((-sinRot * currPoint.x() + cosRot * currPoint.y()) * transformedPointData[1] + (-cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);

      dTr[2] += rotDeriv * funVal;

      H(0, 0) += transformedPointData[1]*transformedPointData[1];
      H(1, 1) += transformedPointData[2]*transformedPointData[2];
      H(2, 2) += rotDeriv*rotDeriv;

      H(0, 1) += transformedPointData[1] * transformedPointData[2];
      H(0, 2) += transformedPointData[1] * rotDeriv;
      H(1, 2) += transformedPointData[2] * rotDeriv;
    }

    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);

  }

  Eigen::Affine2d HectorSLAM::getTransformForState(const Eigen::Vector3d& transVector) const
  {
    return Eigen::Translation2d(transVector[0], transVector[1]) * Eigen::Rotation2Dd(-transVector[2]);
  }

bool HectorSLAM::estimateTransformationLogLh(Eigen::Vector3d& estimate,  const std::vector<Eigen::Vector2d> point_cloud)
  {
    Eigen::Matrix3d H;
    H.Zero();
    Eigen::Vector3d dTr;
    dTr.Zero();
    getCompleteHessianDerivs(estimate, point_cloud, H, dTr);
    //std::cout << "\nH\n" << H  << "\n";
    //std::cout << "\ndTr\n" << dTr  << "\n";


    if ((H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {


      //H += Eigen::Matrix3d::Identity() * 1.0f;
      Eigen::Vector3d searchDir (H.inverse() * dTr);

      //std::cout << "\nsearchdir\n" << searchDir  << "\n";

      if (searchDir[2] > 0.2f) {
        searchDir[2] = 0.2f;
        std::cout << "SearchDir angle change too large\n";
      } else if (searchDir[2] < -0.2f) {
        searchDir[2] = -0.2f;
        std::cout << "SearchDir angle change too large\n";
      }

      updateEstimatedPose(estimate, searchDir);
      return true;
    }
    return false;
  }

  void HectorSLAM::updateEstimatedPose(Eigen::Vector3d& estimate, const Eigen::Vector3d& change)
  {
    estimate += change;
  }