#include "common.h"
double LOGS_ODDS_RATIO(double x)  {
    return log(x/(1 - x));
}

double PROB(double x)  {
    return 1/(1+(1/std::pow(10, x)));
}

double POINT_DIS_SQ(double x0,double y0,double x1,double y1) {
    return std::pow(y1-y0 ,2) + std::pow(x1-x0 ,2) ;
}

int get_sign(double x){
    return int(x/abs(x));
}

std::vector<Eigen::Vector2d> scan_data_to_point_cloud(std::vector<double> generated_distance_data ,std::vector<double> generated_angle_data){

    std::vector<Eigen::Vector2d> point_cloud;

    for(size_t i = 0;i<generated_distance_data.size();i++)
    {
        point_cloud.emplace_back(Eigen::Vector2d(generated_distance_data[i]*sin(generated_angle_data[i]),generated_distance_data[i]*cos(generated_angle_data[i])));

    }

    return point_cloud;

}