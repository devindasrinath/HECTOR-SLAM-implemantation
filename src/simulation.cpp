#include "simulation.h"



double DatasetGenerator::solveQuadratic(double a, double b, double c) {
    static int i = 0;

    if(i>=_num_data){
        i = 0;
    }
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
     //   std::cout << "No real roots\n";
    } else if (discriminant == 0) {
        double root = -b / (2 * a);
        return root;
     //   std::cout << "One real root: " << root << "\n";
    } else {
        double root1 = (-b + sqrt(discriminant)) / (2 * a);
        double root2 = (-b - sqrt(discriminant)) / (2 * a);
        if(root1>0 && root2>0){
            if(i<(_num_data/2)){
                return root1;
            }
            else{
                return root2;
            }
        }
        else if (root1>0){
            return root1;
        }
        else if (root2>0){
            return root2;
        }
        
    
    }
    i++;
}


double DatasetGenerator::circle_inside_distance(std::pair<double,double> robot_pos,double beam_angle,double radius){

    auto a =1;
    auto b = 2 *(robot_pos.first * sin(beam_angle) + robot_pos.second * cos(beam_angle));
    auto c = robot_pos.first*robot_pos.first + robot_pos.second*robot_pos.second - radius*radius;

    return solveQuadratic(a,b,c);
}

std::vector<double> DatasetGenerator::getDistanceData(){
    
    return distanceData;
}

std::vector<double> DatasetGenerator::getAngleData(){

    return angleData;
}

void DatasetGenerator::generateData(std::pair<double,double> center){
    
    distanceData.clear();
    angleData.clear();
    
    for (size_t i = 0; i < _num_data; i++)
    {
        distanceData.emplace_back(circle_inside_distance(center,2*M_PI*i/_num_data,radius));//circle distances
        
    }

    for (size_t i = 0; i < _num_data; i++)
    {
        angleData.emplace_back(2*M_PI*i/_num_data);//circle angles
        
    }
}

