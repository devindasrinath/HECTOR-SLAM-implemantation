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

