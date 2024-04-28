#include "occupancy_grid_mapping.h"

OccupancyGridMap::OccupancyGridMap(std::vector<Cell> *cells, SensorProbabilities sensorProbabilities):
 _p_cells(cells),_sensorProbabilities(sensorProbabilities) 
{

}

void OccupancyGridMap::occupancy_grid_mapping(){
    for(auto &cell:(*_p_cells)){
        auto it  = std::find(_cells_detected.begin(), _cells_detected.end(), std::make_pair(cell.x,cell.y));
        if(it != _cells_detected.end()){

            if((cell.x == _occupied_cell.first) && (cell.y == _occupied_cell.second)){
                /* CASE 1 : cell occupied*/
                cell.log_odds_ratio = LOGS_ODDS_RATIO(_sensorProbabilities.PROB_OCCUPIED_N) + cell.pre_log_odds_ratio - LOGS_ODDS_RATIO(_sensorProbabilities.PROB_PRIOR_N);
                //std::cout<<cell.x<<" , " <<cell.y<< " , "<<"occupied , "<<cell.log_odds_ratio<<" , "<<PROB(cell.log_odds_ratio)<< " , "<<cell.pre_log_odds_ratio<<std::endl;
                
            }
            else{
                /* CASE 2 : cell FREE*/
                cell.log_odds_ratio = LOGS_ODDS_RATIO(_sensorProbabilities.PROB_FREE_N) + cell.pre_log_odds_ratio - LOGS_ODDS_RATIO(_sensorProbabilities.PROB_PRIOR_N);
                //std::cout<<cell.x<<" , " <<cell.y<< " , "<<"free , "<<cell.log_odds_ratio<<" , "<<PROB(cell.log_odds_ratio)<< " , "<<cell.pre_log_odds_ratio<<std::endl;
            }
            
        }
        else{
            /* CASE 3 : cell uknown*/
            cell.log_odds_ratio = cell.pre_log_odds_ratio;
           // std::cout<<cell.x<<" , " <<cell.y<< " , "<<"dont know , "<<cell.log_odds_ratio<<" , "<<PROB(cell.log_odds_ratio)<< " , "<<cell.pre_log_odds_ratio<<std::endl;

           // std::cout<<"came to unknown cell"<<std::endl;;
        }
    
        
        cell.prob_occupied = PROB(cell.log_odds_ratio);

        // if(came)
        //     std::cout<<cell.x<<" , " <<cell.y<< " , "<<"occupied , "<<cell.log_odds_ratio<<" , "<<cell.prob_occupied<< " , "<<cell.pre_log_odds_ratio<<std::endl;

        cell.pre_log_odds_ratio = cell.log_odds_ratio;
        
    }
}

void OccupancyGridMap::filter_detect_cells(double x0,double y0,double x1,double y1,std::vector<std::pair<int,int>> &points){
    plotLine(x0, y0, x1, y1, points);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyGridMap::plotLineLow(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert = false) {
    double x1,y1=0;
    if(invert){
        y1 = y1_1;
        x1 = x0 + abs(abs(x0) - abs(x1_1));
    }
    else{
        x1 = x1_1;
        y1 = y1_1;
    }

    double dx = x1 - x0;
    double dy = y1 - y0;
    int yi = 1;
    if (dy < 0) {
        yi = -1;
        dy = -dy;
    }
    double D = (2 * dy) - dx;
    double y = y0;

    for (double x = x0; x < x1; x+=1) {
        if(invert){
            points.emplace_back(std::make_pair(round(x0 - (abs(x-x0))),round(y)));
            _points_float.emplace_back(std::make_pair(x0 - (abs(x-x0)),y));
        }
        else{
            points.emplace_back(std::make_pair(round(x),round(y)));
            _points_float.emplace_back(std::make_pair(x,y));
        }
        if (D > 0) {
            y = y + yi;
            D = D + (2 * (dy - dx));
        } else {
            D = D + 2 * dy;
        }
    }
}

void OccupancyGridMap::plotLineHigh(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert = false) {

double x1,y1=0;
    if(invert){
        x1 = x1_1;
        y1 = y0 + abs(abs(y0) - abs(y1_1));
    }
    else{
        x1 = x1_1;
        y1 = y1_1;
    }
    double dx = x1 - x0;
    double dy = y1 - y0;
    double xi = 1;
    if (dx < 0) {
        xi = -1;
        dx = -dx;
    }
    double D = (2 * dx) - dy;
    double x = x0;

    for (double y = y0; y <= y1; y+=1) {
        if(invert){
            points.emplace_back(std::make_pair(round(x),round(y0 - (abs(y-y0)))));
            _points_float.emplace_back(std::make_pair(x,y0 - (abs(y-y0))));
        }
        else{
            points.emplace_back(std::make_pair(round(x),round(y)));
            _points_float.emplace_back(std::make_pair(x,y));
        }
        if (D > 0) {
            x = x + xi;
            D = D + (2 * (dx - dy));
        } else {
            D = D + 2 * dx;
        }
    }
}

void OccupancyGridMap::plotLine(double x0, double y0, double x1, double y1, std::vector<std::pair<int,int>> &points) {
    if (std::abs(y1 - y0) < std::abs(x1 - x0)) {
        if (x0 > x1) {
            plotLineLow(x0, y0, x1, y1, points,true);
        } else {
            plotLineLow(x0, y0, x1, y1, points);
        }
    } else {
        if (y0 > y1) {
            plotLineHigh(x0, y0, x1, y1, points,true);
        } else {
            plotLineHigh(x0, y0, x1, y1, points);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<int,int> OccupancyGridMap::find_occupied_cell_coordinates(double x0, double y0, double x1, double y1){

    int xt,yt=0;

    yt = get_sign(y1-y0)*round(abs(y1-y0)) + y0;
    xt = get_sign(x1-x0)*round(abs(x1-x0)) + x0;
    return std::make_pair(xt,yt);
}

