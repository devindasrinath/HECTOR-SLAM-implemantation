#include "occupancy_grid_mapping.h"

OccupancyGridMap::OccupancyGridMap(SensorProbabilities sensorProbabilities, uint32_t width, uint32_t height):
 _sensorProbabilities(sensorProbabilities) , _width(width),_height(height)
{
    init_cells();
}

OccupancyGridMap::~OccupancyGridMap()
{
    delete _p_cells;
}

void OccupancyGridMap::init_cells(){
    _p_cells = new std::vector<Cell>(_width*_height);
    for(size_t i=1; i<=_height; i++){
        for(size_t j=1; j<=_width; j++){
            auto index = (j-1) + (i-1)*_width;
            (*_p_cells)[index].x = j;
            (*_p_cells)[index].y = i;
            (*_p_cells)[index].prob_occupied = _sensorProbabilities.PROB_PRIOR_N;
            (*_p_cells)[index].log_odds_ratio = LOGS_ODDS_RATIO(_sensorProbabilities.PROB_PRIOR_N);
            (*_p_cells)[index].pre_log_odds_ratio = LOGS_ODDS_RATIO(_sensorProbabilities.PROB_PRIOR_N);
        }
    }

    log_prob_occ = LOGS_ODDS_RATIO(_sensorProbabilities.PROB_OCCUPIED_N);
    log_prob_free = LOGS_ODDS_RATIO(_sensorProbabilities.PROB_FREE_N);
    log_prob_prior =LOGS_ODDS_RATIO(_sensorProbabilities.PROB_PRIOR_N);

}

std::vector<Cell>* OccupancyGridMap::get_cells(){
    return _p_cells;
}

void OccupancyGridMap::occupancy_grid_mapping(){
    for(auto cell:_cells_detected){

            auto index = (cell.second-1)*_width + (cell.first-1);

            if(index>(_width*_height)){
                continue;
            }

            if((cell.first == _occupied_cell.first) && (cell.second == _occupied_cell.second)){
                /* CASE 1 : cell occupied*/
                (*_p_cells)[index].log_odds_ratio = log_prob_occ + (*_p_cells)[index].pre_log_odds_ratio - log_prob_prior;
                //std::cout<<cell.x<<" , " <<cell.y<< " , "<<"occupied , "<<cell.log_odds_ratio<<" , "<<PROB(cell.log_odds_ratio)<< " , "<<cell.pre_log_odds_ratio<<std::endl;
                
            }
            else{
                /* CASE 2 : cell FREE*/
                (*_p_cells)[index].log_odds_ratio = log_prob_free + (*_p_cells)[index].pre_log_odds_ratio - log_prob_prior;
                //std::cout<<cell.x<<" , " <<cell.y<< " , "<<"free , "<<cell.log_odds_ratio<<" , "<<PROB(cell.log_odds_ratio)<< " , "<<cell.pre_log_odds_ratio<<std::endl;
            }
            

    
    
        (*_p_cells)[index].prob_occupied = PROB((*_p_cells)[index].log_odds_ratio);

        // if(came)
        //     std::cout<<cell.x<<" , " <<cell.y<< " , "<<"occupied , "<<cell.log_odds_ratio<<" , "<<cell.prob_occupied<< " , "<<cell.pre_log_odds_ratio<<std::endl;

        (*_p_cells)[index].pre_log_odds_ratio = (*_p_cells)[index].log_odds_ratio;
        
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


std::pair<int,int> OccupancyGridMap::find_occupied_cell_coordinates(double x1, double y1){
    return std::make_pair(round(x1),round(y1));
}


void OccupancyGridMap::runOccupancyGridMap(std::vector<double> distanceDataSet,std::vector<double> angleDataSet,Eigen::Vector3d robot_pos){
    

    if (distanceDataSet.size()!=angleDataSet.size()){
        throw std::runtime_error("Data sets are not equal!");
        return;
    }

    //std::cout<<"robot_pos : "<<robot_pos(0)<<" "<<robot_pos(1)<<" "<<robot_pos(2)<<std::endl;
    for(size_t i=0;i<distanceDataSet.size();i++)
    {
        if(distanceDataSet[i]<=0){
            continue;
        }
        _cells_detected.clear();
        
        auto x1_r = distanceDataSet[i]*sin(angleDataSet[i]);
        auto y1_r = distanceDataSet[i]*cos(angleDataSet[i]);

        auto x1_g = (sin(robot_pos(2))*y1_r) +  (cos(robot_pos(2))*x1_r) + robot_pos(0);
        auto y1_g = (-sin(robot_pos(2))*x1_r) +  (cos(robot_pos(2))*y1_r) + robot_pos(1);
        
        _occupied_cell = find_occupied_cell_coordinates(x1_g,y1_g);


        filter_detect_cells(robot_pos(0) , robot_pos(1), x1_g,y1_g, _cells_detected);

        auto it = std::find(_cells_detected.begin(), _cells_detected.end(), _occupied_cell);

        if (it == _cells_detected.end()) {
            _cells_detected.push_back(_occupied_cell);
        } 
        occupancy_grid_mapping();

    }
}








