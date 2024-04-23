#include <SFML/Graphics.hpp>
#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Dense>

#define GRID_STEP_SIZE 5
#define GRID_WIDTH 840
#define GRID_HEIGTH 840

void plotLine(double x0, double y0, double x1, double y1, std::vector<std::pair<int,int>> &points);
Eigen::Vector3d hector_slam(Eigen::Vector3d robot_pos , std::vector<Eigen::Vector2d> scan_endpoints);

struct GridParameters{
    size_t grid_height;
    size_t grid_width;
    size_t step_size;
    std::pair<size_t,size_t> origin;
    sf::Color grid_color;

};



class Grid{

    public :

        Grid(GridParameters &grid_parameters) :
        _grid_parameters(grid_parameters) {
        }


        std::vector<sf::Vertex>* init_grid()
        {
            /* vertical grid lines*/
            for(size_t i = 0; i<_grid_parameters.grid_width; i+=_grid_parameters.step_size){
                if(i == _grid_parameters.origin.second ){
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, 0), sf::Color::Red));
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, _grid_parameters.grid_height), sf::Color::Red));
                }
                else{
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, 0), _grid_parameters.grid_color));
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(i, _grid_parameters.grid_height), _grid_parameters.grid_color));
                }
            }

            /* horizontal grid lines*/
            for(size_t i = 0; i<_grid_parameters.grid_height; i+=_grid_parameters.step_size){
                if(i == _grid_parameters.origin.first ){
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(0, i), sf::Color::Red));
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(_grid_parameters.grid_width, i), sf::Color::Red));
                }
                else{
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(0, i), _grid_parameters.grid_color));
                    _vertice_array.push_back(sf::Vertex(sf::Vector2f(_grid_parameters.grid_width, i), _grid_parameters.grid_color));
                }
            }
        
            return &_vertice_array;

        }

        sf::Text* init_texts(){
            
            _font.loadFromFile("./fonts/open-sans/OpenSans-Bold.ttf");

            _origin_text.setString(std::string("0,0"));
            _origin_text.setPosition(_grid_parameters.origin.first,_grid_parameters.grid_height-_grid_parameters.origin.second );
            _origin_text.setOrigin(0,0);
            _origin_text.setFont(_font);
            _origin_text.setColor(sf::Color::White);
            _origin_text.setCharacterSize(12);

            return &_origin_text;
        }

        sf::RectangleShape* draw_cell(size_t x, size_t y,sf::Color color = sf::Color::Yellow){
            auto p_rectangle_shape = new sf::RectangleShape; 
            _cell_vector.x = (float) _grid_parameters.step_size;
            _cell_vector.y = (float) _grid_parameters.step_size;
            _cell_position.x =  ((x-0.5)*_grid_parameters.step_size) + _grid_parameters.origin.first;
            _cell_position.y = _grid_parameters.grid_height - ((y+0.5)*_grid_parameters.step_size) - _grid_parameters.origin.second;
            p_rectangle_shape->setFillColor(color);
            p_rectangle_shape->setSize(_cell_vector);
            p_rectangle_shape->setPosition(_cell_position);

            return p_rectangle_shape;

        }


        sf::Vertex* draw_line(size_t x1 , size_t y1, size_t x2 , size_t y2)
        {

            _line_points[0] = sf::Vertex(sf::Vector2f((x1*_grid_parameters.step_size) + _grid_parameters.origin.first
             , _grid_parameters.grid_height - (y1*_grid_parameters.step_size) - _grid_parameters.origin.second));
            _line_points[0].color = sf::Color::Blue;

            _line_points[1] = sf::Vertex(sf::Vector2f((x2*_grid_parameters.step_size) + _grid_parameters.origin.first
             , _grid_parameters.grid_height - (y2*_grid_parameters.step_size) - _grid_parameters.origin.second));
            _line_points[1].color = sf::Color::Blue;
        
            return _line_points;
        }
        
        sf::Vertex* draw_line_without_grid(size_t x1 , size_t y1, size_t x2 , size_t y2)
        {

            _line_points[0] = sf::Vertex(sf::Vector2f(x1 + _grid_parameters.origin.first
             , _grid_parameters.grid_height - y1 - _grid_parameters.origin.second));
            _line_points[0].color = sf::Color::Blue;

            _line_points[1] = sf::Vertex(sf::Vector2f(x2 + _grid_parameters.origin.first
             , _grid_parameters.grid_height - y2 - _grid_parameters.origin.second));
            _line_points[1].color = sf::Color::Blue;
        
            return _line_points;
        }


    private:
        GridParameters _grid_parameters;
        std::vector<sf::Vertex> _vertice_array;
        sf::Text _origin_text;
        sf::Font _font;
        sf::Vector2f _cell_vector;
        sf::Vector2f _cell_position;
        sf::Vertex _line_points[2];
        
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define NUM_X_CELLS GRID_WIDTH/GRID_STEP_SIZE
#define NUM_Y_CELLS GRID_HEIGTH/GRID_STEP_SIZE
#define NUM_CELLS NUM_X_CELLS*NUM_Y_CELLS
#define PROB_OCCUPIED 0.95
#define PROB_PRIOR 0.5
#define PROB_FREE 0.05

#define LOGS_ODDS_RATIO(x)  log(x/(1 - x))
#define PROB(x)  1/(1+(1/std::pow(10, x)))
#define POINT_DIS_SQ(x0,y0,x1,y1) std::pow(y1-y0 ,2) + std::pow(x1-x0 ,2) 

struct Cell{
    int x;
    int y;
    double log_odds_ratio;
    double pre_log_odds_ratio;
    double prob_occupied;
};

std::vector<std::pair<int,int>> cells_detected;
std::pair<int,int> occupied_cell={};

Cell cells[NUM_CELLS]={};

void init_cells(){

    for(size_t i=1; i<=NUM_Y_CELLS; i++){
        for(size_t j=1; j<=NUM_Y_CELLS; j++){
            auto index = (j-1) + (i-1)*NUM_Y_CELLS;
            cells[index].x = j;
            cells[index].y = i;
            cells[index].prob_occupied = PROB_PRIOR;
            cells[index].log_odds_ratio = LOGS_ODDS_RATIO(PROB_PRIOR);
            cells[index].pre_log_odds_ratio = LOGS_ODDS_RATIO(PROB_PRIOR);
        }
    }
}


void occupancy_grid_mapping(){
    for(auto &cell:cells){
        auto it  = std::find(cells_detected.begin(), cells_detected.end(), std::make_pair(cell.x,cell.y));
        if(it != cells_detected.end()){

            if((cell.x == occupied_cell.first) && (cell.y == occupied_cell.second)){
                /* CASE 1 : cell occupied*/
                cell.log_odds_ratio = LOGS_ODDS_RATIO(PROB_OCCUPIED) + cell.pre_log_odds_ratio - LOGS_ODDS_RATIO(PROB_PRIOR);
                //std::cout<<cell.x<<" , " <<cell.y<< " , "<<"occupied , "<<cell.log_odds_ratio<<" , "<<PROB(cell.log_odds_ratio)<< " , "<<cell.pre_log_odds_ratio<<std::endl;
                
            }
            else{
                /* CASE 2 : cell FREE*/
                cell.log_odds_ratio = LOGS_ODDS_RATIO(PROB_FREE) + cell.pre_log_odds_ratio - LOGS_ODDS_RATIO(PROB_PRIOR);
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

void filter_detect_cells(double x0,double y0,double x1,double y1,std::vector<std::pair<int,int>> &points){
    plotLine(x0, y0, x1, y1, points);
    size_t max_dis=0;
    // occupied_cell.first = 0;
    // occupied_cell.second = 0;
    // for(auto point:points){
    //     auto point_dis = POINT_DIS_SQ(x0,y0,point.first,point.second);
    //     if (max_dis <=point_dis ){
    //         max_dis = point_dis;
    //         occupied_cell.first = point.first;
    //         occupied_cell.second = point.second;
    //     }
    // }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<std::pair<double,double>> points_float;
void plotLineLow(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert = false) {
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
            points_float.emplace_back(std::make_pair(x0 - (abs(x-x0)),y));
        }
        else{
            points.emplace_back(std::make_pair(round(x),round(y)));
            points_float.emplace_back(std::make_pair(x,y));
        }
        if (D > 0) {
            y = y + yi;
            D = D + (2 * (dy - dx));
        } else {
            D = D + 2 * dy;
        }
    }
}

void plotLineHigh(double x0, double y0, double x1_1, double y1_1, std::vector<std::pair<int,int>> &points,bool invert = false) {

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
            points_float.emplace_back(std::make_pair(x,y0 - (abs(y-y0))));
        }
        else{
            points.emplace_back(std::make_pair(round(x),round(y)));
            points_float.emplace_back(std::make_pair(x,y));
        }
        if (D > 0) {
            x = x + xi;
            D = D + (2 * (dx - dy));
        } else {
            D = D + 2 * dx;
        }
    }
}

void plotLine(double x0, double y0, double x1, double y1, std::vector<std::pair<int,int>> &points) {
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
int get_sign(double x){
    return int(x/abs(x));
}

std::pair<int,int> find_occupied_cell_cordinates(double x0, double y0, double x1, double y1){

    int xt,yt=0;

    yt = get_sign(y1-y0)*round(abs(y1-y0)) + y0;
    xt = get_sign(x1-x0)*round(abs(x1-x0)) + x0;
    return std::make_pair(xt,yt);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double solveQuadratic(double a, double b, double c) {
    static int i = 0;

    if(i>=360){
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
            if(i<180){
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
        
        
      //  std::cout << "Two real roots: " << root1 << " and " << root2 << "\n";
    }
    i++;
}


double circle_inside_distance(std::pair<double,double> robot_pos,double beam_angle,double radius){

    auto a =1;
    auto b = 2 *(robot_pos.first * sin(beam_angle) + robot_pos.second * cos(beam_angle));
    auto c = robot_pos.first*robot_pos.first + robot_pos.second*robot_pos.second - radius*radius;

    return solveQuadratic(a,b,c);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::pair<int,int> robot_pos((GRID_WIDTH/2)/GRID_STEP_SIZE,(GRID_HEIGTH/2)/GRID_STEP_SIZE);
double distances[360]={};

void get_data_0(){

    // for (size_t i = 0; i < 360; i++)
    // {
    //     double f = (double)rand() / RAND_MAX;
    //     distances[i]= 0.1 + f * (28 - 0.1);
    // }
    for (size_t i = 0; i < 360; i++)
    {
        distances[i]= 40;//circle
        //std::cout<<distances[i]<<std::endl;
    }
}

std::pair<int,int> robot_pos1((GRID_WIDTH/2)/GRID_STEP_SIZE,(GRID_HEIGTH/2)/GRID_STEP_SIZE);
void get_data_1(){
    robot_pos1.first +=1;
    robot_pos1.second +=1;
    // for (size_t i = 0; i < 360; i++)
    // {
    //     double f = (double)rand() / RAND_MAX;
    //     distances[i]= 0.1 + f * (28 - 0.1);
    // }
    for (size_t i = 0; i < 360; i++)
    {
        distances[i]= circle_inside_distance(std::make_pair(10,10),2*M_PI*i/360,40);//circle
        
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




auto angle_min = -3.1415927410125732;
auto angle_max = 3.1415927410125732;
auto angle_increment = 0.012466637417674065;
// time_increment: 0.000200034148292616
// scan_time: 0.12302099913358688
// range_min: 0.10000000149011612
// range_max: 12.0
double ranges[] = {0.0, 1.3990000486373901, 1.399999976158142, 1.4010000228881836, 1.4040000438690186, 1.406000018119812, 1.4119999408721924, 1.4160000085830688, 1.4179999828338623, 1.4210000038146973, 1.7910000085830688, 1.7960000038146973, 1.8020000457763672, 1.8049999475479126, 0.0, 2.0390000343322754, 0.0, 1.878000020980835, 1.847000002861023, 0.0, 0.0, 0.0, 1.6959999799728394, 1.687000036239624, 1.687999963760376, 0.0, 1.590999960899353, 1.5809999704360962, 1.5750000476837158, 1.565000057220459, 1.8070000410079956, 1.7899999618530273, 1.7760000228881836, 1.7690000534057617, 1.7640000581741333, 1.7740000486373901, 1.7860000133514404, 0.0, 2.184000015258789, 2.190000057220459, 2.2019999027252197, 2.2209999561309814, 2.2309999465942383, 0.0, 2.066999912261963, 2.0820000171661377, 2.1089999675750732, 2.124000072479248, 2.13700008392334, 2.1540000438690186, 2.171999931335449, 2.2079999446868896, 2.2219998836517334, 2.243000030517578, 2.257999897003174, 2.2829999923706055, 2.319000005722046, 2.3369998931884766, 2.365000009536743, 2.382999897003174, 2.4100000858306885, 2.4590001106262207, 2.4790000915527344, 2.5299999713897705, 0.0, 0.0, 3.3550000190734863, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1449999809265137, 1.1330000162124634, 1.1230000257492065, 1.1160000562667847, 1.1050000190734863, 1.0859999656677246, 1.0800000429153442, 1.0700000524520874, 1.0640000104904175, 1.055999994277954, 1.0399999618530273, 1.034999966621399, 1.0269999504089355, 1.0180000066757202, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9359999895095825, 0.8849999904632568, 0.8700000047683716, 0.859000027179718, 0.8489999771118164, 0.8339999914169312, 0.828000009059906, 0.8220000267028809, 0.8180000185966492, 0.8119999766349792, 0.8090000152587891, 0.8069999814033508, 0.8050000071525574, 0.8040000200271606, 0.8029999732971191, 0.8040000200271606, 0.8050000071525574, 0.8059999942779541, 0.8069999814033508, 0.8130000233650208, 0.8169999718666077, 0.8209999799728394, 0.8259999752044678, 0.8320000171661377, 0.8460000157356262, 0.8579999804496765, 0.8700000047683716, 0.8859999775886536, 0.0, 0.0, 0.0, 1.034999966621399, 1.0119999647140503, 0.996999979019165, 0.9879999756813049, 0.9750000238418579, 0.9710000157356262, 0.9660000205039978, 0.9629999995231628, 0.9610000252723694, 0.9610000252723694, 0.9639999866485596, 0.9639999866485596, 0.9670000076293945, 0.9760000109672546, 0.9829999804496765, 0.9919999837875366, 1.0019999742507935, 0.0, 1.1480000019073486, 1.1369999647140503, 1.1390000581741333, 1.1619999408721924, 1.187999963760376, 1.2020000219345093, 1.2280000448226929, 0.0, 1.2920000553131104, 1.284000039100647, 1.2760000228881836, 1.2280000448226929, 1.2029999494552612, 1.2359999418258667, 1.2239999771118164, 1.218000054359436, 1.1970000267028809, 0.0, 1.2369999885559082, 1.2569999694824219, 1.2669999599456787, 1.2979999780654907, 1.3079999685287476, 1.312999963760376, 0.0, 1.7569999694824219, 0.0, 1.6510000228881836, 1.7289999723434448, 0.0, 1.819000005722046, 1.8300000429153442, 1.840000033378601, 1.8680000305175781, 1.88100004196167, 1.8940000534057617, 1.906999945640564, 1.9249999523162842, 1.9589999914169312, 1.972000002861023, 1.9930000305175781, 2.00600004196167, 2.0239999294281006, 0.0, 1.6799999475479126, 1.659000039100647, 1.625, 1.6080000400543213, 1.590000033378601, 1.5770000219345093, 1.559000015258789, 1.531000018119812, 1.5169999599456787, 1.5049999952316284, 1.4889999628067017, 1.4639999866485596, 1.4520000219345093, 1.440000057220459, 1.4270000457763672, 1.4049999713897705, 1.3940000534057617, 1.3839999437332153, 1.3739999532699585, 1.3639999628067017, 1.343999981880188, 1.3350000381469727, 1.3279999494552612, 1.3179999589920044, 1.3029999732971191, 1.2940000295639038, 1.2869999408721924, 1.277999997138977, 1.2719999551773071, 1.2589999437332153, 1.2519999742507935, 1.246000051498413, 1.2400000095367432, 1.2280000448226929, 1.222000002861023, 1.215999960899353, 1.2100000381469727, 1.2059999704360962, 1.1970000267028809, 1.1920000314712524, 1.1859999895095825, 1.1820000410079956, 1.1779999732971191, 1.1710000038146973, 1.1679999828338623, 1.1619999408721924, 1.159999966621399, 1.152999997138977, 1.1510000228881836, 1.1480000019073486, 1.1440000534057617, 1.1419999599456787, 1.1369999647140503, 1.1339999437332153, 1.1319999694824219, 1.128999948501587, 1.128999948501587, 1.1239999532699585, 1.1230000257492065, 1.121000051498413, 0.0, 0.0, 1.5870000123977661, 1.6069999933242798, 1.6059999465942383, 1.6050000190734863, 1.6069999933242798, 1.6069999933242798, 1.6050000190734863, 1.5219999551773071, 1.531999945640564, 1.562000036239624, 1.5779999494552612, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1629999876022339, 1.1699999570846558, 1.1720000505447388, 1.1679999828338623, 1.1670000553131104, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1540000438690186, 1.125, 1.1299999952316284, 1.1369999647140503, 1.1440000534057617, 1.1540000438690186, 1.1670000553131104, 1.1929999589920044, 1.2059999704360962, 1.2319999933242798, 1.2319999933242798, 1.253000020980835, 1.2860000133514404, 1.3040000200271606, 1.319000005722046, 0.0, 1.409000039100647, 1.4170000553131104, 1.4359999895095825, 1.4479999542236328, 1.4589999914169312, 1.4700000286102295, 1.4789999723434448, 1.5019999742507935, 1.5149999856948853, 1.5260000228881836, 1.5360000133514404, 1.5490000247955322, 1.5750000476837158, 1.590999960899353, 1.6269999742507935, 1.61899995803833, 0.0, 0.0, 1.684000015258789, 1.7089999914169312, 1.7230000495910645, 1.7480000257492065, 0.671999990940094, 0.6639999747276306, 0.6359999775886536, 0.628000020980835, 0.6330000162124634, 0.6470000147819519, 0.652999997138977, 0.6700000166893005, 0.6800000071525574, 0.6840000152587891, 0.6850000023841858, 0.6840000152587891, 0.6880000233650208, 0.6909999847412109, 0.6919999718666077, 0.6880000233650208, 0.6840000152587891, 0.6779999732971191, 0.6700000166893005, 0.6690000295639038, 0.6710000038146973, 0.6800000071525574, 0.6940000057220459, 0.0, 0.0, 1.1690000295639038, 1.149999976158142, 1.1319999694824219, 1.1150000095367432, 1.0800000429153442, 1.065000057220459, 1.0499999523162842, 1.034999966621399, 0.9739999771118164, 0.9459999799728394, 0.9340000152587891, 0.9139999747276306, 0.9100000262260437, 0.9210000038146973, 0.9290000200271606, 0.9359999895095825, 0.0, 0.0, 2.3610000610351562, 0.0, 0.0, 2.4030001163482666, 2.3420000076293945, 2.3489999771118164, 0.0, 2.316999912261963, 2.318000078201294, 2.2909998893737793, 2.259999990463257, 2.23799991607666, 2.2190001010894775, 2.180999994277954, 2.1530001163482666, 2.188999891281128, 1.9859999418258667, 1.9739999771118164, 1.965999960899353, 1.965999960899353, 1.965999960899353, 1.9620000123977661, 1.5829999446868896, 0.0, 1.9249999523162842, 1.9249999523162842, 0.0, 0.0, 1.805999994277954, 0.0, 2.234999895095825, 0.0, 0.0, 1.4550000429153442, 1.440000057220459, 1.434999942779541, 1.4299999475479126, 1.4270000457763672, 1.4220000505447388, 1.4140000343322754, 1.4110000133514404, 1.409000039100647, 1.406000018119812, 1.4010000228881836, 1.3990000486373901, 1.3980000019073486, 1.3969999551773071, 1.3960000276565552, 1.3949999809265137, 1.3940000534057617, 1.3930000066757202, 1.3949999809265137, 1.3940000534057617, 1.3940000534057617, 1.3949999809265137, 1.3960000276565552, 1.3960000276565552};


int main() {
    // Create the window
    sf::RenderWindow window(sf::VideoMode(GRID_WIDTH, GRID_HEIGTH), "SFML Test Grid");

    GridParameters grid_paramters = {GRID_HEIGTH, GRID_WIDTH, GRID_STEP_SIZE, std::make_pair(GRID_STEP_SIZE/2 ,GRID_STEP_SIZE/2),sf::Color(0,100,0) };

    Grid grid(grid_paramters);

    auto p_grid_vertice_array = grid.init_grid();
    auto p_grid_text = grid.init_texts();

    std::vector<sf::Vertex> lines;
    init_cells();
    double angle = 0;
    get_data_0();

    std::vector<std::pair<int, int>> detetcted_all_cells;
    for(auto data :distances)
    {
        // if(data<0.1){
        //     angle-=0.012466637417674065;
        //     continue;
        // }
        cells_detected.clear();
        
        auto y1 = data*cos(angle) + robot_pos.first;
        auto x1 = data*sin(angle)+ robot_pos.second;
        angle+=2*M_PI/360;
        

        occupied_cell = find_occupied_cell_cordinates(robot_pos.first,robot_pos.second,x1,y1);

        filter_detect_cells(robot_pos.first , robot_pos.second, x1,y1,cells_detected);

        auto it = std::find(cells_detected.begin(), cells_detected.end(), occupied_cell);

        if (it == cells_detected.end()) {
            cells_detected.push_back(occupied_cell);
        } 
        occupancy_grid_mapping();

        // auto p_line = grid.draw_line(robot_pos.first,robot_pos.second,round(x1),round(y1));
        auto p_line = grid.draw_line_without_grid(robot_pos.first*GRID_STEP_SIZE,robot_pos.second*GRID_STEP_SIZE,round(x1*GRID_STEP_SIZE),round(y1*GRID_STEP_SIZE));
        
        lines.emplace_back(*p_line);
        lines.emplace_back(*(p_line+1U));
 
        detetcted_all_cells.insert(detetcted_all_cells.end(), cells_detected.begin(), cells_detected.end());
    }


     get_data_1();
     angle = 0;

std::vector<Eigen::Vector2d> scan_endpoints;

    for(auto data :distances)
    {
        // if(data<0.1){
        //     angle-=0.012466637417674065;
        //     continue;
        // }
        // cells_detected.clear();
        
        // auto y1 = data*cos(angle) + robot_pos1.first;
        // auto x1 = data*sin(angle)+ robot_pos1.second;
        
        

        // occupied_cell = find_occupied_cell_cordinates(robot_pos1.first,robot_pos1.second,x1,y1);

        // filter_detect_cells(robot_pos1.first , robot_pos1.second, x1,y1,cells_detected);

        // auto it = std::find(cells_detected.begin(), cells_detected.end(), occupied_cell);

        // if (it == cells_detected.end()) {
        //     cells_detected.push_back(occupied_cell);
        // } 
        // occupancy_grid_mapping();

        // // auto p_line = grid.draw_line(robot_pos.first,robot_pos.second,round(x1),round(y1));
        // auto p_line = grid.draw_line_without_grid(robot_pos1.first*GRID_STEP_SIZE,robot_pos1.second*GRID_STEP_SIZE,round(x1*GRID_STEP_SIZE),round(y1*GRID_STEP_SIZE));
        
        // lines.emplace_back(*p_line);
        // lines.emplace_back(*(p_line+1U));
 
        // detetcted_all_cells1.insert(detetcted_all_cells1.end(), cells_detected.begin(), cells_detected.end());

        scan_endpoints.emplace_back(Eigen::Vector2d(data*sin(angle),data*cos(angle)));
        angle+=2*M_PI/360;
    }
    

    std::vector<sf::RectangleShape*> cells_for_draw;
    for(auto cell:cells){
        cells_for_draw.emplace_back(grid.draw_cell(cell.x,cell.y,sf::Color(255*(1-cell.prob_occupied),255*(1-cell.prob_occupied),255*(1-cell.prob_occupied))));
       // std::cout<<cell.x<<" , " <<cell.y<< " , "<<cell.prob_occupied<<std::endl;
    }

    std::vector<sf::RectangleShape*> cells_for_draw1;
    for(auto cell:detetcted_all_cells){
        cells_for_draw1.emplace_back(grid.draw_cell(cell.first,cell.second,sf::Color::Yellow));
    }
    
    
Eigen::Vector3d g = hector_slam(Eigen::Vector3d(robot_pos.first,robot_pos.second,0) , scan_endpoints);

std::cout<<g<<std::endl;

    // Run the main loop
    while (window.isOpen()) {
        // Handle events
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Clear the window
        window.clear(sf::Color::Black);

        
        window.draw(*p_grid_text);
        for(auto cell:cells_for_draw1){
            window.draw(*cell);
        }
        for(auto cell:cells_for_draw){
            window.draw(*cell);
        }
        window.draw(&lines[0],1440,sf::Lines);
        window.draw(&(p_grid_vertice_array->at(0)),p_grid_vertice_array->size(),sf::Lines);
        // Display the content of the window
        window.display();
    }

    return 0;
}

#define DIFF(a,b) a - b
#define M_P(x,y) map_value(x,y)
#define D_X(x,y) map_derivative_x(x,y)
#define D_Y(x,y) map_derivative_y(x,y)

bool isInteger(double value) {
    return ceil(value) == floor(value);
}

int findCellIndexByXAndY(int x, int y) {
    int i = 0;
    for (const auto cell : cells) {
        if (cell.x == x && cell.y == y) {
            return i;
        }
        i++;
    }
    return -1;
}

double map_value(double x, double y){
    int x0 = floor(x);
    int x1 = ceil(x);
    int y0 = floor(y);
    int y1 = ceil(y);

    int cell_index_00 = findCellIndexByXAndY(x0, y0);
    double M_P_00 = (cell_index_00 == -1) ? PROB_PRIOR : cells[cell_index_00].prob_occupied;

    int cell_index_01 = findCellIndexByXAndY(x0, y1);
    double M_P_01 = (cell_index_01 == -1) ? PROB_PRIOR : cells[cell_index_01].prob_occupied;

    int cell_index_10 = findCellIndexByXAndY(x1, y0);
    double M_P_10 = (cell_index_10 == -1) ? PROB_PRIOR : cells[cell_index_10].prob_occupied;

    int cell_index_11 = findCellIndexByXAndY(x1, y1);
    double M_P_11 = (cell_index_11 == -1) ? PROB_PRIOR : cells[cell_index_11].prob_occupied;

    if(x0!=x1 && y0!=y1){
        return (DIFF(y,y0)/DIFF(y1,y0))*(( (DIFF(x,x0)/DIFF(x1,x0))*M_P_11) + ((DIFF(x1,x)/DIFF(x1,x0))*M_P_01) ) +
                (DIFF(y1,y)/DIFF(y1,y0))*( ((DIFF(x,x0)/DIFF(x1,x0))*M_P_10) + ((DIFF(x1,x)/DIFF(x1,x0))*M_P_00) ) ;
    }
    else if(x0!=x1 ){
        return (DIFF(x,x0)/DIFF(x1,x0))*M_P_11 + (DIFF(x1,x)/DIFF(x1,x0))*M_P_01;
    }
    else if(y0!=y1 ){
        return (DIFF(y,y0)/DIFF(y1,y0))*M_P_01 + (DIFF(y1,y)/DIFF(y1,y0))*M_P_00;
    }
    else{
        return M_P_00;
    }

}

double map_derivative_x(double x,double y){
    int x0 = floor(x);
    int x1 = ceil(x);
    int y0 = floor(y);
    int y1 = ceil(y);

    int cell_index_00 = findCellIndexByXAndY(x0, y0);
    double M_P_00 = (cell_index_00 == -1) ? PROB_PRIOR : cells[cell_index_00].prob_occupied;

    int cell_index_01 = findCellIndexByXAndY(x0, y1);
    double M_P_01 = (cell_index_01 == -1) ? PROB_PRIOR : cells[cell_index_01].prob_occupied;

    int cell_index_10 = findCellIndexByXAndY(x1, y0);
    double M_P_10 = (cell_index_10 == -1) ? PROB_PRIOR : cells[cell_index_10].prob_occupied;

    int cell_index_11 = findCellIndexByXAndY(x1, y1);
    double M_P_11 = (cell_index_11 == -1) ? PROB_PRIOR : cells[cell_index_11].prob_occupied;



    if(y0!=y1 && x0!=x1){
        return (DIFF(y,y0)/DIFF(y1,y0))*(M_P_11 - M_P_01 ) +
                (DIFF(y1,y)/DIFF(y1,y0))*(M_P_10 - M_P_00 ) ;
    }
    else if(x0!=x1){
        return M_P_11 - M_P_01;
    }
    else{
        return 0;
    }

}

double map_derivative_y(double x,double y){
    int x0 = floor(x);
    int x1 = ceil(x);
    int y0 = floor(y);
    int y1 = ceil(y);

    int cell_index_00 = findCellIndexByXAndY(x0, y0);
    double M_P_00 = (cell_index_00 == -1) ? PROB_PRIOR : cells[cell_index_00].prob_occupied;

    int cell_index_01 = findCellIndexByXAndY(x0, y1);
    double M_P_01 = (cell_index_01 == -1) ? PROB_PRIOR : cells[cell_index_01].prob_occupied;

    int cell_index_10 = findCellIndexByXAndY(x1, y0);
    double M_P_10 = (cell_index_10 == -1) ? PROB_PRIOR : cells[cell_index_10].prob_occupied;

    int cell_index_11 = findCellIndexByXAndY(x1, y1);
    double M_P_11 = (cell_index_11 == -1) ? PROB_PRIOR : cells[cell_index_11].prob_occupied;


    if(x0!=x1){
    return (DIFF(x,x0)/DIFF(x1,x0))*(M_P_11 - M_P_10 ) +
            (DIFF(x1,x)/DIFF(x1,x0))*(M_P_01 - M_P_00 ) ;
    }
    else if(y0!=y1){
        return M_P_11 - M_P_10;
    }
    else{
        return 0;
    }

}




Eigen::Vector2d get_world_coordinate_for_scan_endpoint(Eigen::Vector3d robot_pos ,Eigen::Vector2d scan_point){
    Eigen::Matrix<double, 2, 2> rot_matrix;

    rot_matrix(0,0) = cos(robot_pos(2));
    rot_matrix(0,1) = -sin(robot_pos(2));
    rot_matrix(1,0) = sin(robot_pos(2));
    rot_matrix(1,1) = cos(robot_pos(2));

    Eigen::Vector2d robot_pos_xy(robot_pos(0),robot_pos(1));

    return rot_matrix*scan_point + robot_pos_xy;

}

Eigen::Matrix<double,2, 3> world_cordinate_derivative(double robot_angle ,Eigen::Vector2d scan_point){

    Eigen::Matrix<double,2, 3> world_cordinate_derivative_matrix;


    world_cordinate_derivative_matrix(0,0) = 1;
    world_cordinate_derivative_matrix(0,1) = 0;
    world_cordinate_derivative_matrix(0,2) = -sin(robot_angle)*scan_point(0)-cos(robot_angle)*scan_point(1);
    world_cordinate_derivative_matrix(1,0) = 0;
    world_cordinate_derivative_matrix(1,1) = 1;
    world_cordinate_derivative_matrix(1,2) = cos(robot_angle)*scan_point(0)-sin(robot_angle)*scan_point(1);

    return world_cordinate_derivative_matrix;

}

Eigen::Vector3d hector_slam(Eigen::Vector3d robot_pos , std::vector<Eigen::Vector2d> scan_endpoints){

    Eigen::Matrix<double, 3, 1> part_1_1;
    part_1_1.setZero();

    Eigen::Matrix<double, 3, 3> part_1_2;
    part_1_2.setZero();

    for(auto scan_endpoint : scan_endpoints){
        auto scan_world_cordinate = get_world_coordinate_for_scan_endpoint(robot_pos, scan_endpoint);

        if(isInteger(scan_world_cordinate(0)) || isInteger(scan_world_cordinate(1))){
            continue;
        }

        std::cout<<scan_world_cordinate<<std::endl;
        /* ∇M(S_i(ξ)) */
        Eigen::Vector2d full_derivative_transpose(D_X(scan_world_cordinate(0),scan_world_cordinate(1)) , D_Y(scan_world_cordinate(0),scan_world_cordinate(1)));

        /* (∂S_i(ξ)/∂ξ)*/
        Eigen::Matrix<double,2, 3> world_cordinate_derivative_matrix = world_cordinate_derivative(robot_pos(2) ,scan_endpoint);

        /* ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ)*/
        Eigen::Matrix<double, 1, 3> part_1 = (full_derivative_transpose.transpose()*world_cordinate_derivative_matrix);

        /* [1 − M(S_i(ξ))] */
        double part_2 = 1 - M_P(scan_world_cordinate(0),scan_world_cordinate(1));

        /* ( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )' * [1 − M(S_i(ξ))]*/
        Eigen::Matrix<double, 3, 1> part_3 = part_1.transpose() * part_2;

        /* summation */
        part_1_1+=part_3;

        /* ( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )' * ( ∇M(S_i(ξ)) * (∂S_i(ξ)/∂ξ) )*/
        Eigen::Matrix<double, 3, 3>  part_4 = part_1.transpose()*part_1;

        part_1_2+=part_4;

    }

    return (part_1_2.inverse()*part_1_1);

}


