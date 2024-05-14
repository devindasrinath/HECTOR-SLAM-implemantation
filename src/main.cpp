#include "main.h"
#include "simulation.cpp"
#include "hector_slam.h"
#include "occupancy_grid_mapping.h"
#include "common.h"
#include "grid_operations.h"
#include "simulation.h"
#include <future>
#include <chrono>
#include "web_socket.h"
#include "configurations.h"


/*structs*/
struct LocalizationData {
    SensorProbabilities sensorProbabilities;
    OccupancyGridMap *occupancy_grid_map;
    std::array<Eigen::Vector2d, NUM_DATA> point_cloud;
    uint32_t num_x_cells;
    uint32_t num_y_cells;
};

struct RobotDrawingProperties{
    std::vector<sf::Vertex*> robot_path;
    std::vector<sf::RectangleShape> updatedCells;
    std::vector<sf::RectangleShape> cellsOfPreviousPosition;
    sf::CircleShape *circleShape = nullptr;
};

/*variables*/
LocalizationData localizationData0;
LocalizationData localizationData1;
LocalizationData localizationData2;

uint8_t grid_step_sizes[]={8,4,2};
uint8_t step_size_for_draw = grid_step_sizes[1];

Eigen::Vector3d threshold(0.1,0.1,0.1);
sf::Color grid_color = sf::Color(0,100,0);

std::vector<Cell> cells = {};
std::vector<Cell> pre_cells = {};

uint32_t num_cells_in_grid = (GRID_WIDTH*GRID_HEIGHT)/step_size_for_draw*step_size_for_draw;
uint32_t num_x_cells_in_grid = (GRID_WIDTH)/step_size_for_draw;
uint32_t num_y_cells_in_grid = (GRID_HEIGHT)/step_size_for_draw;

std::array<Eigen::Vector2d,NUM_DATA> point_cloud_0;
std::array<Eigen::Vector2d,NUM_DATA> point_cloud_1;
std::array<Eigen::Vector2d,NUM_DATA> point_cloud_2;

Eigen::Vector3d robot_pos;
Eigen::Vector3d robot_pos_old;
size_t data_set_index = 0;

/*flags*/
std::atomic_bool render_changes(false);
std::atomic_bool background_completed(false);
std::atomic_bool end_of_program(false);

/*function prototypes*/
template <typename T, size_t N>
void point_generate_according_to_grid(const uint8_t &step_size,
                                const uint8_t &min_step_size,
                               std::array<T, N> distance_dataset_new,
                               std::array<T, N> angle_dataset_new,
                               std::array<Eigen::Vector2d,N> &point_cloud);
void runLocalization(Eigen::Vector3d &robot_pos_old, LocalizationData &localizationData0,LocalizationData &localizationData1,LocalizationData &localizationData2,Eigen::Vector3d &robot_pos_new);
template <typename T, size_t N>
Eigen::Vector3d localization(  Eigen::Vector3d robot_pos,
                    const SensorProbabilities &sensorProbabilities,
                    OccupancyGridMap &occupancyGridMap,
                    std::array<Eigen::Vector2d,N> &point_cloud,
                    const uint32_t &num_x_cells, const uint32_t &num_y_cells);
void runMapping(uint8_t grid_step_sizes[], std::array<double, NUM_DATA> &distance_dataset_old,std::array<double, NUM_DATA> &angle_dataset_old,std::vector<Cell> &cells, Eigen::Vector3d robot_pos_old );
template <typename T, size_t N>
void mapping(const uint8_t &step_size,
                               const uint8_t &min_step_size,
                               std::array<T, N> &distance_dataset_old,
                               std::array<T, N> &angle_dataset_old,
                               double *robot_pos_old,
                               OccupancyGridMap &occupancyGridMap);
void updateDrawingProperties(std::vector<Cell> &cells, Eigen::Vector3d robot_pos, Grid &grid, RobotDrawingProperties &robotDrawingProperties);
void initDrawingProperties(uint32_t num_x_cells,uint32_t num_y_cells, std::vector<sf::RectangleShape> &updatedCells,Grid &grid);
bool isPosUpdated(Eigen::Vector3d robot_pos,Eigen::Vector3d robot_pos_old,Eigen::Vector3d threshold);
Eigen::Vector3d robot_pos_init(void);
void renderGraphics(sf::RenderWindow &window, RobotDrawingProperties &robotDrawingProperties);
void deployWebSocket(ROSBridgeClient &client,const std::string webSocketAddress);



int main()
{


    pre_cells.reserve((GRID_WIDTH*GRID_WIDTH)/(grid_step_sizes[1]*grid_step_sizes[1]));

    /**create window and grid **/
    sf::RenderWindow window(sf::VideoMode(GRID_WIDTH, GRID_HEIGHT), "SLAM Map Grid");

    GridParameters grid_parameters = {GRID_HEIGHT, GRID_WIDTH, step_size_for_draw, std::make_pair(step_size_for_draw,step_size_for_draw),grid_color };
    Grid grid(grid_parameters);

    RobotDrawingProperties robotDrawingProperties;

    std::thread rendering_thread(renderGraphics , std::ref(window),std::ref(robotDrawingProperties));

    cells.reserve(num_cells_in_grid);

    background_completed.store(false);
    initDrawingProperties(num_x_cells_in_grid,num_y_cells_in_grid, robotDrawingProperties.updatedCells,grid);
    while(!background_completed.load());

    robotDrawingProperties.circleShape = grid.create_robot_point(ROBOT_INDICATOR_CIRCLE_RADIUS);

    ROSBridgeClient client;
    std::thread web_socket_thread(deployWebSocket, std::ref(client) ,WEB_SOCKET_ADDRESS);


    while(true){
        
        if(client.get_status()==DATA_RECEIVED)
        /* generate or retrieved  */
        {  
            auto logic_instance_1 = std::chrono::high_resolution_clock::now(); 
            
            client.reset_status();

            data_set_index++;
            
            auto distance_data = client.getRanges();
            auto angle_data = client.getAngles();

            /* run localization*/
            if(data_set_index == FIRST_DATA_SET){
                /* no need of find the postion since we set it as 0,0,0 which consider as the center of the grid*/
                robot_pos =  robot_pos_init();
                
            }
            else{
                /* generate point cloud data and run algorithm ofr localization*/
                point_generate_according_to_grid<double,NUM_DATA>(grid_step_sizes[0],
                                    grid_step_sizes[2],
                                distance_data,
                                angle_data,
                                point_cloud_0);
                localizationData0.point_cloud = point_cloud_0;
                point_generate_according_to_grid<double,NUM_DATA>(grid_step_sizes[1],
                                    grid_step_sizes[2],
                                distance_data,
                                angle_data,
                                point_cloud_1);
                localizationData1.point_cloud = point_cloud_1;
                point_generate_according_to_grid<double,NUM_DATA>(grid_step_sizes[2],
                                    grid_step_sizes[2],
                                    distance_data,
                                    angle_data,
                                    point_cloud_2);
                localizationData2.point_cloud = point_cloud_2;
                
                runLocalization(robot_pos, localizationData0,localizationData1, localizationData2,robot_pos);
            }
            auto logic_instance_2 = std::chrono::high_resolution_clock::now(); 

            /* run mapping*/
            if(isPosUpdated(robot_pos,robot_pos_old,threshold) || (data_set_index == FIRST_DATA_SET)){
                runMapping(grid_step_sizes, distance_data,angle_data, cells,robot_pos );
                robot_pos_old =robot_pos;
                updateDrawingProperties(cells, robot_pos, grid, robotDrawingProperties);
            }
            else{
                robot_pos = robot_pos_old;
            }
            
            auto logic_instance_3 = std::chrono::high_resolution_clock::now(); 

            auto localization_duration = std::chrono::duration_cast<std::chrono::milliseconds>(logic_instance_2 - logic_instance_1).count();
            auto mapping_duration = std::chrono::duration_cast<std::chrono::milliseconds>(logic_instance_3 - logic_instance_2).count();
            std::cout << "Localization logic Execution time : " << localization_duration << " ms" << std::endl;
            std::cout << "Mapping logic Execution time : " << mapping_duration << " ms" << std::endl;
            
            std::this_thread::sleep_for(std::chrono::microseconds(MAIN_THREAD_SLEEP_TIME));
            
            std::cout<<"Number data sets processed: "<<data_set_index<<std::endl;
       }
        

    }
   
    rendering_thread.join();
    web_socket_thread.detach();
    web_socket_thread.join();

    delete robotDrawingProperties.circleShape;
     
    return 0;
}



void renderGraphics(sf::RenderWindow &window, RobotDrawingProperties &robotDrawingProperties){
        while (window.isOpen() && (!end_of_program.load())) {
            sf::Event event;

            while (window.pollEvent(event)) {
                // Handle different event types
                switch (event.type) {
                    case sf::Event::Closed:
                        end_of_program.store(true);
                        window.close();
                        break;
                    default:
                        break; // Handle other events if needed
                }
            }

            if (render_changes.load() && (!end_of_program.load())) {
                
                render_changes.store(false);

                auto rendering_instance_1 = std::chrono::high_resolution_clock::now(); 

                for (auto &cell : robotDrawingProperties.updatedCells) {
                    window.draw(cell);
                }

                for (auto &cell : robotDrawingProperties.cellsOfPreviousPosition) {
                    
                    window.draw(cell);
                }
                

                if(robotDrawingProperties.robot_path.size()>=2){
                    sf::VertexArray vertices(sf::LineStrip, robotDrawingProperties.robot_path.size());
                    for(std::size_t i =0; i<robotDrawingProperties.robot_path.size();i++){
                        vertices[i] = *(robotDrawingProperties.robot_path[i]);
                    }
                    window.draw(vertices);
                }



                if(robotDrawingProperties.circleShape!=nullptr){
                    window.draw(*(robotDrawingProperties.circleShape));
                }
                else{
                    background_completed.store(true);
                }

                window.display();

                auto rendering_instance_2 = std::chrono::high_resolution_clock::now();
                auto rendering_duration = std::chrono::duration_cast<std::chrono::milliseconds>(rendering_instance_2 - rendering_instance_1).count();
                std::cout << "Graphic rendering time : " << rendering_duration << " ms" << std::endl;
            }
        }
        std::cout<<"Rendering is terminated."<<std::endl;

}



void deployWebSocket(ROSBridgeClient &client,const std::string webSocketAddress){
    client.connect(webSocketAddress); 
}




template <typename T, size_t N>
void point_generate_according_to_grid(const uint8_t &step_size,
                                const uint8_t &min_step_size,
                               std::array<T, N> distance_dataset_new,
                               std::array<T, N> angle_dataset_new,
                               std::array<Eigen::Vector2d,N> &point_cloud) {



    auto scalar = (min_step_size) / (double(step_size));

    /* Scaling the current dataset*/
    for (double &data : distance_dataset_new)
    {
        data = data * scalar;
    }

    /* generate point cloud*/
    scan_data_to_point_cloud(distance_dataset_new, angle_dataset_new,point_cloud);

}


void runLocalization(Eigen::Vector3d &robot_pos_old, LocalizationData &localizationData0,LocalizationData &localizationData1,LocalizationData &localizationData2,Eigen::Vector3d &robot_pos_new){

    /**************************** run localization for all maps ***************************/
//    auto start2 = std::chrono::high_resolution_clock::now();
    auto new_robot_pos_0 = localization<double,NUM_DATA >(Eigen::Vector3d(robot_pos_old(0)/4,robot_pos_old(1)/4,robot_pos_old(2)),
                                localizationData0.sensorProbabilities,
                                *localizationData0.occupancy_grid_map,
                                localizationData0.point_cloud,
                                localizationData0.num_x_cells, localizationData0.num_y_cells);


    
    auto new_robot_pos_1 = localization<double,NUM_DATA >(Eigen::Vector3d(new_robot_pos_0(0)*2,new_robot_pos_0(1)*2,new_robot_pos_0(2)),
                                localizationData1.sensorProbabilities,
                                *localizationData1.occupancy_grid_map,
                                localizationData1.point_cloud,
                                localizationData1.num_x_cells, localizationData1.num_y_cells);

    
    auto new_robot_pos_2 = localization<double,NUM_DATA >(Eigen::Vector3d(new_robot_pos_1(0)*2,new_robot_pos_1(1)*2,new_robot_pos_1(2)),
                                localizationData2.sensorProbabilities,
                                *localizationData2.occupancy_grid_map,
                                localizationData2.point_cloud,
                                localizationData2.num_x_cells, localizationData2.num_y_cells);



// auto start2 = std::chrono::high_resolution_clock::now();
// auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(start2 - start1).count();
// std::cout << "Execution time : " << duration1 << " ms" << std::endl;


robot_pos_new[0] = new_robot_pos_2[0];
robot_pos_new[1] = new_robot_pos_2[1];
robot_pos_new[2] = new_robot_pos_2[2];

}

template <typename T, size_t N>
Eigen::Vector3d localization(  Eigen::Vector3d robot_pos,
                    const SensorProbabilities &sensorProbabilities,
                    OccupancyGridMap &occupancyGridMap,
                    std::array<Eigen::Vector2d,N> &point_cloud,
                    const uint32_t &num_x_cells, const uint32_t &num_y_cells) {


    HectorSLAM hector_slam(occupancyGridMap.get_cells(), sensorProbabilities, num_x_cells, num_y_cells);
    std::vector<Eigen::Vector2d> vec4(point_cloud.begin(), point_cloud.end());
    auto current_robot_pos = hector_slam.runLocalizationLoop(robot_pos, vec4,50);
    // for(int i =0 ;i<10;i++){
    //     hector_slam.estimateTransformationLogLh(robot_pos,  vec4);
    // }

    Eigen::Vector3d new_pos_deg(current_robot_pos(0), current_robot_pos(1), current_robot_pos(2) * 360 / (2 * M_PI));
    std::cout << "new_pos : " << new_pos_deg.transpose() << std::endl;

    return current_robot_pos;
}


void runMapping(uint8_t grid_step_sizes[], std::array<double, NUM_DATA> &distance_dataset_old,std::array<double, NUM_DATA> &angle_dataset_old,std::vector<Cell> &cells, Eigen::Vector3d robot_pos_old ){
    /*
* NOTE: WE assume that all the data(distances, robot pos ..) are available in highest accuracy way
*/
    //auto start1 = std::chrono::high_resolution_clock::now();
    /********************************iteration 1**************************************/

    std::array<double, NUM_DATA> distance_dataset_old_0 = distance_dataset_old;
    std::array<double, NUM_DATA> angle_dataset_old_0 = angle_dataset_old;
    double robot_pos_old_0[] = {robot_pos_old[0], robot_pos_old[1], robot_pos_old[2]};
    SensorProbabilities sensorProbabilities_0 = {PROB_OCCUPIED,PROB_PRIOR,PROB_FREE};
    const uint32_t num_x_cells_0 = NUM_X_CELLS(grid_step_sizes[0]);
    const uint32_t num_y_cells_0 = NUM_Y_CELLS(grid_step_sizes[0]);
    static OccupancyGridMap occupancy_grid_map_0(sensorProbabilities_0, num_x_cells_0, num_y_cells_0);


    std::thread thread_0([&]() {
        
        mapping<double,NUM_DATA >(grid_step_sizes[0], grid_step_sizes[NUM_MAPS-1],
            distance_dataset_old_0, angle_dataset_old_0,
            robot_pos_old_0,
            occupancy_grid_map_0);
    });



    /********************************iteration 2**************************************/
    std::array<double, NUM_DATA> distance_dataset_old_1 = distance_dataset_old;
    std::array<double, NUM_DATA> angle_dataset_old_1 = angle_dataset_old;
    double robot_pos_old_1[] = {robot_pos_old[0], robot_pos_old[1], robot_pos_old[2]};
    SensorProbabilities sensorProbabilities_1 = {PROB_OCCUPIED,PROB_PRIOR,PROB_FREE};
    const uint32_t num_x_cells_1 = NUM_X_CELLS(grid_step_sizes[1]);
    const uint32_t num_y_cells_1 = NUM_Y_CELLS(grid_step_sizes[1]);
    static OccupancyGridMap occupancy_grid_map_1(sensorProbabilities_1, num_x_cells_1, num_y_cells_1);

    std::thread thread_1([&]() {
        mapping<double,NUM_DATA >(grid_step_sizes[1], grid_step_sizes[NUM_MAPS-1],
            distance_dataset_old_1, angle_dataset_old_1,
            robot_pos_old_1,
            occupancy_grid_map_1);
    });


    /********************************iteration 3**************************************/
    std::array<double, NUM_DATA> distance_dataset_old_2 = distance_dataset_old;
    std::array<double, NUM_DATA> angle_dataset_old_2 = angle_dataset_old;
    double robot_pos_old_2[] = {robot_pos_old[0], robot_pos_old[1], robot_pos_old[2]};
    SensorProbabilities sensorProbabilities_2 = {PROB_OCCUPIED,PROB_PRIOR,PROB_FREE};
    const uint32_t num_x_cells_2 = NUM_X_CELLS(grid_step_sizes[2]);
    const uint32_t num_y_cells_2 = NUM_Y_CELLS(grid_step_sizes[2]);
    static OccupancyGridMap occupancy_grid_map_2(sensorProbabilities_2, num_x_cells_2, num_y_cells_2);

    std::thread thread_2([&]() {
        mapping<double,NUM_DATA >(grid_step_sizes[2], grid_step_sizes[NUM_MAPS-1],
            distance_dataset_old_2, angle_dataset_old_2,
            robot_pos_old_2,
            occupancy_grid_map_2);
    });

 thread_0.join();
 thread_1.join();
 thread_2.join();

//auto start2 = std::chrono::high_resolution_clock::now();

localizationData0.sensorProbabilities = sensorProbabilities_0;

localizationData0.num_x_cells = num_x_cells_0;
localizationData0.num_y_cells = num_y_cells_0;


localizationData1.sensorProbabilities = sensorProbabilities_1;

localizationData1.num_x_cells = num_x_cells_1;
localizationData1.num_y_cells = num_y_cells_1;

localizationData2.sensorProbabilities = sensorProbabilities_2;

localizationData2.num_x_cells = num_x_cells_2;
localizationData2.num_y_cells = num_y_cells_2;

//auto start3 = std::chrono::high_resolution_clock::now();

localizationData0.occupancy_grid_map = &occupancy_grid_map_0;
localizationData1.occupancy_grid_map = &occupancy_grid_map_1;
localizationData2.occupancy_grid_map = &occupancy_grid_map_2;



cells = *(localizationData1.occupancy_grid_map->get_cells());

//  auto start4 = std::chrono::high_resolution_clock::now();
// auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(start2 - start1).count();
// auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(start3 - start2).count();
// auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(start4 - start3).count();
// std::cout << "Execution time mapping 1: " << duration1<< " ms" << std::endl;
// std::cout << "Execution time mapping 2: " << duration2<< " ms" << std::endl;
// std::cout << "Execution time mapping 3: " << duration3<< " ms" << std::endl;


}

template <typename T, size_t N>
void mapping(const uint8_t &step_size,
                               const uint8_t &min_step_size,
                               std::array<T, N> &distance_dataset_old,
                               std::array<T, N> &angle_dataset_old,
                               double *robot_pos_old,
                               OccupancyGridMap &occupancyGridMap) {


    /* Scaling the previous dataset*/
    auto scalar = (min_step_size) / (double(step_size));
    for (double &data : distance_dataset_old)
    {
        data = data * scalar;
    }

    /* Scaling the previous robot position*/
    robot_pos_old[0] = robot_pos_old[0] * scalar;
    robot_pos_old[1] = robot_pos_old[1] * scalar;
    /* initial mapping*/

    std::vector<double> vec1(distance_dataset_old.begin(), distance_dataset_old.end());
    std::vector<double> vec2(angle_dataset_old.begin(), angle_dataset_old.end());
    Eigen::Vector3d vec3(robot_pos_old[0], robot_pos_old[1],robot_pos_old[2]);
    occupancyGridMap.runOccupancyGridMap(vec1, vec2, vec3);


}


void updateDrawingProperties(std::vector<Cell> &cells, Eigen::Vector3d robot_pos, Grid &grid, RobotDrawingProperties &robotDrawingProperties){
    
    static Eigen::Vector3d pre_robot_pos(0,0,0);
    
    robotDrawingProperties.updatedCells.clear();
    robotDrawingProperties.cellsOfPreviousPosition.clear();

    if(!robotDrawingProperties.robot_path.empty()){
        /*clear the previous robot location*/
        for(size_t i = 0; i<7;i++){
            for(size_t j = 0; j<7;j++){
                /*clear 49 elements*/
                auto x = round(pre_robot_pos(0)/2) + j -3;
                auto y = round(pre_robot_pos(1)/2) + i -3;
                auto prob = cells[int((y-1)*GRID_WIDTH/4 + (x-1))].prob_occupied;
                //std::cout<<x <<" "<<y<<std::endl;
                robotDrawingProperties.cellsOfPreviousPosition.emplace_back(grid.draw_cell(x,y,sf::Color(255*(1-prob),255*(1-prob),255*(1-prob))));
            }

        }
    }

    /* collect only updated cells for draw*/
    for(size_t i=0;i<cells.size();i++){
        if (pre_cells[i].prob_occupied != cells[i].prob_occupied){
            robotDrawingProperties.updatedCells.emplace_back(grid.draw_cell(cells[i].x,cells[i].y,sf::Color(255*(1-cells[i].prob_occupied),255*(1-cells[i].prob_occupied),255*(1-cells[i].prob_occupied))));
            pre_cells[i].prob_occupied = cells[i].prob_occupied;
        }
    }

    //std::cout<<"updated cells : "<<updatedCells.size()<<std::endl;
    /*create robot indicator*/
    grid.update_circle(robot_pos(0)/2 , robot_pos(1)/2,robotDrawingProperties.circleShape);

    /* update robot route continuously*/
    robotDrawingProperties.robot_path.emplace_back(grid.draw_point(robot_pos(0)/2 ,robot_pos(1)/2));

    /* release the lock the main thread*/
    render_changes.store(true);    
    pre_robot_pos = robot_pos;

}


void initDrawingProperties(uint32_t num_x_cells,uint32_t num_y_cells, std::vector<sf::RectangleShape> &updatedCells,Grid &grid){
    
    updatedCells.clear();

    /* collect only updated cells for draw*/
    for(size_t i=1;i<=num_x_cells;i++){
        for(size_t j=1;j<=num_y_cells;j++){
            updatedCells.emplace_back(grid.draw_cell(i,j,sf::Color(255*(1-0.5),255*(1-0.5),255*(1-0.5))));
            Cell cell ={(int) i,(int)j,LOGS_ODDS_RATIO(0.5),LOGS_ODDS_RATIO(0.5),0.5};
            pre_cells.emplace_back(cell);
        }
    }

    /* release the lock the main thread*/
    render_changes.store(true);

}

bool isPosUpdated(Eigen::Vector3d robot_pos,Eigen::Vector3d robot_pos_old,Eigen::Vector3d threshold){
    return (abs(robot_pos_old(0) - robot_pos(0)) > threshold(0)) ||
    (abs(robot_pos_old(1) - robot_pos(1)) > threshold(1)) ||
    (abs(robot_pos_old(2) - robot_pos(2)) > threshold(2));

}

Eigen::Vector3d robot_pos_init(void){
    double robot_pos_x_old = (GRID_WIDTH / 2) / grid_step_sizes[NUM_MAPS - 1];
    double robot_pos_y_old = (GRID_HEIGHT / 2) / grid_step_sizes[NUM_MAPS - 1];
    double robot_pos_theta_old = 0;

    return Eigen::Vector3d(robot_pos_x_old,robot_pos_y_old,robot_pos_theta_old);
}