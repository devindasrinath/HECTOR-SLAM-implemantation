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

#define GRID_WIDTH 840
#define GRID_HEIGTH 840

#define NUM_DATA_SETS 200

int frame_num = 0;
bool first_data_set = true;
double circle_points[NUM_DATA_SETS][2] = {
    {30.0, 0.0},
    {29.985047673785214, 0.9470564929443158},
    {29.940205599944804, 1.8931689393802094},
    {29.865518478033177, 2.837394233845348},
    {29.761060757797733, 3.778791152031527},
    {29.626936564965714, 4.716421289017523},
    {29.46327959744941, 5.649349994691536},
    {29.270252992073207, 6.57664730543074},
    {29.048049161955216, 7.4973888711092505},
    {28.79688960470571, 8.410656876510433},
    {28.517024681633487, 9.315540956225064},
    {28.20873336818027, 10.211139102123385},
    {27.872322975831917, 11.096558562496405},
    {27.508128845783663, 11.97091673197023},
    {27.116514014664705, 12.833342031306287},
    {26.69786885265541, 13.682974776210479},
    {26.25261067435781, 14.518968034285162},
    {25.781183322807358, 15.340488469269802},
    {25.284056727040518, 16.146717171728657},
    {24.761726433659287, 16.93685047535754},
    {24.21471311285956, 17.71010075809585},
    {23.643562039415773, 18.46569722724538},
    {23.048842549139124, 19.202886687813212},
    {22.431147471351274, 19.920934293312936},
    {21.791092537939182, 20.619124278275528},
    {21.129315769580163, 21.296760671739985},
    {20.446476839749014, 21.953167991012304},
    {19.74325641714113, 22.58769191500131},
    {19.02035548716716, 23.19969993646017},
    {18.278494653195498, 23.78858199248336},
    {17.518413418239188, 24.353751072630622},
    {16.740869447803206, 24.894643804071773},
    {15.946637814627042, 25.410721013168967},
    {15.136510226075325, 25.9014682629367},
    {14.311294234946686, 26.366396365843812},
    {13.471812434487605, 26.80504187144622},
    {12.618901638413526, 27.216967528364442},
    {11.753412046754795, 27.601762720145235},
    {10.876206398358738, 27.959043874573045},
    {9.988159110892832, 28.28845484602311},
    {9.090155409206183, 28.589667270475175},
    {8.183090442918097, 28.862380892833887},
    {7.267868394113496, 29.10632386622959},
    {6.345401576034572, 29.321253023001216},
    {5.416609523667069, 29.506954117091095},
    {4.482418077127829, 29.66324203761008},
    {3.5437584587672415, 29.78996099336009},
    {2.601566344906522, 29.886984668130204},
    {1.656780933135245, 29.954216346611364},
    {0.710344006098723, 29.99158901080434},
    {-0.2368010072913993, 29.99906540682469},
    {-1.183709972287581, 29.97663808203827},
    {-2.12943898944048, 29.92432939249016},
    {-3.073045335498399, 29.842191480619707},
    {-4.013588403134371, 29.730306223283762},
    {-4.950130638563979, 29.588785150140083},
    {-5.88173847611934, 29.417769332472123},
    {-6.807483268847701, 29.217429242566133},
    {-7.726442214206902, 28.987964583780656},
    {-8.637699273934992, 28.72960409147789},
    {-9.540346087177143, 28.442605305015285},
    {-10.433482875959472, 28.127254311024753},
    {-11.316219342107278, 27.783865458235237},
    {-12.187675554713671, 27.4127810441231},
    {-13.046982827273789, 27.01437097370255},
    {-13.89328458361043, 26.589032390796262},
    {-14.725737211727806, 26.13718928215379},
    {-15.543510904742327, 25.659292054812354},
    {-16.345790488052113, 25.15581708712132},
    {-17.131776231920842, 24.627266253877877},
    {-17.900684648665678, 24.07416642604734},
    {-18.65174927365485, 23.497068945566717},
    {-19.384221429336296, 22.896549075755054},
    {-20.097370971535657, 22.273205427878494},
    {-20.790487017279954, 21.62765936444149},
    {-21.462878653421136, 20.9605543797992},
    {-22.113875625353366, 20.272555458708265},
    {-22.74282900513736, 19.564348413455605},
    {-23.3491118383659, 18.83663920022575},
    {-23.932119769125578, 18.090153215388437},
    {-24.491271642431972, 17.325634572407658},
    {-25.026010083537525, 16.543845360093226},
    {-25.535802053534837, 15.745564882934053},
    {-26.020139380701444, 14.93158888427049},
    {-26.47853926705641, 14.102728753080088},
    {-26.910544769623904, 13.259810715167374},
    {-27.315725255923866, 12.403675009564026},
    {-27.69367683323582, 11.53517505096033},
    {-28.044022751207947, 10.655176579002859},
    {-28.366413777410013, 9.764556795306348},
    {-28.660528545455843, 8.8642034890401},
    {-28.92607387534835, 7.955014151960362},
    {-29.162785065727707, 7.037895083771066},
    {-29.370426157731433, 6.11376048870457},
    {-29.548790170203315, 5.1835315642229824},
    {-29.697699306016712, 4.248135582748548},
    {-29.817005129306633, 3.3085049673382523},
    {-29.906588713433795, 2.365576362224298},
    {-29.966360759533305, 1.4202896991467462},
    {-29.99626168552972, 0.4735872604091046},
    {-29.99626168552972, -0.47358726040911053},
    {-29.966360759533305, -1.4202896991467389},
    {-29.906588713433795, -2.3655763622242905},
    {-29.817005129306636, -3.3085049673382447},
    {-29.697699306016712, -4.248135582748553},
    {-29.548790170203315, -5.183531564222989},
    {-29.370426157731437, -6.113760488704563},
    {-29.162785065727707, -7.037895083771059},
    {-28.926073875348354, -7.955014151960355},
    {-28.660528545455847, -8.864203489040094},
    {-28.366413777410013, -9.764556795306355},
    {-28.04402275120795, -10.655176579002852},
    {-27.693676833235823, -11.535175050960323},
    {-27.315725255923866, -12.403675009564019},
    {-26.910544769623908, -13.259810715167367},
    {-26.478539267056405, -14.102728753080093},
    {-26.02013938070144, -14.931588884270495},
    {-25.53580205353484, -15.74556488293405},
    {-25.02601008353753, -16.54384536009322},
    {-24.491271642431975, -17.325634572407655},
    {-23.932119769125585, -18.09015321538843},
    {-23.349111838365896, -18.836639200225754},
    {-22.742829005137363, -19.564348413455598},
    {-22.11387562535337, -20.27255545870826},
    {-21.462878653421143, -20.96055437979919},
    {-20.790487017279958, -21.627659364441485},
    {-20.097370971535653, -22.273205427878498},
    {-19.384221429336304, -22.896549075755047},
    {-18.651749273654858, -23.497068945566713},
    {-17.90068464866567, -24.07416642604734},
    {-17.13177623192085, -24.627266253877874},
    {-16.34579048805211, -25.155817087121324},
    {-15.543510904742332, -25.65929205481235},
    {-14.725737211727813, -26.137189282153788},
    {-13.893284583610425, -26.589032390796266},
    {-13.046982827273796, -27.014370973702547},
    {-12.187675554713666, -27.4127810441231},
    {-11.31621934210729, -27.783865458235233},
    {-10.433482875959474, -28.127254311024753},
    {-9.540346087177136, -28.442605305015288},
    {-8.637699273934999, -28.729604091477885},
    {-7.7264422142068945, -28.98796458378066},
    {-6.807483268847714, -29.21742924256613},
    {-5.881738476119341, -29.417769332472123},
    {-4.950130638563967, -29.588785150140083},
    {-4.013588403134378, -29.730306223283762},
    {-3.073045335498393, -29.84219148061971},
    {-2.1294389894404873, -29.92432939249016},
    {-1.1837099722875815, -29.97663808203827},
    {-0.23680100729141332, -29.99906540682469},
    {0.7103440060987157, -29.99158901080434},
    {1.6567809331352508, -29.954216346611364},
    {2.6015663449065145, -29.886984668130204},
    {3.543758458767241, -29.78996099336009},
    {4.482418077127814, -29.66324203761008},
    {5.416609523667063, -29.506954117091098},
    {6.345401576034578, -29.321253023001216},
    {7.267868394113488, -29.10632386622959},
    {8.183090442918095, -28.862380892833887},
    {9.090155409206169, -28.589667270475182},
    {9.988159110892832, -28.28845484602311},
    {10.876206398358741, -27.959043874573045},
    {11.753412046754788, -27.601762720145235},
    {12.61890163841353, -27.21696752836444},
    {13.471812434487592, -26.805041871446228},
    {14.311294234946686, -26.366396365843812},
    {15.136510226075332, -25.901468262936696},
    {15.946637814627035, -25.410721013168967},
    {16.74086944780321, -24.894643804071773},
    {17.518413418239177, -24.35375107263063},
    {18.278494653195498, -23.78858199248336},
    {19.020355487167166, -23.199699936460167},
    {19.743256417141126, -22.587691915001315},
    {20.446476839749018, -21.9531679910123},
    {21.12931576958016, -21.296760671739992},
    {21.79109253793918, -20.619124278275528},
    {22.431147471351263, -19.92093429331295},
    {23.048842549139117, -19.20288668781322},
    {23.643562039415777, -18.46569722724537},
    {24.214713112859553, -17.710100758095862},
    {24.761726433659287, -16.93685047535754},
    {25.28405672704051, -16.14671717172867},
    {25.781183322807355, -15.340488469269806},
    {26.252610674357815, -14.518968034285157},
    {26.697868852655404, -13.682974776210486},
    {27.116514014664705, -12.833342031306286},
    {27.508128845783656, -11.970916731970242},
    {27.872322975831914, -11.096558562496408},
    {28.20873336818027, -10.21113910212338},
    {28.517024681633483, -9.315540956225073},
    {28.79688960470571, -8.41065687651043},
    {29.048049161955213, -7.497388871109264},
    {29.270252992073207, -6.576647305430743},
    {29.46327959744941, -5.649349994691528},
    {29.62693656496571, -4.716421289017532},
    {29.761060757797736, -3.7787911520315243},
    {29.865518478033174, -2.8373942338453615},
    {29.9402055999448, -1.8931689393802116},
    {29.985047673785214, -0.9470564929443073},
    {30.0, -7.34788079488412e-15}
};



struct LocalizationData {
    SensorProbabilities sensorProbabilities;
    OccupancyGridMap occupancy_grid_map;
    std::array<Eigen::Vector2d, NUM_DATA> point_cloud;
    uint32_t num_x_cells;
    uint32_t num_y_cells;
};

    LocalizationData localizationData0;
LocalizationData localizationData1;
LocalizationData localizationData2;


std::atomic_bool values_assigning_to_draw(false);

std::atomic_bool first_frame_completed(false);

std::atomic_bool map_updated(true);

std::atomic_bool background_completed(false);

bool end_of_program = false;

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



void drawUpdatedMap(std::vector<Cell> &cells, Eigen::Vector3d robot_pos, Grid &grid, std::vector<sf::RectangleShape> &updatedCells, sf::CircleShape *&circleShape,std::vector<sf::Vertex*> &robot_all_pos );

void drawNewMap(uint32_t num_x_cells,uint32_t num_y_cells, std::vector<sf::RectangleShape> &updatedCells,Grid &grid);

uint8_t grid_step_sizes[]={8,4,2};

std::vector<Cell> pre_cells;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// auto angle_min = -3.1415927410125732;
// auto angle_max = 3.1415927410125732;
// auto angle_increment = 0.012466637417674065;
// // time_increment: 0.000200034148292616
// // scan_time: 0.12302099913358688
// // range_min: 0.10000000149011612
// // range_max: 12.0
// double ranges[] = {0.0, 1.3990000486373901, 1.399999976158142, 1.4010000228881836, 1.4040000438690186, 1.406000018119812, 1.4119999408721924, 1.4160000085830688, 1.4179999828338623, 1.4210000038146973, 1.7910000085830688, 1.7960000038146973, 1.8020000457763672, 1.8049999475479126, 0.0, 2.0390000343322754, 0.0, 1.878000020980835, 1.847000002861023, 0.0, 0.0, 0.0, 1.6959999799728394, 1.687000036239624, 1.687999963760376, 0.0, 1.590999960899353, 1.5809999704360962, 1.5750000476837158, 1.565000057220459, 1.8070000410079956, 1.7899999618530273, 1.7760000228881836, 1.7690000534057617, 1.7640000581741333, 1.7740000486373901, 1.7860000133514404, 0.0, 2.184000015258789, 2.190000057220459, 2.2019999027252197, 2.2209999561309814, 2.2309999465942383, 0.0, 2.066999912261963, 2.0820000171661377, 2.1089999675750732, 2.124000072479248, 2.13700008392334, 2.1540000438690186, 2.171999931335449, 2.2079999446868896, 2.2219998836517334, 2.243000030517578, 2.257999897003174, 2.2829999923706055, 2.319000005722046, 2.3369998931884766, 2.365000009536743, 2.382999897003174, 2.4100000858306885, 2.4590001106262207, 2.4790000915527344, 2.5299999713897705, 0.0, 0.0, 3.3550000190734863, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1449999809265137, 1.1330000162124634, 1.1230000257492065, 1.1160000562667847, 1.1050000190734863, 1.0859999656677246, 1.0800000429153442, 1.0700000524520874, 1.0640000104904175, 1.055999994277954, 1.0399999618530273, 1.034999966621399, 1.0269999504089355, 1.0180000066757202, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9359999895095825, 0.8849999904632568, 0.8700000047683716, 0.859000027179718, 0.8489999771118164, 0.8339999914169312, 0.828000009059906, 0.8220000267028809, 0.8180000185966492, 0.8119999766349792, 0.8090000152587891, 0.8069999814033508, 0.8050000071525574, 0.8040000200271606, 0.8029999732971191, 0.8040000200271606, 0.8050000071525574, 0.8059999942779541, 0.8069999814033508, 0.8130000233650208, 0.8169999718666077, 0.8209999799728394, 0.8259999752044678, 0.8320000171661377, 0.8460000157356262, 0.8579999804496765, 0.8700000047683716, 0.8859999775886536, 0.0, 0.0, 0.0, 1.034999966621399, 1.0119999647140503, 0.996999979019165, 0.9879999756813049, 0.9750000238418579, 0.9710000157356262, 0.9660000205039978, 0.9629999995231628, 0.9610000252723694, 0.9610000252723694, 0.9639999866485596, 0.9639999866485596, 0.9670000076293945, 0.9760000109672546, 0.9829999804496765, 0.9919999837875366, 1.0019999742507935, 0.0, 1.1480000019073486, 1.1369999647140503, 1.1390000581741333, 1.1619999408721924, 1.187999963760376, 1.2020000219345093, 1.2280000448226929, 0.0, 1.2920000553131104, 1.284000039100647, 1.2760000228881836, 1.2280000448226929, 1.2029999494552612, 1.2359999418258667, 1.2239999771118164, 1.218000054359436, 1.1970000267028809, 0.0, 1.2369999885559082, 1.2569999694824219, 1.2669999599456787, 1.2979999780654907, 1.3079999685287476, 1.312999963760376, 0.0, 1.7569999694824219, 0.0, 1.6510000228881836, 1.7289999723434448, 0.0, 1.819000005722046, 1.8300000429153442, 1.840000033378601, 1.8680000305175781, 1.88100004196167, 1.8940000534057617, 1.906999945640564, 1.9249999523162842, 1.9589999914169312, 1.972000002861023, 1.9930000305175781, 2.00600004196167, 2.0239999294281006, 0.0, 1.6799999475479126, 1.659000039100647, 1.625, 1.6080000400543213, 1.590000033378601, 1.5770000219345093, 1.559000015258789, 1.531000018119812, 1.5169999599456787, 1.5049999952316284, 1.4889999628067017, 1.4639999866485596, 1.4520000219345093, 1.440000057220459, 1.4270000457763672, 1.4049999713897705, 1.3940000534057617, 1.3839999437332153, 1.3739999532699585, 1.3639999628067017, 1.343999981880188, 1.3350000381469727, 1.3279999494552612, 1.3179999589920044, 1.3029999732971191, 1.2940000295639038, 1.2869999408721924, 1.277999997138977, 1.2719999551773071, 1.2589999437332153, 1.2519999742507935, 1.246000051498413, 1.2400000095367432, 1.2280000448226929, 1.222000002861023, 1.215999960899353, 1.2100000381469727, 1.2059999704360962, 1.1970000267028809, 1.1920000314712524, 1.1859999895095825, 1.1820000410079956, 1.1779999732971191, 1.1710000038146973, 1.1679999828338623, 1.1619999408721924, 1.159999966621399, 1.152999997138977, 1.1510000228881836, 1.1480000019073486, 1.1440000534057617, 1.1419999599456787, 1.1369999647140503, 1.1339999437332153, 1.1319999694824219, 1.128999948501587, 1.128999948501587, 1.1239999532699585, 1.1230000257492065, 1.121000051498413, 0.0, 0.0, 1.5870000123977661, 1.6069999933242798, 1.6059999465942383, 1.6050000190734863, 1.6069999933242798, 1.6069999933242798, 1.6050000190734863, 1.5219999551773071, 1.531999945640564, 1.562000036239624, 1.5779999494552612, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1629999876022339, 1.1699999570846558, 1.1720000505447388, 1.1679999828338623, 1.1670000553131104, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1540000438690186, 1.125, 1.1299999952316284, 1.1369999647140503, 1.1440000534057617, 1.1540000438690186, 1.1670000553131104, 1.1929999589920044, 1.2059999704360962, 1.2319999933242798, 1.2319999933242798, 1.253000020980835, 1.2860000133514404, 1.3040000200271606, 1.319000005722046, 0.0, 1.409000039100647, 1.4170000553131104, 1.4359999895095825, 1.4479999542236328, 1.4589999914169312, 1.4700000286102295, 1.4789999723434448, 1.5019999742507935, 1.5149999856948853, 1.5260000228881836, 1.5360000133514404, 1.5490000247955322, 1.5750000476837158, 1.590999960899353, 1.6269999742507935, 1.61899995803833, 0.0, 0.0, 1.684000015258789, 1.7089999914169312, 1.7230000495910645, 1.7480000257492065, 0.671999990940094, 0.6639999747276306, 0.6359999775886536, 0.628000020980835, 0.6330000162124634, 0.6470000147819519, 0.652999997138977, 0.6700000166893005, 0.6800000071525574, 0.6840000152587891, 0.6850000023841858, 0.6840000152587891, 0.6880000233650208, 0.6909999847412109, 0.6919999718666077, 0.6880000233650208, 0.6840000152587891, 0.6779999732971191, 0.6700000166893005, 0.6690000295639038, 0.6710000038146973, 0.6800000071525574, 0.6940000057220459, 0.0, 0.0, 1.1690000295639038, 1.149999976158142, 1.1319999694824219, 1.1150000095367432, 1.0800000429153442, 1.065000057220459, 1.0499999523162842, 1.034999966621399, 0.9739999771118164, 0.9459999799728394, 0.9340000152587891, 0.9139999747276306, 0.9100000262260437, 0.9210000038146973, 0.9290000200271606, 0.9359999895095825, 0.0, 0.0, 2.3610000610351562, 0.0, 0.0, 2.4030001163482666, 2.3420000076293945, 2.3489999771118164, 0.0, 2.316999912261963, 2.318000078201294, 2.2909998893737793, 2.259999990463257, 2.23799991607666, 2.2190001010894775, 2.180999994277954, 2.1530001163482666, 2.188999891281128, 1.9859999418258667, 1.9739999771118164, 1.965999960899353, 1.965999960899353, 1.965999960899353, 1.9620000123977661, 1.5829999446868896, 0.0, 1.9249999523162842, 1.9249999523162842, 0.0, 0.0, 1.805999994277954, 0.0, 2.234999895095825, 0.0, 0.0, 1.4550000429153442, 1.440000057220459, 1.434999942779541, 1.4299999475479126, 1.4270000457763672, 1.4220000505447388, 1.4140000343322754, 1.4110000133514404, 1.409000039100647, 1.406000018119812, 1.4010000228881836, 1.3990000486373901, 1.3980000019073486, 1.3969999551773071, 1.3960000276565552, 1.3949999809265137, 1.3940000534057617, 1.3930000066757202, 1.3949999809265137, 1.3940000534057617, 1.3940000534057617, 1.3949999809265137, 1.3960000276565552, 1.3960000276565552};

int main()
{


    pre_cells.reserve((GRID_WIDTH*GRID_WIDTH)/(grid_step_sizes[2]*grid_step_sizes[2]));
    

    /*************************************************** create window and grid ******************************************************/
    sf::RenderWindow window(sf::VideoMode(GRID_WIDTH, GRID_HEIGTH), "SFML Test Grid");

    GridParameters grid_paramters = {GRID_HEIGTH, GRID_WIDTH, grid_step_sizes[NUM_MAPS-1], std::make_pair(grid_step_sizes[NUM_MAPS-1],grid_step_sizes[NUM_MAPS-1]),sf::Color(0,100,0) };
    Grid grid(grid_paramters);

    std::vector<sf::Vertex*> robot_path;
    std::vector<sf::RectangleShape> updatedCells;
    sf::CircleShape *circleShape = nullptr;

        std::thread thread_3([&]() {
        while (window.isOpen() && (!end_of_program)) {
            sf::Event event;

            while (window.pollEvent(event)) {
                // Handle different event types
                switch (event.type) {
                    case sf::Event::Closed:
                        end_of_program =true;
                        window.close();
                        break;
                    default:
                        break; // Handle other events if needed
                }
            }

            if (values_assigning_to_draw.load() && (!end_of_program) &&(!map_updated.load())) {
                
                map_updated.store(true);
                
                //auto start1 = std::chrono::high_resolution_clock::now();  

                for (auto &cell : updatedCells) {
                    window.draw(cell);
                }

                if(robot_path.size()>=2){
                    sf::VertexArray vertices(sf::LineStrip, robot_path.size());
                    for(std::size_t i =0; i<robot_path.size();i++){
                        vertices[i] = *(robot_path[i]);
                    }
                    window.draw(vertices);
                }

                if(circleShape!=nullptr){
                    //window.draw(*circleShape);
                }
                else{
                    background_completed.store(true);
                }

                std::cout<<frame_num<<std::endl;
                window.display();
            }
        }
        std::cout<<"come to end"<<std::endl;
    });










    first_data_set = true;
    std::vector<Cell> cells = {};
    uint32_t num_cells = (GRID_WIDTH*GRID_HEIGTH)/(grid_step_sizes[2]*grid_step_sizes[2]);
    uint32_t num_x_cells = (GRID_WIDTH)/grid_step_sizes[2];
    uint32_t num_y_cells = (GRID_HEIGTH)/grid_step_sizes[2];
    cells.reserve(num_cells);
    DatasetGenerator<NUM_DATA> datasetGenerator(120);

    std::array<Eigen::Vector2d,NUM_DATA> point_cloud_0;
    std::array<Eigen::Vector2d,NUM_DATA> point_cloud_1;
    std::array<Eigen::Vector2d,NUM_DATA> point_cloud_2;

    Eigen::Vector3d robot_pos;
    int data_set_index = 0;
    background_completed.store(false);
    drawNewMap(num_x_cells,num_y_cells, updatedCells,grid);
    while(!background_completed.load());

    ROSBridgeClient client;
    client.connect("ws://localhost:9090"); // Replace with your ROSBridge server URI


    while(true){

        if(client.message_received)
        /* generate or retrieved  */
        {  
            client.message_received = false;

            if(data_set_index>=NUM_DATA_SETS){
                break;
            }
            datasetGenerator.generateData(std::make_pair(circle_points[data_set_index][0]*2, circle_points[data_set_index][1]*2), 0);
            auto distance_data = datasetGenerator.getDistanceData();
            auto angle_data = datasetGenerator.getAngleData();
            data_set_index++;


            /* run localization*/
            if(first_data_set == true){
                /* no need of find the postion since awe set it as 0,0,0*/
                double robot_pos_x_old = (GRID_WIDTH / 2) / grid_step_sizes[NUM_MAPS - 1];
                double robot_pos_y_old = (GRID_HEIGTH / 2) / grid_step_sizes[NUM_MAPS - 1];
                double robot_pos_theta_old = 0;
                robot_pos(0) = robot_pos_x_old + circle_points[0][0];
                robot_pos(1) = robot_pos_y_old + circle_points[0][1];
                robot_pos(2) = robot_pos_theta_old;
                first_data_set = false;
            }
            else{
                /* generate point clound data and run algorithm ofr localization*/
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

            runMapping(grid_step_sizes, distance_data,angle_data, cells,robot_pos );
            
            drawUpdatedMap(cells, robot_pos, grid, updatedCells, circleShape,robot_path);

            //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        

    }


    thread_3.join();

    delete circleShape;

    return 0;
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
                                localizationData0.occupancy_grid_map,
                                localizationData0.point_cloud,
                                localizationData0.num_x_cells, localizationData0.num_y_cells);


    
    auto new_robot_pos_1 = localization<double,NUM_DATA >(Eigen::Vector3d(new_robot_pos_0(0)*2,new_robot_pos_0(1)*2,new_robot_pos_0(2)),
                                localizationData1.sensorProbabilities,
                                localizationData1.occupancy_grid_map,
                                localizationData1.point_cloud,
                                localizationData1.num_x_cells, localizationData1.num_y_cells);

    
    auto new_robot_pos_2 = localization<double,NUM_DATA >(Eigen::Vector3d(new_robot_pos_1(0)*2,new_robot_pos_1(1)*2,new_robot_pos_1(2)),
                                localizationData2.sensorProbabilities,
                                localizationData2.occupancy_grid_map,
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

localizationData0.sensorProbabilities = sensorProbabilities_0;
localizationData0.occupancy_grid_map = occupancy_grid_map_0;
localizationData0.num_x_cells = num_x_cells_0;
localizationData0.num_y_cells = num_y_cells_0;


localizationData1.sensorProbabilities = sensorProbabilities_1;
localizationData1.occupancy_grid_map = occupancy_grid_map_1;
localizationData1.num_x_cells = num_x_cells_1;
localizationData1.num_y_cells = num_y_cells_1;

localizationData2.sensorProbabilities = sensorProbabilities_2;
localizationData2.occupancy_grid_map = occupancy_grid_map_2;
localizationData2.num_x_cells = num_x_cells_2;
localizationData2.num_y_cells = num_y_cells_2;

cells = *(localizationData2.occupancy_grid_map.get_cells());

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


void drawUpdatedMap(std::vector<Cell> &cells, Eigen::Vector3d robot_pos, Grid &grid, std::vector<sf::RectangleShape> &updatedCells, sf::CircleShape *&circleShape,std::vector<sf::Vertex*> &robot_path){
    
    /* lock the main thread*/
    values_assigning_to_draw.store(false);

    updatedCells.clear();

    /* collect only updated cells for draw*/
    for(size_t i=0;i<cells.size();i++){
        if (pre_cells[i].prob_occupied != cells[i].prob_occupied){
            updatedCells.emplace_back(grid.draw_cell(cells[i].x,cells[i].y,sf::Color(255*(1-cells[i].prob_occupied),255*(1-cells[i].prob_occupied),255*(1-cells[i].prob_occupied))));
            pre_cells[i].prob_occupied = cells[i].prob_occupied;
        }
    }
    
    /*create robot indicator*/
    circleShape =  grid.draw_circle(robot_pos(0) , robot_pos(1), 3);

    /* update robot route continuously*/
    robot_path.emplace_back(grid.draw_point(robot_pos(0) ,robot_pos(1)));

    /* release the lock the main thread*/
    values_assigning_to_draw.store(true);    
    map_updated.store(false);
    frame_num++;

}


void drawNewMap(uint32_t num_x_cells,uint32_t num_y_cells, std::vector<sf::RectangleShape> &updatedCells,Grid &grid){
    
    /* lock the main thread*/
    values_assigning_to_draw.store(false);

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
    values_assigning_to_draw.store(true);
    map_updated.store(false);
    frame_num=1;

}