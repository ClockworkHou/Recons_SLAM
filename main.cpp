#include <iostream>

#include"Include/include_libs.h"
#include"Include/slam_class/Camera.h"
#include"Include/slam_class/config_hc.h"
#include"Include/slam_class/Frame.h"
#include"Include/slam_class/MapPoint.h"
#include"Include/slam_class/Map.h"
#include"Include/slam_class/Input_Handler.h"
#include"Include/slam_class/Triangulation_Solver.h"
#include"Include/slam_class/Pnp_Solver.h"
#include"Include/slam_class/Map_Viewer.h"

using namespace std;
using namespace slam_class;

slam_class::Map Global_Map;
slam_class::Map Local_Map;
Input_Handler input_handler;
Triangulation_Solver triangulation_solver;
Pnp_Solver  pnp_solver;
Map_Viewer map_viewer;

int main(int argc, char **argv) 
{
    triangulation_solver.Global_Map = &Global_Map;
    triangulation_solver.Local_Map = &Local_Map;
    pnp_solver.Global_Map = &Global_Map;
    pnp_solver.Local_Map = &Local_Map;
    
    cout << "开始读入文件!" << endl;
    
    cout << "开始处理..."<< endl;
    
    triangulation_solver.initialize();
    
    pnp_solver.run();
    
    cout << "开始显示地图..." << endl;
    
    
    
    return 0;
}
