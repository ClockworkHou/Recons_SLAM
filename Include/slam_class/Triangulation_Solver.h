//calcualte initial map with 8-points/5-points algorithm

#ifndef Triangulation_Solver_H
#define Triangulation_Solver_H

#include"../include_libs.h"
#include"Map.h"

namespace slam_class
{
	
class Triangulation_Solver
{
public:
	Map* Global_Map;
	Map* Local_Map;
	
	void run();
	
	
};

}

#endif