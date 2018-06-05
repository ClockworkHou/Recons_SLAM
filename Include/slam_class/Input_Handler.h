//read  a video file/image set  into the program and save frames/images as the object "Frame"

#ifndef INPUT_HANDLER_H
#define INPUT_HANDLER_H

#include"../include_libs.h"
#include"Map.h"
#include"Camera.h"

namespace slam_class
{
	
class Input_Handler
{
public:
	Map* Global_Map;
	
	void run(char* filename);
};



}
#endif