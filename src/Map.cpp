
#include"../Include/slam_class/Map.h"

namespace slam_class 
{
void Map::insertKeyFrame( Frame* frame )
{
	 if ( key_frames.find(frame->id) == key_frames.end() )
	{
		key_frames.insert( make_pair(frame->id, frame) );
	}
	else
	{
		key_frames[ frame->id] = frame;
	}
};

void Map::insertMapPoint( MapPoint* map_point )
{
	 if ( map_points.find(map_point->id) == map_points.end() )
	{
		map_points.insert( make_pair(map_point->id, map_point) );
	}
	else 
	{
		map_points[map_point->id] = map_point;
	}
};
	
}