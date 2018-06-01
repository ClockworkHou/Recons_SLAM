#ifndef MAP_H
#define MAP_H

#include "../include_libs.h"
#include "Frame.h"
#include "MapPoint.h"

namespace slam_class
{
class Map
{
public:
    
    unordered_map<unsigned long, MapPoint* >  map_points;        // all landmarks
    unordered_map<unsigned long, Frame*>     key_frames;             // all key-frames

    Map() {}
    
    void insertKeyFrame( Frame* frame );
    void insertMapPoint( MapPoint* map_point );
};


}

#endif