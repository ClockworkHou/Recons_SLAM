# Recons_SLAM
# Created by Hou Chen & Zhou Haotian from Peking Univ.

This is a simple(toy) real-time monocular SLAM system.

The system runs on a linux environment. Ubuntu 14.04 is ok.

3rd Party Libraries:
1. Eigen2
2. Ceres
3. g2o
4. Sophus
5. opencv 3.1.0
6. pcl 1.7

The system now have:

1.An input handler which reads a .mp4/.avi file from fimesystem and save all the frames into the memory.

2.Global Map & Local Map with cloud points as map point and frames to record camera motion.

3.An initializer using ORB features and two view geometory to builde an initial map.

4.A map builder using ORB features and two view geometory to track keyframes(simple choosing: every n frames) and add new cloud points.

5.A map viewer which reads information from Global map and save .pcd file in the filesystem.


Things to be done:
1. Bundle Adjustment
2. Elaborate & fine map point filtering methods.
3. Loop detection & fusion.
4. Parallel computing.
  (ORB SLAM system has three threads:
  one tracking cameras, 
  one building map, 
  and one handling loop fusion.) 
