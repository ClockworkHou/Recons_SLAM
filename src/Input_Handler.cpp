
#include"../Include/slam_class/Input_Handler.h"
#include"../Include/slam_class/config_hc.h"

namespace slam_class 
{

void Input_Handler::run(char* filename)
{  
    //读入视频  
    VideoCapture capture(filename);  
  
    //循环显示每一帧  
    int flag=0;
    int cnt = 0;
    while (1)  
    {  
        Mat img;//定义一个Mat变量，用于存储每一帧的图像  
        capture >> img;  //读取当前帧  
        
         //若视频播放完成，退出循环  
        if (img.empty())  
        {  
            break;  
        }
       flag += 1;
  	if (flag==5)
        {
           flag=0;
           // imshow("读取视频", frame);  //显示当前帧
	    SE3 * tmpse3 = new SE3();
	    Camera * tmpcam = new Camera();
	    tmpcam->intrinsics.fx =  camera_Intrinsics_FX;
	    tmpcam->intrinsics.fy =  camera_Intrinsics_FY;
	    tmpcam->intrinsics.cx =  camera_Intrinsics_CX;
	    tmpcam->intrinsics.cy =  camera_Intrinsics_CY;
	    Frame* tmp_f = new Frame(cnt++, 0, *tmpse3, tmpcam, img.clone(), true);
	    
	    Global_Map->insertKeyFrame(tmp_f);
            //FrameList.push_back(frame);
        }
        //waitKey(50);  //延时 
    }  
    return;  
};


}
	
	
	
	
