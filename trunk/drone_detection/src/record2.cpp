#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv/cv.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
//#include <time.h>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings; 
static const char WINDOW[] = "PUJ";
cv_bridge::CvImagePtr cv_ptr;
int imageready=0;
 
 
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
// Acceder a la camara del AR-Drone por medio de ros
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      imageready=2;
      return;
    }

  imageready=1;

   // cv::imshow(WINDOW, cv_ptr->image);
   // cv::waitKey(1);
   //image_pub_.publish(cv_ptr->toImageMsg());
}




int main( int argc, char** argv )
{

ros::init(argc, argv,"image_converter");

ros::NodeHandle nh_;
ros::NodeHandle camImage;
cout << "before subscriber" << endl ;

ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, imageCb);
 cout << "After subscriber" << endl ;

ros::Rate loop_rate(15);  //simulate 15 fps


 
while(imageready==0)
	{
		ROS_INFO(" Waitting for image to be ready\n");
		cout << "Waitting for image to be ready" << endl ;
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	int isColor = 1;
	int fps     = 15;  
	int frameW  = 640; //Para la imagen del Drone
	int frameH  = 360; //Para la imagen del Drone
	CvSize size;
	size.width = frameW;
	size.height = frameH;
	
	CvVideoWriter *writer = cvCreateVideoWriter(
		    "/home/caro/fuerte_workspace/sandbox/drone_detection/video/video.avi",
		    CV_FOURCC('M','J','P','G'),fps,size,isColor);
  	
       
	while(imageready=!2)
	    {
	    Mat IM=cv_ptr->image;
	    IplImage img = IM;
	    cvWriteFrame(writer,&img);      // add the frame to the file
	    imshow("Video", IM);
	    waitKey(1);
	    ros::spinOnce();
	    loop_rate.sleep();
	    //cout << "contador "<<counter<< endl ;
	    }

cvReleaseVideoWriter(&writer);
cout << "sali del for!!!" << endl;
    
return 0;
}
