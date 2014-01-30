 /**

 *  record.cpp
 *  This file is part of drone_detection
 *  Created by: Juan Pablo Rodríguez y Carolina Castiblanco
 *  Information: j_rodriguezg@javeriana.edu.co   jenny.castiblanco@javeriana.edu.co
 *  
 *  In this file is the program in charge of create a video file with the images from the topic /ardrone/image_raw. The created file has  
 *  .AVI format and is saved in the folder video of drone_detection package.


 */

/* Includes para ROS y OpenCV */
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


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings; 

/* Global variables of the system */
static const char WINDOW[] = "PUJ";
cv_bridge::CvImagePtr cv_ptr;
int imageready=0;
 
/*Funcion Image_Upload  
 
 * This function is in charge to obtain the image frome the topic /ardrone/image, it will be use for the algorithm to perform the video record

*/
 
void Image_Upload(const sensor_msgs::ImageConstPtr& msg){

/* The follow instruction is use to save the obtained image in BGR format afte the transformation of sensor_msg format */

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

/* The variable imageready is activated (1) to avoid go inside of the first while() of the algorithm. The while() function is notify the user 	  that the image has not been loaded */

  imageready=1;  
}



int main( int argc, char** argv ){

/* The program begins with the function init() by ROS and the inscription of record in the CMakelists.text file*/
ros::init(argc, argv,"record");

/* Some variables of the ROS enviroment are created for  future use */
ros::NodeHandle nh_;
ros::NodeHandle camImage;

/* The suscription to the ros topic /ardrone/image_raw is created */
ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, Image_Upload);
/* The operation frequency of the program is defined */
ros::Rate loop_rate(15);  //simulate 15 fps 

/* The next variables are created to obtain the current date and hour */
time_t rawtime;
struct tm * timeinfo;
/* In Fecha are saved the location where al the video will be saved */
char Fecha [80];
/* The current date is obtained  */
time ( &rawtime );
timeinfo = localtime (&rawtime);
/* In Fecha is saved only year-month-day (%F) to be concatenaded with the address where the folder with the video will be created */
strftime (Fecha,80,"%F-%R",timeinfo);



/* This while is only used to publish a errror message if the image don't be upload */	
while(imageready!=1)
	{
		/* Se declara la variable parada */ 
		/* The parada variable is created */ 	 
	        char parada=waitKey(50);
		ROS_INFO(" Waitting for image to be ready\n");
		ros::spinOnce();
		loop_rate.sleep();

		/* This condition is used to stop the program if the r key is pressed*/
		if( parada=='r'){
			cout << "Adios!" << endl ;
			break;
				}
	}
	/* Some variables are created to future use in the program */
	int isColor = 1;
	int fps     = 15; 
	int frameW  = 640; 
	int frameH  = 360;
	CvSize size;
	size.width = frameW;
	size.height = frameH;
		
	/* The os, str and nombre are defined to assign to the video a name  with the current date and hour   */	
	ostringstream os;
	os <<"/home/dell-077/fuerte_workspace/sandbox/drone_detection/video/"<<Fecha<<".avi";
        std::string str = os.str();
	const char * nombre=str.c_str();
	
	/* CvVideoWriter is used to indicate the location and format of the video using "nombre" */
	CvVideoWriter *writer = cvCreateVideoWriter(nombre,CV_FOURCC('M','J','P','G'),fps,size,isColor);
	
  	/* The obtained images are saved using a infinte loop. The loop is over when the user press the r key over the video window */	
	while(1){
	    /* The parada variable is created */
	    char parada=waitKey(50);
	    /* The obtained image is saved in the IM variable*/
	    Mat IM=cv_ptr->image;
	    /* The image is converted to img format */
	    IplImage img = IM;
	    /* The image obtained is added to the writer variable */
	    cvWriteFrame(writer,&img);      // add the frame to the file
	     /* The obtained image is shown in special window named Video Window */
	    imshow("Video Window", IM);
	    /* The waitKey() is used to publish the image indefinitely */
	    waitKey(1);

	    /* This condition is use to stop the program when the key r is pressed */
	    if( parada=='r'){
		cvReleaseVideoWriter(&writer);
		break;
		}
	    /* If the condition is not satisfy the loop continues */
	    else {
		ros::spinOnce();
	        loop_rate.sleep();
		}
	    }
cout << "Fin del video!!!" << endl;    
return 0;
}
