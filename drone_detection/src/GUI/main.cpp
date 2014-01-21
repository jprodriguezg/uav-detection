#include "detection_gui.h"
#include <QtGui>
#include <QApplication>
#include "ros/ros.h"

// Para ROS
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv/cv.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ardrone_autonomy/Navdata.h"
#include <ros/time.h>
//#include <time.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <sys/stat.h>


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings; 
static const char WINDOW[] = "PUJ";
cv_bridge::CvImagePtr cv_ptr;
int imageready=0;
int contador=0;
char filename;
char Directorio_Log [100];
int mina = 0;
int contador2=0;

void savenavdata(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	FILE *datanav;
 	char datalog[512];
	//strcpy(datalog,"log.txt");

	if ((datanav=fopen(Directorio_Log,"a+"))!=NULL)
	sprintf(datalog,"%d %f %f %f %d %f %f %f %f %f %f\n",contador2, navdataPtr->rotX,navdataPtr->rotY,navdataPtr->rotZ,navdataPtr->altd, navdataPtr->vx,navdataPtr->vy,navdataPtr->vz,navdataPtr->ax,navdataPtr->ay,navdataPtr->az);
	fprintf(datanav,"%s \n", datalog);
	{
   	//fwrite(&navdataPtr->altd,1,sizeof(navdataPtr),datanav);
	}
	fclose(datanav);

	// Para la GUI	

		detection_gui npp;
		char bufaltd[200];
		snprintf(bufaltd,200,"Estimated Altitude: %d ",(int)navdataPtr->altd);
		//gui->setEstimated_Altitude(std::string(bufaltd));
		npp.setEstimated_Altitude(std::string(bufaltd));
		
		char bufvelx[200];
		snprintf(bufvelx,200,"Linear Velocity x: %f ",(float)navdataPtr->vx);
		//gui->setLinear_Velocity_x(std::string(bufvelx));
		npp.setLinear_Velocity_x(std::string(bufvelx));

		char bufvely[200];
		snprintf(bufvely,200,"Linear Velocity y: %f ",(float)navdataPtr->vy);
		//gui->setLinear_Velocity_y(std::string(bufvely));
		npp.setLinear_Velocity_y(std::string(bufvely));

		char bufvelz[200];
		snprintf(bufvelz,200,"Linear Velocity y: %f ",(float)navdataPtr->vz);
		//gui->setLinear_Velocity_z(std::string(bufvelz));
		npp.setLinear_Velocity_z(std::string(bufvelz));

		char bufPitch[200];
		snprintf(bufPitch,200,"Pitch Angle: %f ",(float)navdataPtr->rotY);
		//gui->setPitch_Angle(std::string(bufPitch));
		//detection_gui::setPitch_Angle(std::string(bufPitch));

		char bufYaw[200];
		snprintf(bufYaw,200,"Yaw Angle: %f ",(float)navdataPtr->rotX);
		//gui->setYaw_Angle(std::string(bufYaw));
		npp.setYaw_Angle(std::string(bufYaw));

		char bufRoll[200];
		snprintf(bufRoll,200,"Roll Angle: %f ",(float)navdataPtr->rotZ);
		//gui->setRoll_Angle(std::string(bufRoll));
		npp.setRoll_Angle(std::string(bufRoll));

		char bufBP[200];
		snprintf(bufBP,200,"Battery Percent: %f ",(float)navdataPtr->batteryPercent);
		//gui->setBattery_Percent(std::string(bufBP)); 
		npp.setBattery_Percent(std::string(bufBP));  

 return;
  
}

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
      return;
    }

  imageready=1;

   // cv::imshow(WINDOW, cv_ptr->image);
   // cv::waitKey(1);
   //image_pub_.publish(cv_ptr->toImageMsg());
} 

/*
int main(int argc, char *argv[])
{
    
    		
		ros::init(argc, argv,"drone_detection_GUI");
		
		ros::NodeHandle nh_;
		ros::NodeHandle camImage;
		ros::NodeHandle navdata;
		//cout << "before subscriber" << endl ;

		//ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, imageCb);
		//cout << "After subscriber" << endl ;
		ros::Subscriber Nav = camImage.subscribe("/ardrone/navdata", 5, savenavdata);

		ros::Rate loop_rate(15);  //simulate 15 fps

		 //Creo el objeto a	
		 QApplication a(argc, argv);
		 //Creo el objeto w de la clase detection_gui
		 detection_gui w;

		  
		 //Muestro la GUI
		 w.show();

				while(imageready!=1)
				{
					char parada=waitKey(50);
					//ROS_INFO(" Waitting for image to be ready\n");
					//cout << "Waitting for image to be ready" << endl ;
					ros::spinOnce();
					loop_rate.sleep();
					if( parada=='r'){
						cout << "Adios!" << endl ;
						break;
							}
				}

	
			  for (;;)
			  {
				//contador2++;
				char parada=waitKey(50);
				// Leer la Imagen

				//Mat IM = imread(image, CV_LOAD_IMAGE_UNCHANGED);
	
				Mat IM=cv_ptr->image;
				//cv::imshow(WINDOW, cv_ptr->image);
				//cv::imshow(WINDOW, IM);
				//cv::waitKey(1);
				// Comprobar que la imagen asignada a IM no sea vacio
				//delay.sleep();
				//cout << "Delay!" << endl ;
				//deteccion(cv_ptr->image,Directorio,Directorio_Error);
	
			  	
				ros::spinOnce();
				loop_rate.sleep();
				if(parada=='r'){
					cout << "Adios!" << endl ;
					break;
					}
			  } 

				   
		    // wait until windows closed....   
		    int ec = a.exec();

		     
		    //Espera a que se ejectura a
		    return ec;

} */
