 /**
 *  This file is part of drone_GPS.
 *
 *  Copyright 2014 Juan Pablo Rodíguez <j_rodriguezg@javeriana.edu.co> (Pontificia Universidad Javeriana - Bogotá)
 
 *  drone_GPS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  drone_detection is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with drone_GPS.  If not, see <http://www.gnu.org/licenses/>.
 */



/* Includes para ROS y OpenCV */

#include <iostream>
#include <vector>
#include "std_msgs/String.h"
#include <sstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "drone_GPS/GPS_data.h"
#include "/home/dell-077/fuerte_workspace/sandbox/tum_ardrone/msg_gen/cpp/include/tum_ardrone/filter_state.h"

using namespace std;
namespace enc = sensor_msgs::image_encodings; 

/* Declaracion de variables globales para el sistema */
static const char WINDOW[] = "PUJ";
int contador=0;
char Directorio_Navdata [200];
char Directorio_Estimator [200];
char Directorio_GPS [200];

/* Navdata */
float yaw,pitch,roll,velX,velY,velZ,aclX,aclY,aclZ;
/* drone_GPS */
double altitude,lon,lat,elevation,X,Y;
int tzone, Nsatelites;
/* Estate estimator */
float xs,ys,zs,dx,dy,dz,pitch_ES,roll_ES,yaw_ES,dyaw;

/* threads */

void Navdata_Upload(const ardrone_autonomy::NavdataConstPtr navdataPtr){
roll=navdataPtr->rotX;
yaw=navdataPtr->rotY;
pitch=navdataPtr->rotZ;
velX=navdataPtr->vx;
velY=navdataPtr->vy;
velZ=navdataPtr->vz;
aclX=navdataPtr->ax;
aclY=navdataPtr->ay;
aclZ=navdataPtr->az; 
}

void Estate_Stimator_Upload(const tum_ardrone::filter_state Stimator){
xs=Stimator.x;
ys=Stimator.y;
zs=Stimator.z;
dx=Stimator.dx;
dy=Stimator.dy;
dz=Stimator.dz;
roll_ES=Stimator.roll;
pitch_ES=Stimator.pitch;
yaw_ES=Stimator.yaw;
dyaw=Stimator.dyaw;
}

void GPS_Upload(const drone_GPS::GPS_data GPS_data){
lon=GPS_data.longitude;
lat=GPS_data.latitude;
elevation=GPS_data.elevation;
//altitude=GPS_data.altitude;
X=GPS_data.X;
Y=GPS_data.Y;
tzone=GPS_data.time_zone;
Nsatelites=GPS_data.num_sattelites;
}
//Linea agregada
/* Funcion Altitude_Upload */
void Altitude_Upload(const ardrone_autonomy::navdata_altitudeConstPtr altitudePtr){
altitude=(altitudePtr->altitude_vision)/(float)1000;
}

int main( int argc, char** argv ){

/* Se inicia con la funcion init() de ROS y con la afiliacion a record que esta definido en el CMakelists.text */
ros::init(argc, argv,"GPS_test");

/* Se definen algunas variables de manejo de ROS para su posterior uso en el programa */
ros::NodeHandle navdata;
ros::NodeHandle StateEstimator;
ros::NodeHandle GPS;
ros::NodeHandle navdata_altitude;

/* A subscriber is created at topic /ardrone/navdata */
ros::Subscriber Nav = navdata.subscribe("/ardrone/navdata", 5, Navdata_Upload);
/* A subscriber is created at topic /ardrone/navdata */
ros::Subscriber ES = StateEstimator.subscribe("/ardrone/predictedPose", 5, Estate_Stimator_Upload);
/* A subscriber is created at topic /drone_gps/data */
ros::Subscriber Nav_GPS = GPS.subscribe("/drone_GPS/data", 5, GPS_Upload);
/* A subscriber is created at topic /ardrone/navdata_altitutde */
ros::Subscriber Alt = navdata_altitude.subscribe("/ardrone/navdata_altitude", 5, Altitude_Upload);
/* Se define la frecuencia de ejecucion de los lazos del programa */
ros::Rate loop_rate(15);  //simulate 15 fps 
/* Se declara un inicio de tiempo en ROS para poder definir un delay exacto en el programa */
ros::Time begin = ros::Time::now();
/* Se define un delay  de 0.04 segundos para que cada iteracion del progama se practiamente 0.25 s, normalmente el programa sin el delay
   se demora 0.21 segundos por ciclo */
ros::Duration delay = ros::Duration(0.5, 0);


/*  Se crean variables de tiempo para saber la fecha y hora exactas */
time_t rawtime;
struct tm * timeinfo;
/* En Fecha se almacena la ubicacion donde se almacenaran los videos */
char Directorio [150];
int location;
/* Se obtiene la Fecha actual  */
time ( &rawtime );
timeinfo = localtime (&rawtime);
/* Se guarda en la  variable Fecha solamente año-mes-dia (%F) para  concaternarlo con la direccion donde  se creara la carpeta */
strftime (Directorio_Navdata,200,"/home/dell-077/fuerte_workspace/sandbox/drone_GPS/data/Navdata/Navdata_%F-%R.csv",timeinfo);
strftime (Directorio_Estimator,200,"//home/dell-077/fuerte_workspace/sandbox/drone_GPS/data/Stimator/Estate_Stimator_%F-%R.csv",timeinfo);
strftime (Directorio_GPS,200,"/home/dell-077/fuerte_workspace/sandbox/drone_GPS/data/GPS/GPS_%F-%R.csv",timeinfo);

/* Crear el directorio en la direccion escrita en "Directorio" */
location = mkdir(Directorio, S_IRWXU |S_IRWXG | S_IROTH | S_IXOTH);	

	while(1){
	
		/* Se activa el delay de ROS */
		delay.sleep();
	    	
		contador++;
		
		FILE *datanav;
		FILE *dataEst;
		FILE *dataGPS;
		char data_navigation[512];
		char data_Estimator[512];
		char data_GPS[512];

		if(contador==1){

		if ((datanav=fopen(Directorio_Navdata,"a+"))!=NULL)
		sprintf(data_navigation,"ID;roll;yaw;pitch;altura;vx;vy;vz;ax;ay;az");

		fprintf(datanav,"%s \n", data_navigation);
		/* Cierra datanav con los datos de navegacion guardados */
		/* Closed datanav with the navigation data saved */
		fclose(datanav);

		if ((dataEst=fopen(Directorio_Estimator,"a+"))!=NULL)
		sprintf(data_Estimator,"x;y;z;dx;dy;dz;roll;pithc;yaw;dyaw");
		fprintf(dataEst,"%s \n", data_Estimator);
		/* Cierra datanav con los datos de navegacion guardados */
		/* Closed datanav with the navigation data saved */
		fclose(dataEst);
		
		
		if ((dataGPS=fopen(Directorio_GPS,"a+"))!=NULL)
		sprintf(data_GPS,"ID;Longitude;Latitude;Altitude;Elevation;X;Y;Zone;No Satellites");
		fprintf(dataGPS,"%s \n", data_GPS);
		fclose(dataGPS); 	
			
		} /*Cierro if contador */

		if ((datanav=fopen(Directorio_Navdata,"a+"))!=NULL)
		/* Con sprintf() se guarda en datalog los datos de vuelo */
		/* function sprintf() saved the flight data in datalog */
                sprintf(data_navigation,"%d;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f",contador,roll,yaw,pitch,altitude,velX,velY,velZ,aclX,aclY,aclZ);

		fprintf(datanav,"%s \n", data_navigation);
		/* Cierra datanav con los datos de navegacion guardados */
		/* Closed datanav with the navigation data saved */
		fclose(datanav);
		
		if ((dataEst=fopen(Directorio_Estimator,"a+"))!=NULL)
		sprintf(data_Estimator,"%f;%f;%f;%f;%f;%f;%f;%f;%f;%f",xs,ys,zs,dx,dy,dz,roll_ES,pitch_ES,yaw_ES,dyaw);
		fprintf(dataEst,"%s \n", data_Estimator);
		/* Cierra datanav con los datos de navegacion guardados */
		/* Closed datanav with the navigation data saved */
		fclose(dataEst);
		
		if ((dataGPS=fopen(Directorio_GPS,"a+"))!=NULL)
		sprintf(data_GPS,"%d;%f;%f;%f;%f;%f;%f;%d;%d",contador,lon,lat,altitude,elevation,X,Y,tzone,Nsatelites);
		fprintf(dataGPS,"%s \n", data_GPS);
		fclose(dataGPS);
		
		ros::spinOnce();
		loop_rate.sleep();

	} 
  
return 0;
}
