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


/* Includes para ROS */

#define PI 3.14159265

#include <iostream>
#include "math.h"
#include <sstream>
#include <vector>
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_autonomy/navdata_gps.h"
#include "/home/dell-077/fuerte_workspace/sandbox/tum_ardrone/msg_gen/cpp/include/tum_ardrone/filter_state.h"
#include "drone_GPS/GPS_data.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;
namespace enc = sensor_msgs::image_encodings; 

/* Declaracion de variables globales para el sistema */;

/* navdata_altitude*/
float altitude; 
/* navdata_gps */
float GPSlon,GPSlat,elevation;
int satelites;


void Altitude_Upload(const ardrone_autonomy::navdata_altitudeConstPtr altitudePtr){
altitude=(altitudePtr->altitude_vision)/(float)1000;
}

void GPS_Upload(const ardrone_autonomy::navdata_gpsConstPtr gpsPtr){
GPSlat=gpsPtr->lat;
GPSlon=gpsPtr->lon;
elevation=gpsPtr->elevation;
satelites=gpsPtr->num_sattelites;
}

int main( int argc, char** argv ){

/* Se inicia con la funcion init() de ROS y con la afiliacion a record que esta definido en el CMakelists.text */
ros::init(argc, argv,"UTM_converter");

/* Se definen algunas variables de manejo de ROS para su posterior uso en el programa */
ros::NodeHandle Alt;
ros::NodeHandle GPS;
ros::NodeHandle n;

/* Se crea el suscripto al topico /ardrone/image_raw */

/* A subscriber is created at topic /ardrone/navdata_altitude */
ros::Subscriber Nav_alt = Alt.subscribe("/ardrone/navdata_altitude", 20, Altitude_Upload);
/* A subscriber is created at topic /ardrone/navdata_gps */
ros::Subscriber Nav_GPS = GPS.subscribe("/ardrone/navdata_gps", 20, GPS_Upload);

ros::Publisher GPS_X = n.advertise<std_msgs::Float64>("/drone_GPS/X", 100);
ros::Publisher GPS_Y = n.advertise<std_msgs::Float64>("/drone_GPS/Y", 100);
ros::Publisher GPS_data = n.advertise<drone_GPS::GPS_data>("/drone_GPS/data", 100);

/* Se define la frecuencia de ejecucion de los lazos del programa */
ros::Rate loop_rate(15);  //simulate 15 fps 

char zone;
cout <<"Defina en que hemisferio se encuentra: Norte(N)/Sur(S) "<<endl;
cin >> zone; 

	while(ros::ok()){

	/*Semieje mayor (a) - Semieje menor (b) */
	double a=6378388.0;
	double b=6356911.946139;
	
	/* Almaceno la latitud y longitud actuales */
	double lon=GPSlon;
	double lat=GPSlat;
	
	/* Latitud y longitud (En radianes) */
	double latRad=(lat*PI)/180;
	double lonRad=(lon*PI)/180;

	/* Radio de curvatura polar (c) - Aplanamiento (d) */
	double c=pow(a,2)/b;
	double d=(a-b)/a;
	
	/* Excentricidad y Segunda Excentricidad */
	double e=sqrt(pow(a,2)-pow(b,2))/a;
	double se=sqrt(pow(a,2)-pow(b,2))/b;

	/* Huso Horario */
	int huso=int((lon/6)+31);

	/*Distancia angular entre la longitud y el meridiano central del uso meridiano central (mc) */
	float mc=lonRad-(((huso*6-183)*PI)/180);

	/* Calculo Parametros */
	double A=cos(latRad)*sin(mc);
	double ep=((double)1/(double)2)*log((1+A)/(1-A));
	double n=atan(tan(latRad)/cos(mc))-latRad;
	double v=(c/sqrt(1+pow(se,2)*pow(cos(latRad),2)))*(float)0.9996;
	double z=(pow(se,2)/2)*pow(ep,2)*pow(cos(latRad),2);
	double A1=sin(2*latRad);
	double A2=A1*pow(cos(latRad),2);
	double J2=latRad+(A1/2);
	double J4=(3*J2+A2)/4;
	double J6=(5*J4+A2*pow(cos(latRad),2))/(float)3;
	double se2=pow(se,2);
	double alpha=((float)3/(float)4)*se2;
	double betha=((float)5/(float)3)*pow((double)alpha,2);
	double gama=((float)35/(float)27)*pow(alpha,3);

	double B=(float)0.9996*c*(latRad-alpha*J2+betha*J4-gama*J6);

	/* Calculo de X y Y*/
	double X=ep*v*(1+(z/3))+(float)500000;
	double Y=n*v*(1+z)+B;
	
	/* Clausula para calcular Y dependiendo si esta en el hemisferio norte o sur */
	if(zone=='s')
	Y=Y+10000000;

	/* Publicador de X,Y y datos del GPS */
	std_msgs::Float64  msg_X, msg_Y;
	drone_GPS::GPS_data data;

	/* Asignamos las variables a los mensajes definidos en ROS */
  	msg_X.data = X;
	msg_Y.data = Y;

	/* Llena la estructura con los datos */
	data.longitude=lon;
	data.latitude=lat;
	data.elevation=elevation;
	data.X=X;
	data.Y=Y;
	data.time_zone=huso;
	data.altitude=altitude;
	data.num_sattelites=satelites;
	
	/* Publicamos utilizando los publicadores creados */
	GPS_X.publish(msg_X);
	GPS_Y.publish(msg_Y);
	GPS_data.publish(data);

	ros::spinOnce();
	loop_rate.sleep();
 
	} /* Fin while() */
  
return 0;
}
