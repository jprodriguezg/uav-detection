 /**

 *  detection_gui.cpp
 *
 *  Este archivo es parte de drone_detection
 *  Creado por: Juan Pablo Rodríguez y Carolina Castiblanco
 *  Informacion: j_rodriguezg@javeriana.edu.co   jenny.castiblanco@javeriana.edu.co
 *  
 *  En este archivo fue generado para la activacion de la gui (tiene incluido a main.cpp), no es utilizado actualmente en el paquete 
 *  porque  el codigo presente en este se encuentra en su totalidad implementado en detection.cpp. Este archivo fue creado como base 
 *  para la creacion de la GUI. 

 */

/* Includes para el funcionamiento de la GUI con Qt4 */
#include "detection_gui.h"
#include "ui_detection_gui.h"

#include <sys/types.h>
#include <vector>
#include <string>
#include<cstring>
#include <iostream>
#include <fstream>
#include "detection_gui.h"
#include <QtGui>
#include <QApplication>
#include <QWidget>
#include <QString>
#include <QToolBar>
#include <qobject.h>
#include <QStatusBar>
#include <QDockWidget>
#include <QIcon>
#include <QPainter>
#include <QGroupBox>
#include <QPixmap>
#include <QtCore/QVariant>

/* Includes para ROS y OpenCV */
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
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <sys/stat.h>


using namespace cv;
using namespace std;

/* Declaracion de variables globales para el sistema */
namespace enc = sensor_msgs::image_encodings; 
static const char WINDOW[] = "PUJ";
cv_bridge::CvImagePtr cv_ptr;
int imageready=0;

/* Variables donde se almacenaran los datos de navegacion obtenidos */
char datoaltd[250];
char datovelx[250];
char datovely[250];
char datovelz[250];
char datoPitch[250];
char datoYaw[250];
char datoRoll[250];
char datoBP[250];



/* Llamado a la clase detection_gui */
detection_gui::detection_gui(QWidget *parent) :
     QWidget(parent){
    /* Inicializacion de la variable ui, puntero de detection_gui.h */
    ui.setupUi(this);

}
/* Constructor y destructor */
detection_gui::~detection_gui()
{
    //delete ui;
}

/*Funcion savenavdata  
 
 * Esta funcion se encarga de conectarce a los datos de vuelo provenientes de /ardrone/navdata y guardarlos en un archivo con la fecha y
 * hora del comienzo de la mision
 * Los datos se obtienen a partir del subscriptor Nav declarado en el main del programa

*/
void savenavdata(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{

	/* A partir de este punto se declaran en las varibles globales dato_ los datos provenientes de los diferentes sensores del drone 
           con snprintf() se almacena en la variable, el nombre de esta y el valor extraido del topico /ardrone/navdata. La /asignacion
   	   (int)navdataPtr->altd guarda en la variable navdataPtr el valor proveniente de altd */ 

		snprintf(datoaltd,250,"Estimated Altitude: %d ",(int)navdataPtr->altd);
		strcat(datoaltd," mm ");
		
		snprintf(datovelx,250,"Linear Velocity x: %f ",(float)navdataPtr->vx);
		strcat(datovelx," mm/s ");

		
		snprintf(datovely,250,"Linear Velocity y: %f ",(float)navdataPtr->vy);
		strcat(datovely," mm/s ");

		
		snprintf(datovelz,250,"Linear Velocity y: %f ",(float)navdataPtr->vz);
		strcat(datovelz," mm/s ");

		
		snprintf(datoPitch,250,"Pitch Angle: %f ",(float)navdataPtr->rotY);
		strcat(datoPitch," º ");
		
		snprintf(datoYaw,250,"Yaw Angle: %f ",(float)navdataPtr->rotX);
		strcat(datoYaw," º ");
		
		snprintf(datoRoll,250,"Roll Angle: %f ",(float)navdataPtr->rotZ);
		strcat(datoRoll," º ");
		
		snprintf(datoBP,250,"Battery Percent: %f ",(float)navdataPtr->batteryPercent);
		strcat(datoBP," % ");

 return;
  
}  

/*Funcion imageCb  
 
 * Esta funcion se encarga de obtener la imagen proveniente del topico /ardrone/image_raw la cual sera utilizada para el programa para 
 * efectuar la deteccion de minas 

*/

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
/* Mediante la siguiente instruccione se almacena la imagen obtenida en formato sensor_msgs en una imagen con formato BGR8 */
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
/* Se activa la variable imageready=1 para evitar entrar en el if del main() que publica el mensaje que indica que la imagen no fue cargada */
  imageready=1;

} 



int main(int argc, char *argv[])
{
/* Se inicia con la funcion init() de ROS y con la afiliacion a drone_detection_GUI que esta definido en el CMakelists.text */		
ros::init(argc, argv,"drone_detection_GUI");
		
/* Se definen algunas variables de manejo de ROS para su posterior uso en el programa */
ros::NodeHandle nh_;
ros::NodeHandle camImage;
ros::NodeHandle navdata;
ros::NodeHandle n;
		
/* Se crea el suscripto al topico /ardrone/image_raw */
ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, imageCb);
/* Se crea el suscripto al topico /ardrone/navdata */
ros::Subscriber Nav = camImage.subscribe("/ardrone/navdata", 5, savenavdata);
/* Se define la frecuencia de ejecucion de los lazos del programa */
ros::Rate loop_rate(15);  //simulate 15 fps

/* Se crea el objeto a del tipo QApplication para el uso de la GUI */		
QApplication a(argc,argv);
/* Se crea el objeto w de la clase detection_gui, recuerde que esta clase esta definida en detection_gui.h */
detection_gui w; 
/* Se abre la ventana de la GUI */ 
w.show();

	/* Este while solo es utilizado para publicar un error si la imagen no fue cargada, observe que la condicion biene dada por 
           imageready que cambia de valor en la funcion imageCb */
				while(imageready!=1){
				/* Se declara la variable parada */ 
					char parada=waitKey(50);
					ROS_INFO(" Waitting for image to be ready\n");
					cout << "Waitting for image to be ready" << endl ;
					ros::spinOnce();
					loop_rate.sleep();
					/* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
					if( parada=='r'){
						cout << "Adios!" << endl ;
						break;
							}
				}


		/* Se declara un ciclo infinito para caragar los datos de navegacion y procesar las imagenes de entrada de manarea
                   constante e inifinita o hasta que finalice el programa */	
			  for (;;)
			  {
				/* Se declara la variable parada */ 
				char parada=waitKey(50);
				/* Se guarda la imagen obtenida con el drone en la variable IM */
				Mat IM=cv_ptr->image;
				/* Declaracion de IM2 para almacenar en esta la imagen obtenida pero en el espacio BGR, esto se 
				   hace porque la funcion Qimage cambia los colores de la imagen de RGB a BGR */
				Mat IM2;
				cvtColor(IM, IM2, CV_BGR2RGB);
				/* Se utiliza QImage para publicar en la GUI la imagen obtenida del drone en el formato de Qt*/
				QImage imageout=QImage((uchar*) IM2.data, IM2.cols, IM2.rows, IM2.step, QImage::Format_RGB888);
				/* Se publican en la GUI los datos enviados por el drone en los diferentes labels de la GUI */
				w.ui.labelimage->setPixmap(QPixmap::fromImage(imageout));
				w.ui.labelAltitude->setText(datoaltd);
				w.ui.labelVelocity_x->setText(datovelx);
     				w.ui.labelVelocity_y->setText(datovely);
     				w.ui.labelVelocity_z->setText(datovelz);
     				w.ui.labelPitch->setText(datoPitch);
     				w.ui.labelYaw->setText(datoYaw);
     				w.ui.labelRoll->setText(datoRoll);
     				w.ui.labelBattery->setText(datoBP);
				
				ros::spinOnce();
				loop_rate.sleep();
				/* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
				if(parada=='r'){
					cout << "Adios!" << endl ;
					break;
					}
			  	} 

		    /* Esta instruccion espera hasta que la ventana sea cerrada */  
		    int ec = a.exec(); 		   
		    /* Espera a que se ejectura a */
		    return ec;	    
} 
