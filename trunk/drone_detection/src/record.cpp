 /**

 *  record.cpp
 *  This file is part of drone_detection
 *  Este archivo es parte de drone_detection
 *  Created by: Juan Pablo Rodríguez y Carolina Castiblanco
 *  Information: j_rodriguezg@javeriana.edu.co   jenny.castiblanco@javeriana.edu.co
 *  
 *  En se encuentra el programa que se encarga de convetir las imagenes provenientes de /ardrone/image_raw en un archivo de 
 *  video .AVI que se almacena en la carpeta Videos de drone_detection
 *  In this file is the program in charge of create a video file with the images from the topic /ardrone/image_raw. The created file has  
 *  .AVI format and is saved in the folder video of drone_detection package.


 */

/* Includes para ROS y OpenCV */
/* Includes to ROS y OpenCV */
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

/* Declaracion de variables globales para el sistema */
/* Global variables of the system */
static const char WINDOW[] = "PUJ";
cv_bridge::CvImagePtr cv_ptr;
int imageready=0;
 
/* Funcion Image_Upload  
 * Image_Upload  function 

 * Esta funcion se encarga de obtener la imagen proveniente del topico /ardrone/image_raw la cual sera utilizada para el programa para 
 * efectuar la deteccion de minas  
 * This function is in charge to obtain the image frome the topic /ardrone/image, it will be use for the algorithm to perform the video record

*/
 
void Image_Upload(const sensor_msgs::ImageConstPtr& msg){

/* Mediante la siguiente instruccione se almacena la imagen obtenida en formato sensor_msgs en una imagen con formato BGR8 */
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

/* Se activa la variable imageready=1 para evitar entrar en el if del main() que publica el mensaje que indica que la imagen no fue cargada */
/* The variable imageready is activated (1) to avoid go inside of the first while() of the algorithm. The while() function is notify the user 
   that the image has not been loaded */

  imageready=1;  
}



int main( int argc, char** argv ){

/* Se inicia con la funcion init() de ROS y con la afiliacion a record que esta definido en el CMakelists.text */
/* The program begins with the function init() by ROS and the inscription of record in the CMakelists.text file*/
ros::init(argc, argv,"record");

/* Se definen algunas variables de manejo de ROS para su posterior uso en el programa */
/* Some variables of the ROS enviroment are created for  future use */
ros::NodeHandle nh_;
ros::NodeHandle camImage;

/* Se crea el suscripto al topico /ardrone/image_raw */
/* The suscription to the ros topic /ardrone/image_raw is created */
ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, Image_Upload);
/* Se define la frecuencia de ejecucion de los lazos del programa */
/* The operation frequency of the program is defined */
ros::Rate loop_rate(15);  //simulate 15 fps 

/* Se crean variables de tiempo para saber la fecha y hora exactas */
/* The next variables are created to obtain the current date and hour */
time_t rawtime;
struct tm * timeinfo;
/* En Fecha se almacena la ubicacion donde se almacenaran los video */
/* In Fecha are saved the location where al the video will be saved */
char Fecha [80];
/* Se obtiene la Fecha actual  *
/* The current date is obtained  */
time ( &rawtime );
timeinfo = localtime (&rawtime);
/* Se guarda en la  variable Fecha solamente año-mes-dia (%F) para  concaternarlo con la direccion donde  se creara la carpeta */
/* In Fecha is saved only year-month-day (%F) to be concatenaded with the address where the folder with the video will be created */
strftime (Fecha,80,"%F-%R",timeinfo);


/* Este while solo es utilizado para publicar un error si la imagen no fue cargada, observe que la condicion biene dada por 
   imageready que cambia de valor en la funcion imageCb */
/* This while is only used to publish a errror message if the image don't be upload */	
while(imageready!=1)
	{
		/* Se declara la variable parada */ 
		/* The parada variable is created */ 	 
	        char parada=waitKey(50);
		ROS_INFO(" Waitting for image to be ready\n");
		ros::spinOnce();
		loop_rate.sleep();

		/* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
		/* This condition is used to stop the program if the r key is pressed*/
		if( parada=='r'){
			cout << "Adios!" << endl ;
			break;
				}
	}
	/* Se definen algunas variables que se utilizaran para la grabacion del video */
	/* Some variables are created to future use in the program */
	int isColor = 1;
	int fps     = 15; 
	int frameW  = 640; 
	int frameH  = 360;
	CvSize size;
	size.width = frameW;
	size.height = frameH;
	
	/* Las variables os,str y nombre son definidas es para ponerle al video como nombre la fecha y la hora actual  */	
	/* The os, str and nombre are defined to assign to the video a name  with the current date and hour   */	
	ostringstream os;
	os <<"/home/dell-077/fuerte_workspace/sandbox/drone_detection/video/"<<Fecha<<".avi";
        std::string str = os.str();
	const char * nombre=str.c_str();
	
	/* Con la direccion definida en nombre se utiliza en CvVideoWriter para indicar la ubicacion y formato del video */
	/* CvVideoWriter is used to indicate the location and format of the video using "nombre" */
	CvVideoWriter *writer = cvCreateVideoWriter(nombre,CV_FOURCC('M','J','P','G'),fps,size,isColor);

	/* Se declara un ciclo infinito para guardar las imagenes obtenidas por el UAV en un archivo de video de manarea
        inifinita o hasta que finalice el programa */
  	/* The obtained images are saved using a infinte loop. The loop is over when the user press the r key over the video window */	
	while(1){
	    /* Se declara la variable parada */
	    /* The parada variable is created */
	    char parada=waitKey(50);
	    /* Se guarda la imagen obtenida con el drone en la variable IM */
	    /* The obtained image is saved in the IM variable*/
	    Mat IM=cv_ptr->image;
	    /* Se pasa la imagen a formato img */
	    /* The image is converted to img format */
	    IplImage img = IM;
	    /* Se agrega a la variable writer la imagen obtenida *
	    /* The image obtained is added to the writer variable */
	    cvWriteFrame(writer,&img);      // add the frame to the file
	    /* Se muestra la imagen obtenida en una ventana llama Video Window */
	    /* The obtained image is shown in special window named Video Window */
	    imshow("Video Window", IM);
	    /* El waitKey() se utiliza para observar la imagen indefinidamente */
	    /* The waitKey() is used to publish the image indefinitely */
	    waitKey(1);

	    /* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
	    /* This condition is use to stop the program when the key r is pressed */
	    if( parada=='r'){
		cvReleaseVideoWriter(&writer);
		break;
		}
	    /* Si no se cumple la condicion se continua con el ciclo */
	    /* If the condition is not satisfy the loop continues */
	    else {
		ros::spinOnce();
	        loop_rate.sleep();
		}
	    }
cout << "Fin del video!!!" << endl;    
return 0;
}
