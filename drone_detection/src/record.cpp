 /**

 *  record.cpp
 *
 *  Este archivo es parte de drone_detection
 *  Creado por: Juan Pablo Rodríguez y Carolina Castiblanco
 *  Informacion: j_rodriguezg@javeriana.edu.co   jenny.castiblanco@javeriana.edu.co
 *  
 *  En se encuentra el programa que se encarga de convetir las imagenes provenientes de /ardrone/image_raw en un archivo de 
 *  video .AVI que se almacena en la carpeta Videos de drone_detection

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

/* Declaracion de variables globales para el sistema */
static const char WINDOW[] = "PUJ";
cv_bridge::CvImagePtr cv_ptr;
int imageready=0;
 
/*Funcion imageCb  
 
 * Esta funcion se encarga de obtener la imagen proveniente del topico /ardrone/image_raw la cual sera utilizada para el programa para 
 * efectuar la deteccion de minas 

*/
 
void imageCb(const sensor_msgs::ImageConstPtr& msg){
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



int main( int argc, char** argv ){

/* Se inicia con la funcion init() de ROS y con la afiliacion a record que esta definido en el CMakelists.text */
ros::init(argc, argv,"record");

/* Se definen algunas variables de manejo de ROS para su posterior uso en el programa */
ros::NodeHandle nh_;
ros::NodeHandle camImage;

/* Se crea el suscripto al topico /ardrone/image_raw */
ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, imageCb);
/* Se define la frecuencia de ejecucion de los lazos del programa */
ros::Rate loop_rate(15);  //simulate 15 fps 

/*  Se ceran variables de tiempo para saber la fecha y hora exactas */
time_t rawtime;
struct tm * timeinfo;
/* En Fecha se almacena la ubicacion donde se almacenaran los videos */
char Fecha [80];
/* Se obtiene la Fecha actual  */
time ( &rawtime );
timeinfo = localtime (&rawtime);
/* Se guarda en la  variable Fecha solamente año-mes-dia (%F) para  concaternarlo con la direccion donde  se creara la carpeta */
strftime (Fecha,80,"%F-%R",timeinfo);


/* Este while solo es utilizado para publicar un error si la imagen no fue cargada, observe que la condicion biene dada por 
   imageready que cambia de valor en la funcion imageCb */	
while(imageready!=1)
	{
		/* Se declara la variable parada */ 	 
	        char parada=waitKey(50);
		ROS_INFO(" Waitting for image to be ready\n");
		ros::spinOnce();
		loop_rate.sleep();

		/* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
		if( parada=='r'){
			cout << "Adios!" << endl ;
			break;
				}
	}
	/* Se definen algunas variables que se utilizaran para la grabacion del video */
	int isColor = 1;
	int fps     = 15; 
	int frameW  = 640; 
	int frameH  = 360;
	CvSize size;
	size.width = frameW;
	size.height = frameH;
	
	/* Las variables os,str y nombre son definidas es para ponerle al video como nombre la fecha y la hora actual  */	
	ostringstream os;
	os <<"/home/caro/fuerte_workspace/sandbox/drone_detection/video/"<<Fecha<<".avi";
        std::string str = os.str();
	const char * nombre=str.c_str();
	
	/* Con la direccion definida en nombre se utiliza en CvVideoWriter para indicar la ubicacion y formato del video */	
	CvVideoWriter *writer = cvCreateVideoWriter(nombre,CV_FOURCC('M','J','P','G'),fps,size,isColor);
	
  /* Se declara un ciclo infinito para guardar las imagenes obtenidas por el UAV en un archivo de video de manarea
     inifinita o hasta que finalice el programa */	
	while(1){
	    /* Se declara la variable parada */
	    char parada=waitKey(50);
	    /* Se guarda la imagen obtenida con el drone en la variable IM */
	    Mat IM=cv_ptr->image;
	    /* Se pasa la imagen a formato img */
	    IplImage img = IM;
	    /* Se agrega a la variable writer la imagen obtenida */
	    cvWriteFrame(writer,&img);      // add the frame to the file
	    /* Se muestra la imagen obtenida en una ventana llama Video Window */
	    imshow("Video Window", IM);
            /* El waitKey() se utiliza para observar la imagen indefinidamente */
	    waitKey(1);
	    
	    /* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
	    if( parada=='r'){
		cvReleaseVideoWriter(&writer);
		break;
		}
	    /* Si no se cumple la condicion se continua con el ciclo */
	    else {
		ros::spinOnce();
	        loop_rate.sleep();
		}
	    }
cout << "Fin del video!!!" << endl;    
return 0;
}
