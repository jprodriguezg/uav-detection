
 /**
 *  This file is part of drone_detection.
 *
 *  Copyright 2013 Juan Pablo Rodíguez <j_rodriguezg@javeriana.edu.co> (Pontificia Universidad Javeriana - Bogotá)
 *  Jenny Carolina Castiblanco <jenny.castiblanco@javeriana.edu.co> (Pontificia Universidad Javeriana- Bogotá)

 *  drone_detection is free software: you can redistribute it and/or modify
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
 *  along with drone_detection.  If not, see <http://www.gnu.org/licenses/>.
 */


/*
 *  
 *  En este archivo se encuentra la funcion detection() que se encarga del proceso de deteccion de minas a partir de 
 *  las imagenes obtenidas del topico /ardrone/image_raw. Ademas publica en la GUI del sistema los datos de vuelo 
 *  provenientes de /ardrone/navdata

 *  
 *  In this file, there is the detection() function which performs the landmines detection process from images obteined by the 
 *  /ardrone/image_raw topic.
 *  Adittionally, the flight data from /ardrone/navdata are published in the system GUI.  

 */


/* Includes para ROS y OpenCV */
/* Includes for ROS and OpenCV */
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

/* Includes para el funcionamiento de la GUI con Qt4 */
/* Includes for the GUI operation with Qt4*/
#include "GUI/detection_gui.h"
#include "ui_detection_gui.h"
#include <sys/types.h>
#include <vector>
#include <string>
#include<cstring>
#include <iostream>
#include <fstream>
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


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings; 

/* Declaracion de variables globales para el sistema */
/* Statement of global variables for the system */
static const char WINDOW[] = "PUJ";
cv_bridge::CvImagePtr cv_ptr;
int imageready=0;
int contador=0;
char filename;
char Directorio_Log [100];
int mina = 0;
int contador2=0;
string Nombreimage2;

/* Variables donde se almacenaran los datos de navegacion obtenidos */
/* Variables where the navigation data obtained will be stored */
char datoaltd[250];
char datovelx[250];
char datovely[250];
char datovelz[250];
char datoPitch[250];
char datoYaw[250];
char datoRoll[250];
char datoBP[250];
char datomina[250]="Landmines detected: 0";

/* Llamado a la clase detection_gui */
/* Call to detection_gui class*/
detection_gui::detection_gui(QWidget *parent) :
     QWidget(parent){
    /* Inicializacion de la variable ui, puntero de detection_gui.h */
    /* Initialization ui variable, pointer of detection_gui.h*/
    ui.setupUi(this);

}
/* Constructor y destructor */
/* Builder and destroyer */
detection_gui::~detection_gui()
{
    //delete ui;
}

/*Funcion Navdata_Upload
/*Navdata_Upload Function 
 
 * Esta funcion se encarga de conectarce a los datos de vuelo provenientes de /ardrone/navdata y guardarlos en un archivo con la fecha y
   hora del comienzo de la mision. 
   Los datos se obtienen a partir del subscriptor Nav declarado en el main del programa.
 * This function gets the flight data from /ardrone/navdata and saves them in a file which name is formed with the date and starting hour of
   the mission the data are obteined from the Nav subscriptor which is declared in the main() function of the program.  

*/

void Navdata_Upload(const ardrone_autonomy::NavdataConstPtr navdataPtr){

/* Declaracion del archivo datanav y el caracter datalog en donde se almacenaran los datos de vuelo para guardarlos en datanv */
/* Statement file datanav and character datalog, where the flight data are stored in order to save them in file datanav */
FILE *datanav;
char datalog[512];

if ((datanav=fopen(Directorio_Log,"a+"))!=NULL)
/* Con sprintf() se guarda en datalog los datos de vuelo */
/* function sprintf() saved the flight data in datalog */
sprintf(datalog,"%d %f %f %f %d %f %f %f %f %f %f\n",contador2, navdataPtr->rotX,navdataPtr->rotY,navdataPtr->rotZ,navdataPtr->altd, navdataPtr->vx,navdataPtr->vy,navdataPtr->vz,navdataPtr->ax,navdataPtr->ay,navdataPtr->az);
fprintf(datanav,"%s \n", datalog);
/* Cierra datanav con los datos de navegacion guardados */
/* Closed datanav with the navigation data saved */
fclose(datanav);

	/* A partir de este punto se declaran en las varibles globales dato_ los datos provenientes de los diferentes sensores del drone 
           con snprintf() se almacena en la variable, el nombre de esta y el valor extraido del topico /ardrone/navdata. La /asignacion
   	   (int)navdataPtr->altd guarda en la variable navdataPtr el valor proveniente de altd */ 
	/* The data from of different drone sensors are saved in the global variables. Using the function snprintf(), data are saved as the
	   measure obteined of /ardrone/navdata topic and the measure name. 
	   the assignment "(int)navdataPtr->altd" stores the measure from altd in the navdataPtr variable*/ 
		snprintf(datoaltd,250,"Estimated Altitude: %d ",(int)navdataPtr->altd);
		strcat(datoaltd," mm ");
		
		snprintf(datovelx,250,"Linear Velocity x: %f ",(float)navdataPtr->vx);
		strcat(datovelx," mm/s ");

		
		snprintf(datovely,250,"Linear Velocity y: %f ",(float)navdataPtr->vy);
		strcat(datovely," mm/s ");

		
		snprintf(datovelz,250,"Linear Velocity z: %f ",(float)navdataPtr->vz);
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

/* Function Image_Upload   

 * Esta funcion se encarga de obtener la imagen proveniente del topico /ardrone/image_raw la cual sera utilizada por el programa para 
   efectuar la deteccion de minas. 
 * This function gets the images from /ardrone/image_raw topic which will be used by the program to do the landmines detection task.  

*/
 
void Image_Upload(const sensor_msgs::ImageConstPtr& msg)
{

  /* The next instruction changes the image format and then save the new image. The original image has a sensor_msgs format which is changed
     to BGR8 format */
  /* Mediante la siguiente instruccion se almacena la imagen obtenida en formato sensor_msgs en una imagen con formato BGR8 */
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  /* Se activa la variable imageready=1 para evitar entrar en el while del main() que publica el mensaje que indica que la imagen no fue
     cargada */
  /* Initialization of the variable imageready=1 in order to avoid go into the while of main() that publics the message "Waitting for image
     to   be ready" */
  imageready=1;
}

/*Funcion Remove_Medium_Objects  antes llamada -> removeSmallBlobs
 
 * Esta funcion se encarga de eliminar de la imagen de entrada im, los objetos con un tamaño menor que "sizec" con el objetivo de eliminar 
   ruido de la imagen
 * The main goal of the function is removed some image noise. The objects with a size lesser than a parameter that is given by the user
   (sizec) are eliminated of the image.

*/
//**********************************************
void Remove_Medium_Objects(cv::Mat& im, double sizec){
	/* Declaracion de los vectores contours y hierarchy */
	/* Statments contours and hierarchy vectors*/
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/* Se clona la imagen adquirida (im) para que no sea modificada al ser utilizada con función findContours(), esta funcion es
           utilizada par encontrar todos los contornos de los objetos de la imagen */
	/* The input image is cloned in order to not modify its features when the function findContours() is used. This function finds  		   all contours in the binary image */
	Mat IMRSB = im.clone();	
	findContours( IMRSB, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	/* El siguiente ciclo recorre los diferentes contornos para analizar si el área esta por debajo o no del rango definido 
           para determinar si deben ser eliminados o no */
	/* The next cycle defines if the object analyzed must be deleted or not according to gives range */
	for (int i = 0; i < contours.size(); i++)
    	{
		/* Calcula el area Correspondiente a cada Contorno encontrado con la funcion findContours() */
		/* Compute the area corrensponding to each contour that was found with the function findContours() */        	
        	double area = contourArea(contours[i]);
        	/* Condicion (Rango) para determinar si la imagen debe o no ser eliminada de la imagen*/
		/* Condition that decide if the object must be deleted or not */
        	if (area > 0 && area <= sizec)
		/* Remueve los objetos con areas  que se Encuentren en el Rango, para eliminar el contorno, se dibuja encima de este 
		   pixeles de color negro, note que el dibujo se hace directamente sobre la entrada lo que la afecta directamente*/
		/* Remove the objects that satisfies the given condition. The contour is eliminated when over the input image (im) is painted 			   of black the object. */
            	drawContours(im, contours, i, CV_RGB(0,0,0), -1);
    	}
}

/*Funcion Pixels_Sum antes llamada ->Suma_Pixeles

 * Esta funcion suma el numero de pixeles blancos que se encuentran en la imagen de entrada im, para realizar esta accion se recorre la 
   imagen mediente 2 ciclos que van hasta el largo y ancho de la imagen (h y w)  que son entradas de la funcion
 * The main goal of the function is to find the total white pixels of the input image (im). The function scrolls all the image in two cycles. 
   The function input is the image to analyze, its height (h) and its weight (w). 

*/

int Pixels_Sum(cv::Mat im, int h, int w) {

	/* Se declara la funcion "auxiliarim" que es del tipo IplImage para utilizar la funcion cVGet2D() y la variable s del tipo CVScalar */
	/* The input image is assigned to an auxiliarim variable (auxiliarim) which has a different format to use the function cVGet2D() and
	   to manipulate variables of type CVScalar. */
	IplImage auxiliarim = im;
	CvScalar s;
	/* Se declara la variable "pixelesblancos" que es donde se almacenaran el numero de pixeles blancos encontrados */
	/* Statement pixelesblancos variable where the sum of the white pixels are saved */
	int pixelesblancos = 0;
	for (int i = 0; i < h; i++){
		for (int k = 0; k < w; k++){
		/* Mediante cvGet2D se almacena en "S" el valor numerico del pixel analisado en la imagen auxiliarim en la posiion (i,k) */
		/* cvGet2D gets the measure of the pixel that pertein to the position (i,k) of "auxiliarim" image. */
		s=cvGet2D(&auxiliarim,i,k);
		/* Si el pixel leido tiene un valor diferente de 0, se incrementa la variable pixelesblancos que representa el numero de
		   pixeles blancos en la imagen */
		/* The variable "pixelesblancos" is increased when the pixel analyzed have a one value */
		if (s.val[0] != 0) 
		pixelesblancos ++;
		}
	    }
	return pixelesblancos;
}

/* Funcion Biggest_Area antes llamada ->Encontrar_Contorno

 * Esta función analiza todas las áreas de los contornos de la imagen adquirida (im) para entregar en la variable AuxiliarArea el area mas    
   grande encontrada.  
 * This function alalisies all the contorn areas of the input image (im) in order to return the biggest area value that is saved int the
   variable "AuxiliarArea"  

*/

double Biggest_Area(cv::Mat im){
	/* Se crean e inicializan las variables AuxiliarArea, contours y hierarchy */
	/* Initial AuxiliarArea, contours y hierarchy variables*/
	double AuxiliarArea = 0;
 	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/* Se clona la imagen adquirida (im) para que no sea modificada por la función findContours() */
	/* The input image is cloned in order to not modify its features when the function findContours() is used */
	Mat IMEC = im.clone();
	findContours( IMEC, contours, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	/* Mediate el siguiente ciclo se analizan todos los contornos para determinar cual tiene el area mas grande*/
	/* Get the area of the contour number i and it is saved in variable area */
	for (int i = 0; i < contours.size(); i++)
	{
		/* En la variable area se almacena el area del contorno analizado, si el area encontrada es mayor que la anterior 
		   se almacena en la variable AuxiliarArea el valor de la nueva area encontrada */
		/* If the current area is bigger than the last area saved in AuxiliarArea,
		   the AuxiliarArea is update with the value of the current area. */
		double area = contourArea(contours[i]);
		if (AuxiliarArea <= area)
		    AuxiliarArea=area;	
		    
	}
	/* Al finalizar el ciclo en la variable "Auxiliararea" se encuantra el area mas grande encontrada */
	/* This cycle return the biggest area in the auxiliar variable "AuxiliarArea" */
	return AuxiliarArea;
}

/* Funcion Intensity_Classification

 * El objetivo pirncipal es identificar si la imagen de entrada cumple o no con la condicion de intensidad.
 * The main goal is indicate if the input image accomplishs or not with the condition of intensity. 

*/

int Intensity_Classification(int h, int w, Mat IN){
	
	/* Inicializacion de variables. Se realiza un barrido de la imagen para adquirir los valores de intensidad de los poxeles de las tres
	   capas (RGB). */
	/* Initial variables. The image "IN" is analyzed pixel by pixel to adquire the intensity value of the three layers (RGB)  */
	int balde_green = 0, balde_blue = 0, balde_red =0, Resultado =0; 
	for( int i = 0; i < h; i++ )
	{ 
	  for( int k = 0; k < w; k++ )
       	  { 
		/* Los valores de intensidad obtenidos se guardan en tres diferentes variables.
		   Luego los contadores balde_green, balde_blue and balde_red incrementan su valor si la condicion respectiva a cada contador
		   se cumple. */
		/* The three values of intensity are saved in three different variables.
		   Then the balde_green, balde_blue and balde_red variables are incremented when the condition of each layers is
		   accomplished */
		Vec3b intensity = IN.at<Vec3b>(i, k);
		uchar blue = intensity.val[0];
		uchar green = intensity.val[1];
		uchar red = intensity.val[2];
		if(green >=177) 
		{
			balde_green++;
		}
		if(blue >=199) 
		{
			balde_blue++;
		}
		if(red >=200) 
		{
			balde_red++;
		}
	  }
	}
	/* La funcion retorna un valor de zero la suma de los contadores ("balde_") no pertenece al rango que especifica la condicion. */
	/* The function return a zero value if the sum of the "balde_" is not in the condition range */
	if ((balde_green+balde_blue+balde_red)>= 6500 && (balde_green+balde_blue+balde_red)<= 28000)
	{
		return Resultado = 1;
	}
	else
		return Resultado = 0; 
}

/* Funcion Image_Filtering

 * El objetivo pricipal de la funcion es remover todos los objetos de l imagen que no pertenescan a una mina. Realiza el proceso de filtrado
   de la imagen de entrada "IM", asignando el resultado de este proceso en la imagen "IMfinal". La imagen "IMCOriginal" tiene el resultado
   del proceso de filtrado, con la diferencia que esta en escala RGB.
 * The main goal of the image noise filtering is orientated to remove all the objects in the image which do not belong to landmine. This
   function makes the filtering process to the image "IM" and returns the final result in the image "IMfinal". The image IMCOriginal is the
   filtered image in the RGB space.

*/

void Image_Filtering(cv::Mat& IM, cv::Mat& IMCOriginal, cv::Mat& IMfinal,cv::Mat& IMfiltrada, double& AuxiliarArea, int& height , 
int& weight, int& l){


	Mat IM_GRAY, IM_ERODE;
	/* Transformation of the original image to grayscale space and their subsequent binarization. */
	/* Cambiar Imagen Original IM a escala de Grises y luego se binariza*/
	cvtColor(IM, IM_GRAY, CV_RGB2GRAY);
	//Cuando cvtColor no quiera funcionar, usar: Mat IM_GRAY = imread(image,IMREAD_GRAYSCALE);
	/* Binarizar la imagen IM_GRAY */
	Mat IMbin = IM_GRAY > 128;

	/* Se crea una estrucutra morfologica (elipse) para utilizar la funcion erode */	
	/* The morphologics structe is created (ellipse) in oder to execute the morphologics function Erode. */
	Mat disco = getStructuringElement(MORPH_ELLIPSE, Size(7,7));
	/* Aplicar la funcion Erode (erosíon) a la Imagen IMbin */
	/* Apply function Erode to Image IMbin  */
	erode(IMbin, IM_ERODE, disco);
	Mat IMpos = IM_ERODE.clone();
	/* Se remueven los objetos medianos con Remove_Medium_Objects() */
	/* the medium objects are removed with the function Remove_Medium_Objects() */
	Remove_Medium_Objects(IMpos,5000);
	/* Se crea una estrucutra morfologica (elipse) para utilizar la funcion Dilate */
	/* The morphologics structe is created (ellipse) in oder to execute the morphologics function Dilate. */
	Mat disco1 = getStructuringElement(MORPH_ELLIPSE, Size(4,4));
	/* Se recupueran los trozos de imagen perdidos al utilizar erode en los objetos grandes */
	/* Recover the lost pieces of the image using erode in the large objects */
	dilate(IMpos, IMfiltrada, disco1);


	
	IplImage IMB = IM;
	/* Se encuentra el ancho (w) y alto (h) de la imagen */
	/* The hight and wide parameters are finded */
	height = cvGetDimSize(&IMB, 0);
	weight = cvGetDimSize(&IMB, 1);

	int PixelesBlancos = 0; 

	/* Se encuentra el numero de pixeles blancos en la imagen utilizando la funcion Pixels_Sum */	
	/* The function Pixels_Sum allows to find the number of white pixel in the image "IMfiltrada" */
	PixelesBlancos=Pixels_Sum(IMfiltrada,height,weight);

	/* Las siguientes instrucciones son utilizadas si el proceso de filtrado anterior borro de la imagen todos los objetos que tienen 
	   un tamaño semejante al de una mina, si se cumple la condicion se vuelve a efectuar el proceso de filtrado pero esta vez utilizando
	   un valor menor en Remove_Medium_Objects() */
	/* The next intructions are used if the image resulting "IMfiltrada" does not have objets with a similar size to a mine. if the
	   condition is true, the filtering process is executed again but with a lower size value in Remove_Medium_Objects() fuction */	

	if (PixelesBlancos>501 & PixelesBlancos<=5000)
	{
	IMpos = IM_ERODE.clone();
	Remove_Medium_Objects(IMpos,2000);
	Mat disco1 = getStructuringElement(MORPH_ELLIPSE, Size(6,6));
	dilate(IMpos, IMfiltrada, disco1);
	}
	else if (PixelesBlancos<500)
	{
	IMpos = IM_ERODE.clone();
	Remove_Medium_Objects(IMpos,500);
	Mat disco1 = getStructuringElement(MORPH_ELLIPSE, Size(6,6));
	dilate(IMpos, IMfiltrada, disco1);
	}
	
	/* Borrar Areas pequeñas, para dejar la mas grande, note que para este proceso no se utiliza encontrar la fucion Biggest_Area porque
	   en este caso se necesita extraer la posicion del contorno y  llenar las variables center y radius, para su futuro uso en el
	   programa */
	/* The next instructions are used for have only the biggest object in the image. It is necesary to extract of contour position and
	   complete the center and radius vectors because they are necessary for program development. */
	
	Mat ContornoImagen = IMfiltrada.clone();
  	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	// Elimine la asignacion de AuxiliarArea y l de este codigo
	//double AuxiliarArea=0;
	//int l=0;

	findContours( ContornoImagen, contours, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	vector<vector<Point> > contours_poly( contours.size() );
	vector<Point2f>center( contours.size() );
	vector<float>radius( contours.size() );

	for (int i = 0; i < contours.size(); i++)
	{ 
		/* Estas dos lineas de codigo guardan en las variables center y radius el centro y radio de la mina para
		   posteriormente graficar un circulo al rededor de la mina detectada */
		/* These two instructions allow to know the center and radius of all contours and these values are saved in the vectors center
		   and radius. The final values of "l" represents the vector position where is the biggest object */			
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		minEnclosingCircle( contours_poly[i], center[i], radius[i]);    
		double area = contourArea(contours[i]);
	        if (AuxiliarArea <= area)
		{
		    AuxiliarArea=area;
		    l=i; 	
		}       
	    }

	
	/* Se dedine la variable imagen IMCvacio que tiene el tamaño de la imagen original pero solo tienes en su interior */
	Mat IMCvacio = Mat::zeros(ContornoImagen.size(), CV_8UC3);
	/* Adicionalmente se clona IMCvacio en IMCRelleno */
	Mat IMCRelleno = IMCvacio.clone();
	/* Se clona la imagen IM en IMfinal */
	IMfinal = IM.clone();
	
	/* Mediante drawContours se grafica en IMCvacio la el contorno mas grande */
	/* The image "IMCvacio" has the biggest contour */
	//drawContours( IMCvacio, contours, l, CV_RGB(255,255,255), 2, 8, hierarchy, 0, Point() );

	/* Se genera en IMCRelleno el contorno con la imagen del mayor tamaño pero adicionalmente se rellena el contorno con pixeles blancos */
	/* In the image "IMCRelleno" is the biggest contour but contour is filled with white pixels */	
	drawContours(IMCRelleno, contours, l, CV_RGB(255,255,255), -1); 

	/* Se pinta el contorno encontrada en la imagen original */
	/* The image "IMfinal" has the biggest contour */
	drawContours( IMfinal, contours, l, CV_RGB(255,255,255), 2, 8, hierarchy, 0, Point() );

	/* En esta parte se clona en IMCOriginal la imagen IMCRelleno para posteriormente asignarcela  IMCOriginalaux del tipo IplImage,
	   adicionalemte en IMaux se clona IM y es enviada a IMaux2. A continuacion se recorre la imagen para asignar en IMCOriginalaux
	   el valor del pixel de la posicion i,k  de IMaux2. La idea de este procedimiento es poner en la imagen llena de pixeles negros, los
	   pixeles originales de la mina para obtener una imagen en donde se observe unicamente la mina o el objeto mas grande de la imagen
	   original con sus respectivos colores */
	/* The next instructions allow to show the filtered image in RGB scale. The images "IMCRelleno" and "IM" are cloned in
	   "IMCOriginalaux" and "IMaux2" which has a different format to use the function cVGet2D() and to manipulate variables of type
	   CVScalar. The images ("IMCOriginalaux" and "IMaux2") are traveled pixel by pixle so as to assign in IMCOriginalaux the pixel value
	   in the position i,k of IMaux2, if in the position i,k of IMaux2, there is a white pixel. */

	IMCOriginal = IMCRelleno.clone();
	
	IplImage IMCOriginalaux= IMCOriginal;
	Mat IMaux = IM.clone();
	IplImage IMaux2= IMaux;
	CvScalar s2, s3;
	for( int i = 0; i < height; i++ )
	{ 
	  for( int k = 0; k < weight; k++ )
       	  { 
		s2=cvGet2D(&IMCOriginalaux,i,k);
		s3=cvGet2D(&IMaux2,i,k);
		if (s2.val[0] != 0)
		cvSet2D(&IMCOriginalaux,i,k,s3);
       	  }
     	}

}

/*Funcion Image_Features 
 * Image_Features function
 
 * Esta función  determinar las caracteristicas de la imagen que seran determinadas si el objeto encontrado es una mina
 * This function is used to determine the features of the image which will be used to determine if the found object is a landmine

*/

void Image_Features(cv::Mat IMCOriginal,cv::Mat& Mascara,double& CAMascara, double& CAMascara2,double& CARMascara, double& CARMascara2, Point& maxLoc){

/* En esta parte del codigo se realiza el tratamiento de cada una de las mascaras o template del sistema */
/* In this part of code is perform the image processing of templates */
	
	/* Se generan las variables donde se almacenaran las mascaras */
	/* The variables for templates are created */ 
	Mat binMascara, binMascara2; 
	/* Se cargan las mascaras a partir de imread() */
	/* imread() is used to load the images */
	/* Mascara gris */
	/* Grey template */
	Mascara= imread("/home/dell-077/fuerte_workspace/sandbox/drone_detection/src/Mascaras/1m/Mascara1m1.jpg", CV_LOAD_IMAGE_UNCHANGED);
	/* Mascara Azul */
	/* Blue template */
	Mat Mascara2 = imread("/home/dell-077/fuerte_workspace/sandbox/drone_detection/src/Mascaras/1m/Mascara1m3.jpg", CV_LOAD_IMAGE_UNCHANGED);
	/* Se utiliza un condicional para enviar un error si no se cargaron las mascaras */
	/* This condition is used to sent a error message if the template don't be load */
	if (Mascara.empty() || Mascara2.empty()){
		cout << "Error : Imagen mascara no se pudo cargar..!!" << endl;
        	//system("pause"); //wait for a key press
		//return -1;
	}
	/* Se pasan las mascaras a escala de grises */
	/* The templated are converted to grayscale */
	cvtColor(Mascara, binMascara, CV_RGB2GRAY);
	cvtColor(Mascara2, binMascara2, CV_RGB2GRAY);
	/* Binarizacion de las mascaras */
	/* Template binarization */
	binMascara = binMascara > 128;
	binMascara2 = binMascara2 > 128;

	/* Se crean varaibles para almacenar las valores de los contornos de las mascaras */
	/* Some variables are creted to save the values of contours of the templates */

	Mat result, result2;
	double minVal; double maxVal;
	Point minLoc;
	
	/* Se utiliza la funcion matchtemplate para hacer la comparacion entre IMCOriginal y la Mascara, el resultado lo guardo en result */
	/* The matchtemplate function is used to perform the comparation between IMCOriginal and the template, the result is save in result */
	matchTemplate(IMCOriginal, Mascara, result, CV_TM_CCORR);
	matchTemplate(IMCOriginal, Mascara2, result2, CV_TM_CCORR);
	/* Se normaliza el resultado */
	/* The result is normalized */
	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	normalize(result2, result2, 0, 1, NORM_MINMAX, -1, Mat());
	/* Se clonan los resultados obtenidos en las variables resultbin */
	/* The obtained results are cloned in the variables resultbin */
	Mat resultbin = result.clone();
	Mat resultbin2 = result2.clone();
	/* Se binariza result, el numero indica desde que valor los pixeles seran 1 o 0. Si resultbin > 0.45 -> pixel =1 */
	/* result is binarized, the number indicates each what value the pixels will be 1 or 0. If resultbin > 0.45 -> pixel =1 */
	resultbin = resultbin > 0.45;
	resultbin2 = resultbin2 > 0.45;
	/* No hace nada (Se suponne que sirva para utilizar el minimo local o el maximo local) No se uso porque la variacion 
	   de estos valores no es significativa */
	minMaxLoc(result,&minVal,&maxVal,&minLoc,&maxLoc,Mat());

	/* Se encuentran los contornos y valores de las areas de cada mascara */
	/* The contours and the area value of the template are founded */
	
	/*Mascara  */
	CAMascara = Biggest_Area(binMascara); // auxiliar2
	/* Mascara azul */
	CAMascara2 = Biggest_Area(binMascara2); // auxiliar2
	//Matchtemplate
	CARMascara = Biggest_Area(resultbin); // auxiliar 1
	//Matchtemplate2
	CARMascara2 = Biggest_Area(resultbin2); // auxiliar 1
}

/* Funcion Classification_Process 
 * Classification_Process  function
 
 * Esta funcion determina si el objeto encontrado es una mina utilizando las caracteristicas entregados por Image_Features
 * This function determines if the object found is a lanmine using the features obtained from Image_Features

*/


void Classification_Process(cv::Mat& IM,cv::Mat& IMCOriginal,cv::Mat& IMfinal, cv::Mat& Mascara, cv::Mat IMfiltrada,double CAMascara, double CAMascara2, double CARMascara, double CARMascara2, double AuxiliarArea, int height , int weight, Point& maxLoc, int l, char* Directorio, 
char* Directorio_Error){

	/* En esta seccion de codigo se calculan los errores a partir de los cuales se determina si el objeto detecatado es o no una mina */
	/* In this section of code is calculated the errors which determines if the detected object is or not a landmine */

	
	/* Calculo de Error1, error porcentual entre  el area de la imagen y la de la mascara1 */
	/* Calculus of Error1. Percentual error between the area of the image and  mascara1*/	
	float Error1 = abs((AuxiliarArea - CAMascara)/CAMascara)*100;
	/* Calculo de Error2, error porcentual entre  el area de la mascara1  y el resultado del matchtemplate1 */
	/* Calculus of Error2. Percentual error between the area of mascara1 and the result of matchtemplate1 */
	float Error2 = abs((CARMascara - CAMascara)/CAMascara)*100;
	/* Calculo de Error3, error porcentual entre  el area de la imagen y la de la mascara2 */
	/* Calculus of Error3. Percentual error between the area of the image and mascara2*/
	float Error3 = abs((AuxiliarArea  - CAMascara2)/CAMascara2)*100;
	/* Calculo de Error4, error porcentual entre  el area de la mascara2  y el resultado del matchtemplate2 */
	/* Calculus of Error4. Percentual error between the area of mascara2 and the result of matchtemplate2 */
	float Error4 = abs((CARMascara2 - CAMascara2)/CAMascara2)*100;
	/* Calculo de error 5 que depende de la intensidad de los pixeles en la escala RGB de la imagen, en comparacion con 
	   con los parametros encontrados a partir de la base de datos de fotos con minas*/
	/* Calculus of Error5, this depends of the intensity of  pixels in the RGB scale of the image. The data are compare with  the  		   parameters found using pictures of landmines landmines of data base*/
	int Error5 = Intensity_Classification(height,weight,IMCOriginal);

	/* Para determianar si el objeto detectado es mina o no, se deben cumplir una serie de condiciones dadas por logica 
	   convinatoria, si el objeto es una mina, se pinta un rectangulo rojo y un circulo blanco al rededor de esta para resaltar su 
	   su ubicacion */
	/* To determinate if the detected object is or not a lanmine is necessary satisfy a group of conditions is combinatorial logic.
	   If the object is a landmine, a with circle and a red rectangle are draw around the landmine to highlight their location */
	/* Estos if determinan si la imagen es o no  una mina */
	/* This conditions determine if the detected object is or not a landmine */

	
	Scalar color = CV_RGB(255,255,255);

	if ((Error2 <= 15 || Error1 <=11 || Error3 <= 11 || Error4 <=15) && (Error5 == 1) /*&& (PAOriginal >= (PMascara2-(PMascara2*0.1)))*/){
		if (((Error2+Error1) <=100) || ((Error3+Error4) <=100)){

			
				Mat ContornoImagen = IMfiltrada.clone();
			  	vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;
				
				findContours( ContornoImagen, contours, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

				vector<vector<Point> > contours_poly( contours.size() );
				vector<Point2f>center( contours.size() );
				vector<float>radius( contours.size() );

				for (int i = 0; i < contours.size(); i++)
				{ 
					/* Estas dos lineas de codigo guardan en las variables center y radius el centro y radio de la mina 						para posteriormente graficar un circulo al rededor de la mina detectada */	
					/* These lines of code save the center and radius variables of the mine to draw a circle around the
					   the landmine detected */		
					approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
					minEnclosingCircle( contours_poly[i], center[i], radius[i]);          
				    }

	
			Point matchLoc = maxLoc;
			/* Creacion del rectangulo rojo */
			/* The red rectangle is creted */
			rectangle(IMfinal,matchLoc,Point(matchLoc.x + Mascara.cols,matchLoc.y + Mascara.rows),CV_RGB(255,0,0),2,8,0);
			/* Creacion del circulo blanco */
			/* The white circle is created */
			
			circle( IMfinal, center[l], (int)radius[l], color, 2, 8, 0 );

			/* La variable contador es utilizada para asignar el un numero a la imagen con la mina detectada, la cual sera guardada
			   en la carpeta minas, en una carpeta con el nombre y fecha del comienzo de la mision. El nombre de la mina vine dado
			   por la variable contador y la palabra IM */
			/* The contador variable is used to assign the number of the image with the landmine detected, which will be saved on
			   folder mina of drone_detection package, in a folder with the name and date when begin of the mission. The name 
			   mina is defined with contador variable and the word IM  */
			contador++;
			/* Las variables os,os2, son utilizadas para darle al programa la direccion donde se guarda la imagen con el nombre 
			   contador+IMG y contador+IMG2 (Nombreimage e Nombreimage2 respectivamente), la diferencia entre las imagenes radica
 			   en que la primera es la imagen original y la segunda la imagen con la mina resaltada */
			/* The os,os2 are used to give to the program the adress where the image obtained will be save with the name  
			   contador+IMG and contador+IMG2. IMG is the original image and IMG corresponds to the image with the highlight 				   landmine */
			ostringstream os, os2;
		        os << Directorio << "/IMG_" << contador << ".jpg";
			os2 << Directorio << "/IMGB_" << contador << ".jpg";
        		string Nombreimage = os.str();
			string Nombreimage2 = os2.str();
			/* Se define la variable compression_params para definir el formato de compresion de la imagen a guardar */
			/* The compression_params variable is used to define the compression format of the image which will be save*/
    			vector<int> compression_params;
    			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(100);
			/* Para guardar la imagen se utiliza la funcion imwrite() que recibe como entrada la imagen que se quiere guardar y
			   la direccion y el nombre que se le desea poner */
			/* The imwrite() function is used to save the image. The function uses as entrace the image that will be save and 
			   the direction with the name whish  will be save */
			imwrite(Nombreimage, IM, compression_params);
			imwrite(Nombreimage2, IMfinal, compression_params);
			/* A la variable mina 1 se le asigna el valor de 1 para que en el archivo de errores se indique que esos datos 
			   corresponden a una mina detectada */
			/*  The variable mina takes a 1 to indicate in the error files that this data corresponds at a landmine*/
			mina = 1;
			/* Se asigna a la variable datomina la variable contador para publicar en la GUI el numero de minas detectadas */
			/* The contador variable is assign to the variable datomina to publish on GUI the number of landmines detected */
			snprintf(datomina,250,"Landmines detected: %d ",contador);

		}
		else{
		/* Si no se cumple la condicion 2 se muestra en pantalla el letrero "Mina no detectada" y la variable mina se pone en 0 */
		/* If the condition 2 is shown on the scren "Mina no detectada" and the variable mina returns to 0 */
		putText(IMfinal, "Mina No Detectada", cvPoint(height/4,weight/2), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0,0,255), 1, CV_AA);
		mina = 0;
		} /*Cierre del else 2 */ /* Close else 2 */
	} /*Cierre del if 1 */ /* Close if 1 */
	else{
	/* Si no se cumple la condicion 1 se muestra en pantalla el letrero "Mina no detectada" y la variable mina se pone en 0 */
	/* If the condition 1 is no satisfy is shown on the screen "Mina no detectada" and the mina variable returns to 0*/
	putText(IMfinal, "Mina No Detectada", cvPoint(height/4,weight/2), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0,0,255), 1, CV_AA);
	mina = 0;
	} /*Cierre del else 1 */ /* Close else  1 */

	/* Se crea el archivo dataError donde se almacenaran los diferentes datos de los errores del algoritmo, cada vez que este sea
	   utilizado */
	/* The dataError file is created. In this file the differents error of  algorithm are saved */
	FILE *dataError;
	/* En la variable datalogError se guarda el nombre y la direccion del archivo (dataError) donde se guardaran los errores */
	/* In the datalogErrro is saved the name and adress of the file (dataError) where the errors are save */
 	char datalogError[512];
	//strcpy(datalog,"log.txt");

			
			if ((dataError=fopen(Directorio_Error,"a+"))!=NULL)
			/* Con sprintf() se guarda en datalog los datos de error */
			/* The error data are saved in datalog using sprintf() function */
			sprintf(datalogError,"%d %d %f %f %f %f %d \n",contador2,contador,Error1,Error2,Error3,Error4,mina);
			fprintf(dataError,"%s \n", datalogError);
			/* Cierra datanav con los datos de error guardados */
			/* Datanv is close with the saved data */
			fclose(dataError); 
			
}


/*Funcion detection_landmines -> deteccion
 * detection_landmines -> deteccion function

 
 * Esta función  realiza la deteccion de las minas a partir de la imagen proporcionada por el UAV, ademas 
   almacena en un archivo los diferentes archivo producidos por el programa 
 * This function perform the detection landmines process using the image from the bottom camera of UAV, also save in differents locations
   of the package the differents files produced  by the program.


*/

void detection_landmines(cv::Mat imag, char* Directorio, char* Directorio_Error){

	/* Se clona la imagen adquirida (im) para que no sea modificada por el proceso de Deteccion */
	/* The obtained image is cloned to IM */
	Mat IM=imag.clone();
	/* Se Comprueba que la imagen clonada a IM no sea vacia */
	/* this condition check if the cloned image in IM is not empty */
	if (IM.empty()) 
	{
		/* Si se cumple la condicion se publica el siguiente mensaje */
		/* If the condition is satisfied, the follow message is published */
		cout << "Error : Imagen prueba no se pudo cargar..!!" << endl;
        	//system("pause"); //wait for a key press
		//return -1;
	}
  	

	Mat IMCOriginal, IMfinal, Mascara,IMfiltrada;
	int height,weight,radio,l=0;
	double AuxiliarArea=0,CAMascara,CAMascara2,CARMascara,CARMascara2;
	Point centro,maxLoc;


	Image_Filtering(IM,IMCOriginal,IMfinal,IMfiltrada,AuxiliarArea,height,weight,l);


	Image_Features(IMCOriginal,Mascara,CAMascara,CAMascara2,CARMascara,CARMascara2,maxLoc);
	

	Classification_Process(IM,IMCOriginal,IMfinal,Mascara,IMfiltrada,CAMascara,CAMascara2,CARMascara,CARMascara2,AuxiliarArea,
	height,weight,maxLoc,l,Directorio,Directorio_Error);


	/* Con imshow se genera la ventana "Detection Window" donde se muestra la imagen procesada por detection() */
	/* The imshow() function is use to create the "Detection Window" window where the image processed is shown */
	imshow("Detection Window", IMfinal);
	/* La instruccion waitKey(1) utilizada para mantener visible indefinidamente la imagen publicada */
	/* The waiKey(1) instruction is used to see constantly the published image in the screen */
        waitKey(1); //wait infinite time for a keypress
}


/* main
 
 * Es donde se ejecuta el programa principal de detection.cpp, donde se llama a detection_landmines() y se publican los datos de navegacion 
   en la GUI ademas en este se definen los suscriptores que se incriben a los topicos de ROS para utilizar las variables publicadas en estos.
 * Is where the principal program detection.cpp is executed, where the detection_landmines() is called and the navigation data are publishied
   in the GUI. Moreover in this part of the program the subscribers are created to use the datas published in ROS topics. 

*/

int main( int argc, char** argv ){

/* Se inicia con la funcion init() de ROS y con la afiliacion a drone_detection_GUI que esta definido en el CMakelists.text */
/* The function init() is started. Also the affiliation to drone_detection_GUI is created which is defined in CMakelists.text file */
ros::init(argc, argv,"drone_detection_GUI");

/* Se definen algunas variables de manejo de ROS para su posterior uso en el programa */
/* Some ROS variables are created */
ros::NodeHandle nh_;
ros::NodeHandle camImage;
ros::NodeHandle navdata;
ros::NodeHandle n;

/* Se crea el suscriptor al topico /ardrone/image_raw */
/* A subscriber is created at topic /ardrone/image_raw */
ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, Image_Upload);
/* Se crea el suscriptor al topico /ardrone/navdata */
/* A subscriber is created at topic /ardrone/navdata */
ros::Subscriber Nav = navdata.subscribe("/ardrone/navdata", 5, Navdata_Upload);

/* Se define la frecuencia de ejecucion de los lazos del programa */
/* The processing frequency is defined  */
ros::Rate loop_rate(15);  //simulate 15 fps

/* Se declara un inicio de tiempo en ROS para poder definir un delay exacto en el programa */
/* TThe time of ROS is activated to achieve a exact delay in the program */
ros::Time begin = ros::Time::now();
/* Se define un delay  de 0.04 segundos para que cada iteracion del progama se practiamente 0.25 s, normalmente el programa sin el delay
   se demora 0.21 segundos por ciclo */
/* A delay of 0.04 seconds is created to achieve a final time of processing of 0.25 seconds */
ros::Duration delay = ros::Duration(0.04, 0);

	/* Se crea el objeto a del tipo QApplication para el uso de la GUI */
	/* The QApplication object is created to use on GUI */	
	QApplication a(argc,argv);
	/* Se crea el objeto w de la clase detection_gui, recuerde que esta clase esta definida en detection_gui.h */
	/* A object w of detetion_gui class is created. The class was defined in detection_gui.h */
	detection_gui w;	
	/* Se abre la ventana de la GUI */ 
	/* The GUI is opened */
	w.show();
		

	/*  Se crean variables de tiempo para saber la fecha y hora exactas */
	/*  tiempo y fecha are created to know the current date and hour */
   	time_t rawtime;
  	struct tm * timeinfo;
	/* En directorio y directorio_Error se guardan los errores y datos de navegacion obtenidos, estas variables tienen la ubicacion
	   donde se almacenera el archivo y su nombre */
	/*  In directorio and directorio_Error are saved all the errors and navigation data obtained. This variables are saved in a specific 
	    folder in drone_detection package*/
 	char Directorio [80], Directorio_Error [150];
	int status;
	/* Se obtiene la Fecha actual */
	/* The current date is obtain */
  	time ( &rawtime );
  	timeinfo = localtime (&rawtime);
	/* Se guarda en las diferentes variables directorio solamente año-mes-dia (%F) para  concaternarlo con la direccion donde 
	   se creara la carpeta */
	/* The differents variables "directorio" are saved only year-month-day to be concatenaded in the location where folder will 
	   be created */
  	strftime (Directorio,80,"/home/dell-077/fuerte_workspace/sandbox/drone_detection/minas/%F-%R",timeinfo);	
	strftime (Directorio_Log,100,"/home/dell-077/fuerte_workspace/sandbox/drone_detection/datos/log_%F-%R.txt",timeinfo);
	strftime (Directorio_Error,100,"/home/dell-077/fuerte_workspace/sandbox/drone_detection/datos/Error_%F-%R.txt",timeinfo);	
	/* Crear el directorio en la direccion escrita en "Directorio" */
	/* This part of code creates the folder on location written in "Directorio" */
	status = mkdir(Directorio, S_IRWXU |S_IRWXG | S_IROTH | S_IXOTH);
	//int contador = 0;


/* Este while solo es utilizado para publicar un error si la imagen no fue cargada, observe que la condicion biene dada por 
   imageready que cambia de valor en la funcion Image_Upload */
/* This while() is used to publish a error message if the image don't be load */ 
while(imageready!=1)
	{ 
		/* Se declara la variable parada */ 	 
	        char parada=waitKey(50);
		ROS_INFO(" Waitting for image to be ready\n");
		cout << "Waitting for image to be ready" << endl ;
		ros::spinOnce();
		loop_rate.sleep();
		/* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
		/* This condition is use to stop the program when the user press the r key */
		if( parada=='r'){
			cout << "Adios!" << endl ;
			break;
				}
	}

  /* Se declara un ciclo infinito para caragar los datos de navegacion y procesar las imagenes de entrada de manarea
     constante e inifinita o hasta que finalice el programa */
  /* An infinite loop is created to upload the navigation data and images. Also in the loop the detection 
      process is perform */			
  for (;;){
	/* Se declara contador 2 para determinar cuantas veces itero el ciclo infinito donde se usan las funciones del programa */
	/* The variable contador 2 is creted to know how many times the algorithm itarate in the for() loop */	
	contador2++;
	/* Se declara la variable parada */ 
	/* The parada variable is created */
	char parada=waitKey(50);
	
	/* Se guarda la imagen obtenida con el drone en la variable IM */
	/* The obtained image from the drone is saved in variable IM*/
	Mat IM=cv_ptr->image;
	/* Se activa el delay de ROS */
	/* The ROS delay is activated*/
	delay.sleep();
	/* Se envia a IM a la funcion deteccion para realizar el proceso de deteccion y filtrado de ruido */
	/* The IM variable is sent to the function detection_landmines to perform the detection process */
	detection_landmines(cv_ptr->image,Directorio,Directorio_Error);
				/* Declaracion de IM2 para almacenar en esta la imagen obtenida pero en el espacio BGR, esto se 
				   hace porque la funcion Qimage cambia los colores de la imagen de RGB a BGR */
				/* This part of  code is use only to save the obtained image in BGR format because the Qimage function
				changes the colors of the image RGB to BGR*/
				Mat IM2;
				cvtColor(IM, IM2, CV_BGR2RGB);
				/* Se utiliza QImage para publicar en la GUI la imagen obtenida del drone en el formato de Qt*/
				/* QImage is use to publish the image from the bottom camera of the UAV in Qt format*/
				QImage imageout=QImage((uchar*) IM2.data, IM2.cols, IM2.rows, IM2.step, QImage::Format_RGB888);
				/* Se publican en la GUI los datos enviados por el drone en los diferentes labels de la GUI */
				/* The differents labels publish on GUI the navigation data of the UAV  */
				w.ui.labelimage->setPixmap(QPixmap::fromImage(imageout));
				w.ui.labelAltitude->setText(datoaltd);
				w.ui.labelVelocity_x->setText(datovelx);
     				w.ui.labelVelocity_y->setText(datovely);
     				w.ui.labelVelocity_z->setText(datovelz);
     				w.ui.labelPitch->setText(datoPitch);
     				w.ui.labelYaw->setText(datoYaw);
     				w.ui.labelRoll->setText(datoRoll);
     				w.ui.labelBattery->setText(datoBP);
				w.ui.labelmine->setText(datomina);

				/* Esta condicion es utilizada para mostrar en la GUI la ultima mina detectada, si contador es 0 es 
				   porque aun no se a detectado ninguna mina */
				/* This condition is use to show on the GUI the last landmine detected. If contardor is 0, the landmine 
				has not been detected */
				if (contador>0){
				/* El procedimiento con la variable os y directorio se hace para buscar la ubicacion de la ultima mina 
				   guardada */
				/* The os and directorio variables are used to finde the location of the last landmine detected */
				ostringstream os2;
				os2 << Directorio << "/IMGB_" << contador << ".jpg";
				string image3 = os2.str();
				/* Se caraga el la imagen con la ultima mina guardad con imread() */
				/* The image is loaded with the latest detected mine */
				Mat IMmina= imread(image3, CV_LOAD_IMAGE_UNCHANGED);
				/* Como en la GUI el tamaño de la imagen es menor esta se reeescaliza con resize */
				/* The image is rezise */
				Mat IMsize=Mat(320,180,CV_8UC1);
				resize(IMmina,IMsize,Size(320,180),0 ,0, CV_INTER_LINEAR);
				/* Se pasa la imagen escalizada al espacio BGR */
				/* The resized image is transformed to the BGR space */
				cvtColor(IMsize, IMsize, CV_BGR2RGB);
				QImage imageoutmina=QImage((uchar*) IMsize.data, IMsize.cols, IMsize.rows, IMsize.step, QImage::Format_RGB888);
				/* Finalmente se carga en la GUI la foto con la ultima mina detectada */
				/* Finally the image with the last detected landmine is upload on the GUI */
				w.ui.labelfoto->setPixmap(QPixmap::fromImage(imageoutmina));
				} 
	
	ros::spinOnce();
	loop_rate.sleep();
	/* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
	/* This condition is used to stop the program when the user press the r key */
	if(parada=='r'){
		cout << "Adios!" << endl ;
		break;
		}
  } 

return 0;
/* Nota el comando que le indica al Drone que active la imagen de la camara inferior  es "rosservice call /ardrone/togglecam" */
/* "rosservice call  /ardrone/togglecam" is the command which actives the image from bottom camera in the AR Drone 2 */
}
