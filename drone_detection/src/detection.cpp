 /**

 *  detection.cpp
 *
 *  Este archivo es parte de drone_detection
 *  Creado por: Juan Pablo Rodríguez y Carolina Castiblanco
 *  Informacion: j_rodriguezg@javeriana.edu.co   jenny.castiblanco@javeriana.edu.co
 *  
 *  En este archivo se encuentra la funcion detection() que se encarga del proceso de deteccion de minas a partir de 
 *  las imagenes obtenidas del topico /ardrone/image_raw. Ademas publica en la GUI del sistema los datos de vuelo 
 *  provenientes de /ardrone/navdata

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
#include "ardrone_autonomy/Navdata.h"
#include <ros/time.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
#include <sys/stat.h>

/* Includes para el funcionamiento de la GUI con Qt4 */

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


/*Funcion Navdata_Upload  
 
 * Esta funcion se encarga de conectarce a los datos de vuelo provenientes de /ardrone/navdata y guardarlos en un archivo con la fecha y
 * hora del comienzo de la mision
 * Los datos se obtienen a partir del subscriptor Nav declarado en el main del programa

*/

void Navdata_Upload(const ardrone_autonomy::NavdataConstPtr navdataPtr){

/* Declaracion del archivo datanav y el caracter datalog en donde se almacenaran los datos de vuelo para guardarlos en datanv */
FILE *datanav;
char datalog[512];

if ((datanav=fopen(Directorio_Log,"a+"))!=NULL)
/* Con sprintf() se guarda en datalog los datos de vuelo */
sprintf(datalog,"%d %f %f %f %d %f %f %f %f %f %f\n",contador2, navdataPtr->rotX,navdataPtr->rotY,navdataPtr->rotZ,navdataPtr->altd, navdataPtr->vx,navdataPtr->vy,navdataPtr->vz,navdataPtr->ax,navdataPtr->ay,navdataPtr->az);
fprintf(datanav,"%s \n", datalog);
/* Cierra datanav con los datos de navegacion guardados */
fclose(datanav);

	/* A partir de este punto se declaran en las varibles globales dato_ los datos provenientes de los diferentes sensores del drone 
           con snprintf() se almacena en la variable, el nombre de esta y el valor extraido del topico /ardrone/navdata. La /asignacion
   	   (int)navdataPtr->altd guarda en la variable navdataPtr el valor proveniente de altd */ 

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

/*Funcion Image_Upload  
 
 * Esta funcion se encarga de obtener la imagen proveniente del topico /ardrone/image_raw la cual sera utilizada para el programa para 
 * efectuar la deteccion de minas 

*/
 
void Image_Upload(const sensor_msgs::ImageConstPtr& msg)
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

/*Funcion Remove_Medium_Objects  antes llamada -> removeSmallBlobs
 
 * Esta funcion se encarga de elimiar de la imagen de entrada im, los objetos con un tamaño menor que sizec con el objetivo de elimar 
 * ruido de la imagen

*/
//**********************************************
void Remove_Medium_Objects(cv::Mat& im, double sizec){
	/* Declaracion de los vectores contours y hierarchy */
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/* Se clona la imagen adquirida (im) para que no sea modificada al ser utilizada en con función findContours(), esta funcion es
           utilizada par encontrar todos los contornos de los objetos de la imagen */
	Mat IMRSB = im.clone();	
	findContours( IMRSB, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	/* El siguiente ciclo busca recorre los diferentes contornos para analizar si el área esta por debajo o no del rango definido 
           para determinar si deben ser eliminados o no */
	for (int i = 0; i < contours.size(); i++)
    	{
        	/* Calcula el area Correspondiente a cada Contorno encontrado con la funcion findContours() */
        	double area = contourArea(contours[i]);
        	/* Condicion (Rango) para determinar si la imagen debe o no ser eliminada de la imagen*/
        	if (area > 0 && area <= sizec)
		/* Remueve los objetos con areas  que se Encuentren en el Rango, para eliminar el contorno, se dibuja encima de este 
		    pixeles de color negro, note que el dibujo se hace directamente sobre la entrada lo que la afecta directamente*/
            	drawContours(im, contours, i, CV_RGB(0,0,0), -1);
    	}
}

/*Funcion Pixels_Sum antes llamada ->Suma_Pixeles
 
 * Esta funcion suma el numero de pixeles blancos que se encuentran en la imagen de entrada im, para realizar esta accion se recorre la 
 * imagen mediente 2 ciclos que van hasta el largo y ancho de la imagen (h y w)  que son entradas de la funcion

*/

int Pixels_Sum(cv::Mat im, int h, int w) {
	/* Se declara la funcion auxiliarim que es del tipo IplImage para utilizar la funcion cVGet2D() y la variable s del tipo CVScalar */
	IplImage auxiliarim = im;
	CvScalar s;
	/* Se declara la variable pixelesblancos que es donde se almacenaran el numero de pixeles blancos encontrados */
	int pixelesblancos = 0;
	for (int i = 0; i < h; i++){
		for (int k = 0; k < w; k++){
		/* Mediante cvGet2D se almacena en S el valor numerico del pixel analisado en la imagen auxiliarim en la posiion (i,k) */
		s=cvGet2D(&auxiliarim,i,k);
		/* Si el pixel leido tiene un valor diferente de 0, se incrementa la variable pixelesblancos que representa el numero de
 		pixeles blancos en la imagen */
		if (s.val[0] != 0) 
		pixelesblancos ++;
		}
	    }
	return pixelesblancos;
}

/* Funcion Biggest_Area antes llamada ->Encontrar_Contorno
 
 * Esta función analiza todas las áreas de los contornos de la imagen adquirida (im) para entregar en la variable AuxiliarArea el area mas    
 * grande encontrada, junto a su posicion (l) en la variable contours 

*/

double Biggest_Area(cv::Mat im){
	/* Se crean e inicializan las variables AuxiliarArea, contours y hierarchy */
	double AuxiliarArea = 0;
 	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/* Se clona la imagen adquirida (im) para que no sea modificada por la función findContours() */
	Mat IMEC = im.clone();
	findContours( IMEC, contours, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	/* Mediate el siguiente ciclo se analizan todos lso contornos para determinar cual tiene el area mas grande*/
	for (int i = 0; i < contours.size(); i++)
	{
		/* En la variable area se almacena el area del contorno analisado, si el area encontrada es mayor que la anterior 
		   se almacena en la variable AuxiliarArea el valor de la nueva area encontrada */
		double area = contourArea(contours[i]);
		if (AuxiliarArea <= area)
		    AuxiliarArea=area;	
		    
	}
	/* Al finalizar el ciclo en la variable Auxiliar area se almacena en la variable AuxiliarArea el area mas grande encontrada */
	return AuxiliarArea;
}

int Intensity_Classification(int h, int w, Mat IN){
// Suprimir Pasto
	int balde_green = 0, balde_blue = 0, balde_red =0, Resultado =0; 
	for( int i = 0; i < h; i++ )
	{ 
	  for( int k = 0; k < w; k++ )
       	  { 
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
	if ((balde_green+balde_blue+balde_red)>= 6500 && (balde_green+balde_blue+balde_red)<= 28000)
	{
		return Resultado = 1;
	}
	else
		return Resultado = 0; 
}

void Image_Filtering(cv::Mat& IM, cv::Mat& IMCOriginal, cv::Mat& IMfinal,cv::Mat& IMfiltrada, double& AuxiliarArea, int& height , 
int& weight, int& l){


	// Le quite el mat IMfiltrada y la agrege como entrada ---------------
	Mat IM_GRAY, IM_ERODE;
	/* Cambiar Imagen Original IM a escala de Grises */
	cvtColor(IM, IM_GRAY, CV_RGB2GRAY);
	//Cuando cvtColor no quiera funcionar, usar: Mat IM_GRAY = imread(image,IMREAD_GRAYSCALE);
	/* Binarizar la imagen IM_GRAY */
	Mat IMbin = IM_GRAY > 128;
	
	/* Se crea una estrucutra morfologica (elipse) para utilizar la funcion erode */
	Mat disco = getStructuringElement(MORPH_ELLIPSE, Size(7,7));
	/* Aplicar la funcion Erode (erosíon) a la Imagen IMbin */
	erode(IMbin, IM_ERODE, disco);
	Mat IMpos = IM_ERODE.clone();
	/* Se remueven los objetos medianos con Remove_Medium_Objects() */
	Remove_Medium_Objects(IMpos,5000);
	/* Se crea una estrucutra morfologica (elipse) para utilizar la funcion erode */
	Mat disco1 = getStructuringElement(MORPH_ELLIPSE, Size(4,4));
	/* Se recupueran los trozos de imagen perdidos al utilizar erode en los objetos grandes */
	dilate(IMpos, IMfiltrada, disco1);


	
	IplImage IMB = IM;
	/* Se encuentra el ancho (w) y alto (h) de la imagen */
	height = cvGetDimSize(&IMB, 0);
	weight = cvGetDimSize(&IMB, 1);

	int PixelesBlancos = 0; 

	/* Se encuentra el numero de pixeles blancos en la imagen utilizando la funcion Pixels_Sum */
	PixelesBlancos=Pixels_Sum(IMfiltrada,height,weight);

	/* Las siguientes instrucciones son utilizadas si el proceso de filtrado anterior borro de la imagen todos los objetos que tienen 
	   un tamaño semejante al de una mina, si se cumple la condicion se vuelve a efectuar el proceso de filtrado pero esta vez
	   utilizando un valor menor en Remove_Medium_Objects() */

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
	

	/* Borrar Areas pequeñas, para dejar la mas grande, note que para este proceso no se utiliza encontrar la fucion
	   Biggest_Area porque en este caso se necesita extraer la posicion del contorno y  llenar las variables
	   center y radius, para su futuro uso en el programa */

	
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
		/* Estas dos lineas de codigo guardan en las variables center y radius el centro y radio de la mina para 			   posteriormente graficar un circulo al rededor de la mina detectada */			
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
	drawContours( IMCvacio, contours, l, CV_RGB(255,255,255), 2, 8, hierarchy, 0, Point() );

	/* Se genera en IMCRelleno el contorno con la imagen del mayor tamaño pero adicionalmente se rellena el contorno con pixeles blancos */
	drawContours(IMCRelleno, contours, l, CV_RGB(255,255,255), -1); 

	/* Se pinta el contorno encontrada en la imagen original */
	drawContours( IMfinal, contours, l, CV_RGB(255,255,255), 2, 8, hierarchy, 0, Point() );


	/* En esta parte se clona en IMCOriginal la imagen IMCRelleno para posteriormente asignarcela  IMCOriginalaux del tipo IplImage,
 	adicionalemte en IMaux se clona IM y es enviada a IMaux2. A continuacion se recorre la imagen para asignar en IMCOriginalaux el valor

 	del pixel de la posicion i,k  de IMaux2. La idea de este procedimiento es poner en la imagen llena de pixeles negros, los pixeles

 	originales de la mina para obtener una imagen en donde se observe unicamente la mina o el objeto mas grande de la imagen original con 		sus respectivos colores */

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


/*Funcion detection_landmines -> deteccion
 
 * Esta función analiza realiza la deteccion de las minas a partir de la imagen proporcionada por el UAV (entrada image, ademas 
   almacena en un archivo los diferentes errores generados en la funcion 

*/

void Image_Features(cv::Mat IMCOriginal,cv::Mat& Mascara,double& CAMascara, double& CAMascara2,double& CARMascara, double& CARMascara2, Point& maxLoc){

/* En esta parte del codigo se realiza el tratamiento de cada una de las mascaras o template del sistema */
	
	/* Se generan las variables donde se almacenaran las mascaras */
	Mat binMascara, binMascara2; 
	/* Se cargan las mascaras a partir de imread() */
	/* Mascara gris */
	Mascara= imread("/home/dell-077/fuerte_workspace/sandbox/drone_detection/src/Mascaras/1m/Mascara1m1.jpg", CV_LOAD_IMAGE_UNCHANGED);
	/* Mascara Azul */
	Mat Mascara2 = imread("/home/dell-077/fuerte_workspace/sandbox/drone_detection/src/Mascaras/1m/Mascara1m3.jpg", CV_LOAD_IMAGE_UNCHANGED);
	/* Se utiliza un condicional para enviar un error si no se cargaron las mascaras */
	if (Mascara.empty() || Mascara2.empty()){
		cout << "Error : Imagen mascara no se pudo cargar..!!" << endl;
        	//system("pause"); //wait for a key press
		//return -1;
	}
	/* Se pasan las mascaras  a escala de grises */
	cvtColor(Mascara, binMascara, CV_RGB2GRAY);
	cvtColor(Mascara2, binMascara2, CV_RGB2GRAY);
	/* Binarizacion de las mascaras */
	binMascara = binMascara > 128;
	binMascara2 = binMascara2 > 128;

	/* Se crean varaibles para almacenar las valores de los contornos de las mascaras */

	Mat result, result2;
	double minVal; double maxVal;
	Point minLoc;
	
	/* Se utiliza la funcion matchtemplate para hacer la comparacion entre IMCOriginal y la Mascara, el resultado lo guardo en result */
	matchTemplate(IMCOriginal, Mascara, result, CV_TM_CCORR);
	matchTemplate(IMCOriginal, Mascara2, result2, CV_TM_CCORR);
	/* Se normaliza el resultado */
	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
	normalize(result2, result2, 0, 1, NORM_MINMAX, -1, Mat());
	/* Se clonan los resultados obtenidos en las variables resultbin */
	Mat resultbin = result.clone();
	Mat resultbin2 = result2.clone();
	/* Se binariza result, el numero indica desde que valor los pixeles seran 1 o 0. Si resultbin > 0.45 -> pixel =1 */
	resultbin = resultbin > 0.45;
	resultbin2 = resultbin2 > 0.45;
	/* No hace nada (Se suponne que sirva para utilizar el minimo local o el maximo local) No se uso porque la variacion 
	   de estos valores no es significativa */
	minMaxLoc(result,&minVal,&maxVal,&minLoc,&maxLoc,Mat());

	/* Se encuenntran los contronos y valores de las areas de cada mascara */
	
	/*Mascara  */
	CAMascara = Biggest_Area(binMascara); // auxiliar2
	/* Mascara azul */
	CAMascara2 = Biggest_Area(binMascara2); // auxiliar2
	//Matchtemplate
	CARMascara = Biggest_Area(resultbin); // auxiliar 1
	//Matchtemplate2
	CARMascara2 = Biggest_Area(resultbin2); // auxiliar 1
}

void Classification_Process(cv::Mat& IM,cv::Mat& IMCOriginal,cv::Mat& IMfinal, cv::Mat& Mascara, cv::Mat IMfiltrada,double CAMascara, double CAMascara2, double CARMascara, double CARMascara2, double AuxiliarArea, int height , int weight, Point& maxLoc, int l, char* Directorio, 
char* Directorio_Error){

	/* En esta seccion de codigo se calculan los errores a partir de los cuales se determina si el objeto detecatado es o no una mina */

	
	/* Calculo de Error1, error porcentual entre  el area de la imagen y la de la mascara1 */	
	float Error1 = abs((AuxiliarArea - CAMascara)/CAMascara)*100;
	/* Calculo de Error2, error porcentual entre  el area de la mascara1  y el resultado del matchtemplate1 */
	float Error2 = abs((CARMascara - CAMascara)/CAMascara)*100;
	/* Calculo de Error3, error porcentual entre  el area de la imagen y la de la mascara2 */
	float Error3 = abs((AuxiliarArea  - CAMascara2)/CAMascara2)*100;
	/* Calculo de Error4, error porcentual entre  el area de la mascara2  y el resultado del matchtemplate2 */
	float Error4 = abs((CARMascara2 - CAMascara2)/CAMascara2)*100;
	/* Calculo de error 5 que depende de la intensidad de los pixeles en la escala RGB de la imagen, en comparacion con 
	   con los parametros encontrados a partir de la base de datos de fotos con minas*/
	int Error5 = Intensity_Classification(height,weight,IMCOriginal);

	/* Para determianar si el objeto detectado es mina o no, se deben cumplir una serie de condiciones dadas por logica 
	   convinatoria, si el objeto es una mina, se pinta un rectangulo rojo y un circulo blanco al rededor de esta para resaltar su 
	   su ubicacion */
	/* Estos if determinan si la imagen es o no  una mina */

	
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
					approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
					minEnclosingCircle( contours_poly[i], center[i], radius[i]);          
				    }

	
			Point matchLoc = maxLoc;
			/* Creacion del rectangulo rojo */
			rectangle(IMfinal,matchLoc,Point(matchLoc.x + Mascara.cols,matchLoc.y + Mascara.rows),CV_RGB(255,0,0),2,8,0);
			/* Creacion del circulo blanco */
			
			
			circle( IMfinal, center[l], (int)radius[l], color, 2, 8, 0 );

			/* La variable contador es utilizada para asignarl un numero a la imagen con la mina detectada, la cual sera guardada
			   en la carpeta minas, en una carpeta con el nombre y fecha del comienzo de la mision. El nombre del mina vine dado
			   por la variable contador y la palabra IM */
			contador++;
			/* Las variables os,os2, son utilizado para darle al programa la direccion donde se guarda la imagen con el nombre 
			   contador+IMG y contador+IMG2 (Nombreimage e Nombreimage2 respectivamente), la diferencia entre las imagenes radica
 			   en que la primera es la imagen original y la segunda la imagen con la mina resaltada */
			ostringstream os, os2;
		        os << Directorio << "/IMG_" << contador << ".jpg";
			os2 << Directorio << "/IMGB_" << contador << ".jpg";
        		string Nombreimage = os.str();
			string Nombreimage2 = os2.str();
			/* Se define la variable compression_params para definir el formato de compresion de la imagen a guardar */
    			vector<int> compression_params;
    			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(100);
			/* Para guardar la imagen se utiliza la funcion imwrite() que recibe como entrada la imagen que se quiere guardar y
			   la direccion y el nombre que se le desea poner */
			imwrite(Nombreimage, IM, compression_params);
			imwrite(Nombreimage2, IMfinal, compression_params);
			/* A la variable mina 1 se le asigna el valor de 1 para que en el archivo de errores se indique que esos datos 
			   corresponden a una mina detectada */
			mina = 1;
			/* Se asigna a la variable dato mina el la variable contador para publicar en la GUI el numero de minas detectadas */
			snprintf(datomina,250,"Landmines detected: %d ",contador);

		}
		else{
		/* Si no se cumple la condicion 2 se muestra en pantalla el letrero "Mina no detectada" y la variable mina se pone en 0 */
		putText(IMfinal, "Mina No Detectada", cvPoint(height/4,weight/2), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0,0,255), 1, CV_AA);
		mina = 0;
		} /*Cierre del else 2 */
	} /*Cierre del if 1 */
	else{
	/* Si no se cumple la condicion 1 se muestra en pantalla el letrero "Mina no detectada" y la variable mina se pone en 0 */
	putText(IMfinal, "Mina No Detectada", cvPoint(height/4,weight/2), FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0,0,255), 1, CV_AA);
	mina = 0;
	} /*Cierre del else 1 */

	/* Se crea el archivo dataError donde se almacenaran los diferentes datos de los errores del algoritmo, cada vez que este sea
	   utilizado */
	FILE *dataError;
	/* En la variable datalogError se guarda el nombre y la direccion del archivo (dataError) donde se guardaran los errores */
 	char datalogError[512];
	//strcpy(datalog,"log.txt");

			
			if ((dataError=fopen(Directorio_Error,"a+"))!=NULL)
			//Con sprintf() se guarda en datalog los datos de error 
			sprintf(datalogError,"%d %d %f %f %f %f %d \n",contador2,contador,Error1,Error2,Error3,Error4,mina);
			fprintf(dataError,"%s \n", datalogError);
			//Cierra datanav con los datos de error guardados 
			fclose(dataError); 
			
}



void detection_landmines(cv::Mat imag, char* Directorio, char* Directorio_Error){

	/* Se clona la imagen adquirida (im) para que no sea modificada por el proceso de Deteccion */
	Mat IM=imag.clone();
	/* Se Comprueba que la imagen clonada a IM no sea vacia */
	if (IM.empty()) 
	{
		/* Si se cumple la condicion se publica el siguiente mensaje */
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
	imshow("Detection Window", IMfinal);
	/* La instruccion waitKey(1) utilizada para mantener visible indefinidamente la imagen publicada */
        waitKey(1); //wait infinite time for a keypress
}


/* main
 
 * Es donde se ejecuta el programa principal de detection.cpp, donde se llama a detection() y se publican los datos de navegacion en la GUI
   ademas en este se definen los suscriptores que se incriben a los topicos de ROS para utilizar las variables publicadas en estos.

*/

int main( int argc, char** argv ){

/* Se inicia con la funcion init() de ROS y con la afiliacion a drone_detection_GUI que esta definido en el CMakelists.text */
ros::init(argc, argv,"drone_detection_GUI");

/* Se definen algunas variables de manejo de ROS para su posterior uso en el programa */
ros::NodeHandle nh_;
ros::NodeHandle camImage;
ros::NodeHandle navdata;
ros::NodeHandle n;

/* Se crea el suscripto al topico /ardrone/image_raw */
ros::Subscriber img = camImage.subscribe("/ardrone/image_raw", 5, Image_Upload);
/* Se crea el suscripto al topico /ardrone/navdata */
ros::Subscriber Nav = navdata.subscribe("/ardrone/navdata", 5, Navdata_Upload);

/* Se define la frecuencia de ejecucion de los lazos del programa */
ros::Rate loop_rate(15);  //simulate 15 fps

/* Se declara un inicio de tiempo en ROS para poder definir un delay exacto en el programa */
ros::Time begin = ros::Time::now();
/* Se define un delay  de 0.04 segundos para que cada iteracion del progama se practiamente 0.25 s, normalmente el programa sin el delay
   se demora 0.21 segundos por ciclo */
ros::Duration delay = ros::Duration(0.04, 0);

	/* Se crea el objeto a del tipo QApplication para el uso de la GUI */	
	QApplication a(argc,argv);
	/* Se crea el objeto w de la clase detection_gui, recuerde que esta clase esta definida en detection_gui.h */
	detection_gui w;	
	/* Se abre la ventana de la GUI */ 
	w.show();
		

	/*  Se ceran variables de tiempo para saber la fecha y hora exactas */
   	time_t rawtime;
  	struct tm * timeinfo;
	/* En directorio y directorio_Error se guardan los errores y datos de navegacion obtenidos, estas variables tienen la ubicacion
	   donde se almacenera el archivo y su nombre */
 	char Directorio [80], Directorio_Error [150];
	int status;
	/* Se obtiene la Fecha actual */
  	time ( &rawtime );
  	timeinfo = localtime (&rawtime);
	/* Se guarda en las diferentes variables directorio solamente año-mes-dia (%F) para  concaternarlo con la direccion donde 
	   se creara la carpeta */
  	strftime (Directorio,80,"/home/dell-077/fuerte_workspace/sandbox/drone_detection/minas/%F-%R",timeinfo);	
	strftime (Directorio_Log,100,"/home/dell-077/fuerte_workspace/sandbox/drone_detection/datos/log_%F-%R.txt",timeinfo);
	strftime (Directorio_Error,100,"/home/dell-077/fuerte_workspace/sandbox/drone_detection/datos/Error_%F-%R.txt",timeinfo);	
	/* Crear el directorio en la direccion escrita en "Directorio" */
	status = mkdir(Directorio, S_IRWXU |S_IRWXG | S_IROTH | S_IXOTH);
	//int contador = 0;


/* Este while solo es utilizado para publicar un error si la imagen no fue cargada, observe que la condicion biene dada por 
   imageready que cambia de valor en la funcion Image_Upload */ 
while(imageready!=1)
	{ 
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
  for (;;){
	/* Se declara contador 2 para determinar cuantas veces itero el ciclo infinito donde se usan las funciones del programa */	
	contador2++;
	/* Se declara la variable parada */ 
	char parada=waitKey(50);
	
	/* Se guarda la imagen obtenida con el drone en la variable IM */
	Mat IM=cv_ptr->image;
	/* Se activa el delay de ROS */
	delay.sleep();
	/* Se envia a IM a la funcion deteccion para realizar el proceso de deteccion y filtrado de ruido */
	detection_landmines(cv_ptr->image,Directorio,Directorio_Error);
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
				w.ui.labelmine->setText(datomina);

				/* Esta condicion es utilizada para mostrar en la GUI la ultima mina detectada, si contador es 0 es 
				   porque aun no se a detectado ninguna mina */
				if (contador>0){
				/* El procedimiento con la variable os y directorio se hace para buscar la ubicacion de la ultima mina 
				   guardada */
				ostringstream os2;
				os2 << Directorio << "/IMGB_" << contador << ".jpg";
				string image3 = os2.str();
				/* Se caraga el la imagen con la ultima mina guardad con imread() */
				Mat IMmina= imread(image3, CV_LOAD_IMAGE_UNCHANGED);
				/* Como en la GUI el tamaño de la imagen es menor esta se reeescaliza con resize */
				Mat IMsize=Mat(320,180,CV_8UC1);
				resize(IMmina,IMsize,Size(320,180),0 ,0, CV_INTER_LINEAR);
				/* Se pasa la imagen escalizada al espacio BGR */
				cvtColor(IMsize, IMsize, CV_BGR2RGB);
				QImage imageoutmina=QImage((uchar*) IMsize.data, IMsize.cols, IMsize.rows, IMsize.step, QImage::Format_RGB888);
				/* Finalmente se carga en la GUI la foto con la ultima mina detectada */
				w.ui.labelfoto->setPixmap(QPixmap::fromImage(imageoutmina));
				} 
	
	ros::spinOnce();
	loop_rate.sleep();
	/* Esta condicion es utilizada para detener el programa al oprimir la tecla r */
	if(parada=='r'){
		cout << "Adios!" << endl ;
		break;
		}
  } 

return 0;
/* Nota el comando que le indica al Drone que active la imagen de la camara inferior  es "rosservice call /ardrone/togglecam" */
}
