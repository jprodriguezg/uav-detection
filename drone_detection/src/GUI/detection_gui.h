 /**

 *  detection_gui.h
 *
 *  Este archivo es parte de drone_detection
 *  Creado por: Juan Pablo Rodríguez y Carolina Castiblanco
 *  Informacion: j_rodriguezg@javeriana.edu.co   jenny.castiblanco@javeriana.edu.co
 *  
 *  En este archivo se declara la clase detection_gui para su uso en detection.cpp

 */



#ifndef DETECTION_GUI_H
#define DETECTION_GUI_H
#include "ui_detection_gui.h"


/* De claracion de la variable Ui, la cual se utilizara como apuntador para el envio de datos a la GUI */
namespace Ui
{class detection_gui;
}

/* Creacion de la clase detection_gui en la cual se encuentrara la defincion de detection_gui ui que sera del tipo Ui */
class detection_gui :
        public QWidget
        
{
    Q_OBJECT

public:
    explicit detection_gui(QWidget *parent = 0);
    ~detection_gui();
    
    /* Declaracion de detection_gui ui */
    Ui::detection_gui ui;

};

#endif /*DETECTION_GUI*/
