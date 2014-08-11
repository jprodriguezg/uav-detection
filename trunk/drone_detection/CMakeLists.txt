#
# CMakeLists.txt
# Este archivo hace parte de drone_detection
# Autores: Juan Pablo Rodríguez y Carolina Castiblanco
# Informacion: j_rodriguezg@javeriana.edu.co   jenny.castiblanco@javeriana.edu.co
#


cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
FIND_PACKAGE( OpenCV REQUIRED )
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# Autors: Juan Pablo Rodríguez (j_rodriguez@javeriana.edu.co)
#	  Jenny Carolina Castiblanco (jenny.castiblanco@javeriana.edu.co)



rosbuild_init()

# Se vincula a Qt al paquete para el uso de la GUI
FIND_PACKAGE(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
INCLUDE(${QT_USE_FILE})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories (${OpenCV_INCLUDE_DIRS})
# Se agregan las extenciones para vincular los ejecutables con los nodos de ROS que se encuentran en la carpeta launch
#rosbuild_add_executable(Prueba1 src/Prueba1.cpp)
#
#rosbuild_add_executable(detection2 src/detection2.cpp)
#
rosbuild_add_executable(record src/record.cpp)

#----------------------------------- Se agrega esta parte del codigo para utilizar Qt y los .h  en ROS----------------------------------------

ADD_DEFINITIONS(${QT_DEFINITIONS})

SET(GUI_SOURCES src/detection.cpp)
SET(GUI_HEADERS src/GUI/detection_gui.h)
SET(GUI_FORMS src/GUI/detection_gui.ui)
SET(GUI_RESOURCES)

qt4_automoc(${GUI_SOURCES})
QT4_WRAP_CPP(GUI_HEADERS_MOC ${GUI_HEADERS})
QT4_WRAP_UI(GUI_FORMS_HEADERS ${GUI_FORMS})
QT4_ADD_RESOURCES(GUI_RESOURCES_RCC ${GUI_RESOURCES})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

# Finalmente se vinculan todos los headers, forms y recursos a detection.cpp 
rosbuild_add_executable(drone_detection_GUI src/detection.cpp ${GUI_SOURCES} ${GUI_HEADERS_MOC} ${GUI_FORMS_HEADERS} ${GUI_RESOURCES_RCC})
target_link_libraries(drone_detection_GUI ${OpenCV_LIBS})
target_link_libraries(record ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(drone_detection_GUI ${QT_LIBRARIES})


