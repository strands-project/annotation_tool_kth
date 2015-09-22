3D ANNOTATION TOOL
==================
Author: Adrià Gallart del Burgo

E-mail: adriagallart@gmail.com 

3D Annotation Tool is an application designed to annotate objects in a point
cloud scene. Initially, it was developed with the aim to annotate the objects of
a table.

Contents
-----------------------
 - data: folder that contains some examples of .pcd files and annotations (.xml file).
 - documentation: folder that contains the user guide.
 - icons: folder with all the icons used.
 - src: this folder contains the source code.
 - annotation_tool.pro: QtCreator project file.
 - CMakeLists.txt: instructions for the installation.
 - README.txt: readme file.
 - info_app.xml: .xml file with information used for the application.
 - info_objects.xml: .xml file with the list of objects annotated by the user.
 

System requirements
-------------------------------
Two options are available to install the prerequisites for the 3D Annotation
Tool. The first requires the previous installation of the ROS Groovy.

If the ROS Groovy is installed in your machine:
	
1. Make sure that the pcl package is installed inside the ros. If it is not 
 	   installed:  
`sudo apt-get install ros-groovy-pcl`
	
2. Install the vtk-qt library:  
`sudo apt-get install libvtk5.8-qt4`

Without ROS Groovy If the ROS Groovy is not installed, the pcl library
has to be installed. The following is required:
	
1. PCL library:  
`sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl`  
`sudo apt-get update`  
`sudo apt-get install libpcl-all-dev`  

2. VTK-QT library:  
`sudo apt-get install libvtk5.8-qt`

Installation
--------------------------------

To install the annotation tool:

1. `cd /your/path/3d_annotation_tool/ `

2. `mkdir build`

3. `cd build`

4. `cmake ..`

5. `make`

Use
--------------------------------

To use the annotation tool:

1. `cd /your/path/3d_annotation_tool/`

2. `cd bin`

3. `./Annotation_tool`
