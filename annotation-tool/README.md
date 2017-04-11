3D ANNOTATION TOOL
==================
Author: Adrià Gallart del Burgo

Co-author: Balasubramanian Rajasekaran, Akshaya Thippur

Emails: bara@kth.se, akshaya@kth.se


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
3D Annotation Tool requires installation of the ROS Indigo and catkin as prerequisites.

If (ros-indigo-desktop-full) is installed in your machine, you can proceed 
with the installation of the 3D Annotation Tool.

Else, make sure the following packages are installed

1. Install the catkin library:
`sudo apt-get install ros-indigo-catkin`
	
2. Make sure that the pcl package is installed:
`sudo apt-get install libpcl-1.7-all-dev` 

3. Install the vtk-qt library:
`sudo apt-get install libvtk5.8-qt4`


Installation
--------------------------------
To install the annotation tool:

With catkin:

1. Git clone or extract the files into your catkin workspace (`/catkin_workspace_path/src/3d_annotation_tool/`)

2. `cd /catkin_workspace_path/`

3. `catkin_make`


Using the tool
--------------------------------
To use the annotation tool:

With catkin:

1. `cd /catkin_workspace_path/devel/lib/3d_annotation_tool/`

2. `cd annotation-tool`

3. `./Annotation_tool`
