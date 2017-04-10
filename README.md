The 3D Annotation Tool is an application designed to manually annotate objects in a point
cloud 3D image. Initially, it was developed with the aim to annotate the objects on a desktop. 
However, the tool can be used to annotate any objects in 3D standing on a supporting plane.

This is the tool that was used to annotate the KTH-3D-Total dataset - https://strands.pdc.kth.se/public/kth-3d-total/readme.html

KTH-3D-TOTAL: A 3D dataset for discovering spatial structures for long-term autonomous learning

Thippur, Akshaya and Ambrus, Rares and Agrawal, Gaurav and Del Burgo, Adria Gallart and Ramesh, 
Janardhan Haryadi and Jha, Mayank Kumar and Akhil, Malepati Bala Siva Sai and Shetty, 
Nishan Bhavanishankar and Folkesson, John and Jensfelt, Patric
13th International Conference of Control Automation Robotics & Vision (ICARCV), 2014 

The format of the point cloud supported is .pcd. 

3D Annotation Tool has been developed by Adria Gallart del Burgo at KTH in 2013 and modified and maintained by 
Akshaya Thippur since then.

The tool can perform the following:
- Plane detection 
- Plane segmentation
- Object annotation
- Data file generation (XML)

The objects annotated are stored together per scene in their XML file. They record the the following for each annotated object:
- position
- pose
- minimum oriented bounding box
- comprising pixel indices in 3D
- length, breadth, height 


The tool also allows for loading a previous annotation and modifying it.
