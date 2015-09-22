#include "viewerinteractor.h"
#include "mainwindow.h"

#include <qcoreapplication.h>

#include <pcl/common/angles.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>


#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Eigen>
#include <QMessageBox>

#define PI 3.14159265

viewerInteractor::viewerInteractor():
    boundingBox (new pcl::PointCloud<pcl::PointXYZ>),
    _viewer (new pcl::visualization::PCLVisualizer ("Viewer", false))
{
    // The color order is: red, green, blue, yellow, pink, turquoise, orange, purple,
    // dark green, beige, brown, gold, salmon, grey, blueViolet
    int cRed [30] = {  255,   0,   0, 255, 255,   0, 255, 128,   0, 255, 165, 139, 198, 84, 138,      255,   0,   0, 255, 255,   0, 255, 128,   0, 255, 165, 139, 198, 84, 138};
    int cGreen [30] = {  0, 255,   0, 255,   0, 255, 160,  0, 128, 222, 42, 117, 113, 84, 43,          0, 255,   0, 255,   0, 255, 160,  0, 128, 222, 42, 117, 113, 84, 43};
    int cBlue [30] = {   0,   0, 255,   0, 255, 255, 0  , 128,   0, 173, 42,  0, 113, 84, 226,         0,   0, 255,   0, 255, 255, 0  , 128,   0, 173, 42,  0, 113, 84, 226};

    for(int i=0; i<30; i++){
        colorRedValue.push_back(cRed[i]);
        colorGreenValue.push_back(cGreen[i]);
        colorBlueValue.push_back(cBlue[i]);
    }

    boundingBox->width  = 8;
    boundingBox->height = 1;
    boundingBox->points.resize (boundingBox->width * boundingBox->height);
    points_picked = false;
    _numberOfSpheres = 0;
}

int nPoints = 0;
bool picked(false);
pcl::PointXYZRGB pickedPoints;


std::vector<pointT> *temp_pointsPicked;

// Callback function
void pp_callbacks (const pcl::visualization::PointPickingEvent& event, void* viewer_void){
    if (event.getPointIndex () == -1)
        return;

    // Get the point picked by the user
    event.getPoint(pickedPoints.x, pickedPoints.y, pickedPoints.z);
    nPoints++;
    picked=true;
}


void viewerInteractor::registerCallback(){
    _viewer->registerPointPickingCallback(pp_callbacks, (void*)&_viewer);
}

void viewerInteractor::getPointPicked(pcl::PointXYZRGB *point){
    picked = false;

    while(!picked){
        QCoreApplication::processEvents();
    }
    point->x=pickedPoints.x;
    point->y=pickedPoints.y;
    point->z=pickedPoints.z;
    picked = false;

    // Draw an sphere arround the selected point
    QString sphere = QString::number(_numberOfSpheres);
    _viewer->addSphere(*point, 0.009, 1, 1, 0.0, sphere.toStdString());

}

void viewerInteractor::getPointsPicked(int nPoints, std::vector<pointT> *pointsPicked){
   /* if (nPoints == 4){
        //point 1
        pcl::PointXYZRGB point;
        getPointPicked(&point);
        pointsPicked->push_back(point);
        //point 2
        pcl::PointXYZRGB point;
        getPointPicked(&point);
        pointsPicked->push_back(point);
        //Draw line

        _viewer->addLine(pointsPicked[0],pointsPicked[1],id = "top");

        //point 3
        pcl::PointXYZRGB point;
        getPointPicked(&point);
        pointsPicked->push_back(point);

        //point 4
        pcl::PointXYZRGB point;
        getPointPicked(&point);
        pointsPicked->push_back(point);

    }*/

    npointsToBepicked = nPoints;
    points_picked = true;
    first_pick = true;
    int i = 0;

    while(i < nPoints){
        pcl::PointXYZRGB point;
        getPointPicked(&point);

        _numberOfSpheres++;
        npointsToBepicked--;

        i = nPoints - npointsToBepicked;

        if (first_pick == true){
            pointsPicked->push_back(point);
            temp_pointsPicked =  pointsPicked;
            first_pick = false;
        }
        else{
            pointsPicked = temp_pointsPicked;
            pointsPicked->push_back(point);
            temp_pointsPicked =  pointsPicked;
        }

        if (i == nPoints){
            int r = QMessageBox::question(NULL, "Confirm point selection",
                                                "Click 'Yes'' to confirm selected points. And 'No' to Undo last point and continue selection.",
                                                 QMessageBox::Yes | QMessageBox::No);
            if (r == QMessageBox::No){
                undo_points();
                i = nPoints - npointsToBepicked;

            }
        }

    }
    points_picked = false;
}

void viewerInteractor::cleanViewer(){
    removeDrawnClouds();
    removeBoundingBox();
    render();
}

void viewerInteractor::cleanAll(){
    _viewer->removeAllPointClouds();
    _viewer->resetCamera();
    _viewer->removeAllShapes();
    _viewer->removeCoordinateSystem();
    render();
}

void viewerInteractor::visualizePointCloud(pointCloudPtr cloud){
    pcl::visualization::PointCloudColorHandlerRGBField<pointT> rgb(cloud);
    _viewer->addPointCloud<pointT>(cloud,rgb);
    _viewer->setBackgroundColor(0, 0, 0);
    render();
}

void viewerInteractor::highligthPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                       int r,
                                       int g,
                                       int b,
                                       pcl::PointIndices indices){

    removeDrawnClouds();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudObject (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, indices, *cloudObject);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloudObject, r, g, b);

    _listClouds.push_back("colored");
    _viewer->addPointCloud<pcl::PointXYZ> (cloudObject, single_color, _listClouds[0]);
}

void viewerInteractor::highligthPointsinBox(pointCloudPtr cloud){
    pcl::PointIndices indices;
    getPointsInBoundingBox(cloud, &indices);
    highligthPoints(cloud, 255, 0, 0, indices);
    render();
}

void viewerInteractor::highligthAllObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                           std::vector<object> &objectList){
    removeDrawnClouds();

    for(int i=0; i < objectList.size(); i++){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, objectList[i].indices, *cloudObject);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloudObject,
                                                                                     colorRedValue.at(i),
                                                                                     colorGreenValue.at(i),
                                                                                     colorBlueValue.at(i));
        std::stringstream name; name << i;

        _listClouds.push_back(name.str());
        _viewer->addPointCloud<pcl::PointXYZ> (cloudObject, single_color, _listClouds[i]);
    }
}

void viewerInteractor::removeDrawnClouds(){
    if(_listClouds.size()!=0){
        for(int i = 0; i < _listClouds.size(); i++){
            _viewer->removePointCloud(_listClouds[i]);
        }
        _listClouds.clear();
    }
}

void viewerInteractor::removeCoordinateSystem(){
    _viewer->removeCoordinateSystem();
    render();
}

void viewerInteractor::addCoordinateSystem(){
    _viewer->addCoordinateSystem(0.5);
    render();
}

void viewerInteractor::defineBoundingBox(pcl::PointXYZ objectPose,
                                         float roll,
                                         float pitch,
                                         float yaw,
                                         float length,
                                         float width,
                                         float height){
    _pose = objectPose;
    _boxLength = length;
    _boxWidth = width;
    _boxHeight = height;
    _angleRoll = roll;
    _anglePitch = pitch;
    _angleYaw = yaw;

    // Fill in the cloud data
    // First point is the pose
    boundingBox->points[0].x = 0.0; boundingBox->points[0].y = 0.0; boundingBox->points[0].z = 0.0;


    // Calculate the other three points in the front face
    boundingBox->points[1].x = length; boundingBox->points[1].y = 0.0; boundingBox->points[1].z = 0.0;
    boundingBox->points[2].x = length; boundingBox->points[2].y = 0.0; boundingBox->points[2].z = height;
    boundingBox->points[3].x = 0.0; boundingBox->points[3].y = 0.0;  boundingBox->points[3].z = height;


    // Calculate the four points in the back face
    boundingBox->points[4].x = 0.0; boundingBox->points[4].y = width; boundingBox->points[4].z = 0.0;
    boundingBox->points[5].x = length; boundingBox->points[5].y = width; boundingBox->points[5].z = 0.0;
    boundingBox->points[6].x = length; boundingBox->points[6].y = width; boundingBox->points[6].z = height;
    boundingBox->points[7].x = 0.0; boundingBox->points[7].y = width; boundingBox->points[7].z = height;
}

void viewerInteractor::actualizeTransformation(){
    _transformation = pcl::getTransformation (_pose.x,
                                              _pose.y,
                                              _pose.z,
                                              _angleRoll,
                                              _anglePitch,
                                              _angleYaw);
}

void viewerInteractor::drawBoundingBox(){
    // Before drawing, remove the previous lines
    _viewer->removeAllShapes();

    // Auxiliar bounding box to apply the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr boxAux (new pcl::PointCloud<pcl::PointXYZ>);
    actualizeTransformation();

    // Transformation of the point cloud
    // *** takes a lot of time *****
    pcl::transformPointCloud(
                *boundingBox,
                *boxAux,
                _transformation);

    // Draw front face
    _viewer->addLine(boxAux->points[0], boxAux->points[1], 255, 0, 0, "0_1");
    _viewer->addLine(boxAux->points[0], boxAux->points[3], 255, 0, 0, "0_3");
    _viewer->addLine(boxAux->points[1], boxAux->points[2], 255, 0, 0, "1_2");
    _viewer->addLine(boxAux->points[2], boxAux->points[3], 255, 0, 0, "2_3");

    // Draw back face
    _viewer->addLine(boxAux->points[4], boxAux->points[5], 255, 255, 0, "4_5");
    _viewer->addLine(boxAux->points[4], boxAux->points[7], 255, 255, 0, "4_7");
    _viewer->addLine(boxAux->points[5], boxAux->points[6], 255, 255, 0, "5_6");
    _viewer->addLine(boxAux->points[6], boxAux->points[7], 255, 255, 0, "6_7");

    // Draw lines between faces
    _viewer->addLine(boxAux->points[0],boxAux->points[4], 255, 255, 0, "0_4");
    _viewer->addLine(boxAux->points[1], boxAux->points[5], 255, 255, 0, "1_5");
    _viewer->addLine(boxAux->points[2],boxAux->points[6], 255, 255, 0, "2_6");
    _viewer->addLine(boxAux->points[3], boxAux->points[7], 255, 255, 0, "3_7");
}

void viewerInteractor::getPointsInBoundingBox(pointCloudPtr cloud, pcl::PointIndices *indices){
    if(true){
        int nPoints = 0;
        pointCloudPtr cloudAux (new pointCloud);
        cloudAux->width = cloud->size();
        cloudAux->height = 1;
        cloudAux->resize(cloud->size());

        actualizeTransformation();

        Eigen::Affine3f inverse = _transformation.inverse();
        pcl::transformPointCloud(
                    *cloud,
                    *cloudAux,
                    inverse);

        for(size_t index = 0; index < cloud->points.size(); index++){
            pcl::PointXYZ pointToCheck;
            pointToCheck.x = cloudAux->points[index].x;
            pointToCheck.y = cloudAux->points[index].y;
            pointToCheck.z = cloudAux->points[index].z;
            if (pointToCheck.x > 0 && pointToCheck.y > 0 && pointToCheck.z > 0
                    && pointToCheck.x < _boxLength && pointToCheck.y < _boxWidth && pointToCheck.z < _boxHeight){
                indices->indices.push_back(index);
                nPoints++;
            }
        }
    }
}


void viewerInteractor::redefinePose(pcl::PointXYZ objectPose){
    _pose = objectPose;
}

void viewerInteractor::redefineRotation(float roll, float pitch, float yaw){
    _angleRoll = roll;
    _anglePitch = pitch;
    _angleYaw = yaw;
}

void viewerInteractor::redefineLength(float length){
    boundingBox->points[1].x = length;
    boundingBox->points[2].x = length;
    boundingBox->points[5].x = length;
    boundingBox->points[6].x = length;
    _boxLength = length;
}

void viewerInteractor::redefineWidth(float width){
    boundingBox->points[4].y = width;
    boundingBox->points[5].y = width;
    boundingBox->points[6].y = width;
    boundingBox->points[7].y = width;
    _boxWidth = width;
}

void viewerInteractor::redefineHeight(float height){
    boundingBox->points[2].z = height;
    boundingBox->points[3].z = height;
    boundingBox->points[6].z = height;
    boundingBox->points[7].z = height;
    _boxHeight = height;
}


void viewerInteractor::removeBoundingBox(){
    _viewer->removeAllShapes();
}

void viewerInteractor::setCameraPose(double posX, double posY, double posZ,
                                     double viewX, double viewY, double viewZ,
                                     double upX, double upY, double upZ){

    _viewer->setCameraPosition(posX, posY, posZ, viewX, viewY, viewZ, upX, upY, upZ);
    render();
}

vtkSmartPointer<vtkRenderWindow> viewerInteractor::getRenderWindow(int width, int height){
    vtkSmartPointer<vtkRenderWindow> renderWin = _viewer->getRenderWindow();
    renderWin->SetSize(width, height);
    return renderWin;
}

Eigen::Affine3f viewerInteractor::getCameraParametersAndPose(std::vector<pcl::visualization::Camera>& cameras){
    Eigen::Affine3f viewPose;
    _viewPose = _viewer->getViewerPose();
    _viewer->getCameras(cameras);
    return viewPose;

}

void viewerInteractor::getPose(){
     _viewPose = _viewer->getViewerPose();
}

void viewerInteractor::undo_points()
{
    if (points_picked == true)
    {
        _numberOfSpheres--;
        QString sp_id = QString::number(_numberOfSpheres);
        _viewer->removeShape(sp_id.toStdString());


        QMessageBox::information(NULL,
                                 "Undo point",
                                 "Please re-select a point.");

       temp_pointsPicked->pop_back();
       npointsToBepicked++;

    }
}
