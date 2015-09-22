#ifndef VIEWERINTERACTOR_H
#define VIEWERINTERACTOR_H

#include <pcl/visualization/common/common.h>
#include <vtkRenderWindow.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pointcloudmodifier.h"
#include "objectsinformation.h"

#include <eigen3/Eigen/Eigen>

class viewerInteractor
{
public:
    viewerInteractor();


    void getPose();


    /** \brief Register the point picking callback
      */
    void registerCallback();

    /** \brief Get the point picked by the user
      * \param[out] point The point picked
      */
    void getPointPicked(pcl::PointXYZRGB *point);

    /** \brief Get the desired number of points picked
      * \param[in] nPoints Number of points
      * \param[out] pointsPicked The vector with the picked points
      */
    void getPointsPicked(int nPoints, std::vector<pointT> *pointsPicked);

    /** \brief Clean the visualizer, removing all shapes and point clouds but the main point cloud
      */
    void cleanViewer();

    /** \brief Clean all the visualizer including the point cloud loaded before
      */
    void cleanAll();

    /** \brief Add the point cloud to the viewer to visualize
      * \param[in] cloud The point cloud to be visualized
      */
    void visualizePointCloud(pointCloudPtr cloud);

    /** \brief Highlight the points with the color given for the rgb value
      * \param[in] cloud The point cloud
      * \param[in] r The red value of the color
      * \param[in] g The green value of the color
      * \param[in] b The blue value of the color
      * \param[in] indices The indices of the points to highlight
      */
    void highligthPoints(pcl::PointCloud<pointT>::Ptr cloud, int r, int g, int b, pcl::PointIndices indices);

    /** \brief higlight the points inside the current bounding box
      * \param[in] cloud The point cloud
      */
    void highligthPointsinBox(pointCloudPtr cloud);

    /** \brief higlight all the objects in a object list
      * \param[in] cloud The point cloud
      * \param[in] objectList List with all the objects
      */
    void highligthAllObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                             std::vector<object> &objectList);

    /** \brief Remove all the clouds drawn
      */
    void removeDrawnClouds();

    /** \brief Remove the drawn bounding box
      */
    void removeBoundingBox();

    /** \brief Add coordinate system to screen
      */
    void addCoordinateSystem();

    /** \brief Remove coordinate system to screen
      */
    void removeCoordinateSystem();

    // The RGB values for all the colors used. If some change is made, change the name of the colors
    // in the initialization of the class objectsInformation
    // The colors are red, green, blue, yellow, pink, turquoise, orange, purple,
    // dark green, beige, brown, gold, salmon, grey, blueViolet
    std::vector<int> colorRedValue;
    std::vector<int> colorGreenValue;
    std::vector<int> colorBlueValue;

    /** \brief Define the initial bounding box used
      * \param[in] objectPose The xyz coordinates of the pose
      * \param[in] roll The roll angle of the bounding box
      * \param[in] pitch The pitch angle of the bounding box
      * \param[in] yaw The yaw angle of the bounding box
      * \param[in] length The length of the bounding box
      * \param[in] width The width of the bounding box
      * \param[in] height The height of the bounding box
      */
    void defineBoundingBox(pcl::PointXYZ objectPose,
                           float roll,
                           float pitch,
                           float yaw,
                           float length,
                           float width,
                           float height);

    /** \brief Draws the current bounding box
      */
    void drawBoundingBox();

    /** \brief Get the points inside the current bounding box
      * \param[in] cloud The point cloud used
      * \param[out] indices The indices inside the bounding box
      */
    void getPointsInBoundingBox(pointCloudPtr cloud, pcl::PointIndices *indices);

    /** \brief Redefine the xyz pose coordinates
      * \param[in] objectPose Pose of the object(xyz)
      */
    void redefinePose(pcl::PointXYZ objectPose);

    /** \brief Redefine the rotation
      * \param[in] roll Roll angle
      * \param[in] pitch Pitch angle
      * \param[in] yaw Yaw angle
      */
    void redefineRotation(float roll, float pitch, float yaw);

    /** \brief Redefine the length of the bounding box
      * \param[in] length New length value of the bounding box
      */
    void redefineLength(float length);

    /** \brief Redefine the width of the bounding box
      * \param[in] width New width value of the bounding box
      */
    void redefineWidth(float width);

    /** \brief Redefine the height of the bounding box
      * \param[in] height New height value of the bounding box
      */
    void redefineHeight(float height);


    // Falta per comentar /////
    // ++++++++++++++++++ ///
    void setCameraPose(double posX, double posY, double posZ,
                       double viewX, double viewY, double viewZ,
                       double upX, double upY, double upZ);

    /** \brief Returns the pointer of the render window
      * \param[in] width Width of the window
      * \param[in] height Height of the window
      */
    vtkSmartPointer<vtkRenderWindow> getRenderWindow(int width, int height);

    /** \brief Set up the interactor
      * \param[in] iren The interactor to set up
      * \param[in] win The render window
      */
    void setupInteractor(vtkRenderWindowInteractor *iren,
                         vtkRenderWindow *win){
        _viewer->setupInteractor(iren, win);
    }

    /** \brief Upload the image
      */
    void render(){
        _viewer->getRenderWindow()->Render();
    }

    Eigen::Affine3f getCameraParametersAndPose(std::vector<pcl::visualization::Camera>& cameras);

    void undo_points();

private:
    // PCL visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;

    // List of current point cloud drawn
    std::vector<std::string> _listClouds;

    // Number of spheres drawn
    double _numberOfSpheres;

    // Bounding box parameters
    pcl::PointXYZ _pose;
    float _angleRoll, _anglePitch, _angleYaw;
    float _boxLength, _boxWidth, _boxHeight;

    // Current box transformation
    Eigen::Affine3f _transformation;
    Eigen::Affine3f _viewPose;

    // Bounding box as a point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingBox;

    /** \brief Actualize the box transformation
      */
    void actualizeTransformation();

    bool points_picked;
    bool first_pick;
    int npointsToBepicked;


};

#endif // VIEWERINTERACTOR_H
