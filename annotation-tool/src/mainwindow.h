#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTreeWidgetItem>

#include <QCloseEvent>

#include "objectsinformation.h"
#include "pointcloudmodifier.h"
#include "viewerinteractor.h"
#include "objectsinformation.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <eigen3/Eigen/Eigen>



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

    void showInitialMessage();


// All the slots to interact with the application
// More details about them in the .cpp file

private Q_SLOTS:

    void on_actionOpen_triggered();

    void on_actionExit_triggered();

    void on_actionSave_PCD_File_triggered();

    void on_treeWidget_itemSelectionChanged();

    void on_actionAutomatic_plane_detection_triggered();

    void on_actionRotate_z_180_triggered();

    void on_actionManual_plane_definition_triggered();

    void on_actionDesk_segmentation_triggered();

    void on_actionInsert_new_object_triggered();

    void on_actionConfirm_position_triggered();

    void on_actionDelete_object_triggered();

    void on_actionExport_objects_info_triggered();

    void on_actionImport_objects_info_triggered();

    void on_actionHelp_triggered();

    void on_buttonPitchMore_clicked();

    void on_buttonPitchLess_clicked();

    void on_buttonRollMore_clicked();

    void on_buttonRollLess_clicked();

    void on_actionCoordinate_system_toggled(bool arg1);

    void on_actionUp_triggered();

    void on_actionFront_triggered();

    void on_actionLeft_triggered();

    void on_actionRight_triggered();

    void on_actionBack_triggered();

    void on_poseInfo_itemChanged(QTreeWidgetItem *item, int column);

    void on_buttonZMore_clicked();

    void on_buttonYMore_clicked();

    void on_buttonXMore_clicked();

    void on_buttonZLess_clicked();

    void on_buttonYLess_clicked();

    void on_buttonXLess_clicked();

    void on_buttonYawMore_clicked();

    void on_buttonYawLess_clicked();

    void on_boxHeightMore_clicked();

    void on_boxWidthMore_clicked();

    void on_boxLengthMore_clicked();

    void on_boxHeightLess_clicked();

    void on_boxWidthLess_clicked();

    void on_boxLengthLess_clicked();

    void on_boxLength_editingFinished();

    void on_boxWidth_editingFinished();

    void on_boxHeight_editingFinished();

    void on_actionShow_info_messages_toggled(bool arg1);

    void on_actionUndo_triggered();

    void on_actionDownsample_point_cloud_triggered();

    void save_Option(int type);

    void on_actionQSR_values_triggered();

    void on_actionDescription_of_scene_using_QSR_triggered();

    void on_actionSave_QSR_in_txt_file_triggered();

    void on_actionSaveAs_PCD_xml_triggered();

    void on_actionShiftorigin_triggered();

    void on_actionUndoPoints_triggered();

private:
    // The main window used
    Ui::MainWindow *_ui;

    // Used to store the pointcloud
    pcl::PointCloud<pointT>::Ptr _cloud;

    // Used to store the previous point cloud after some modifications
    pcl::PointCloud<pointT>::Ptr _cloudUndo;

    // The name of the current object to annotate
    QString _objectName;

    // The pose of the object bounding box
    pcl::PointXYZ _objectPose;

    // The size and orientation of the bounding box
    float _boxLength, _boxWidth, _boxHeight;
    float _boxRoll, _boxPitch, _boxYaw;

    // Class to modify the point cloud
    pointCloudModifier cloudModifier;

    // Class to interact with the visualizer
    viewerInteractor viewInteractor;

    // Class to edit the information of the annotated objects
    objectsInformation objectsInfo;

    // Name of the .pcd file load and the last directory used
    QString _fileName;
    QString _lastDir;

    // Scenario
    QString _scenario;

    // Camera and view point
    pcl::visualization::Camera _camera;
    Eigen::Affine3f _viewPose;


    // Some bool variables
    bool _pcdLoaded, _pcdLoadError, _planeDefined, _planeSegmentated;
    bool _insertingObject, _objectModifed;
    bool _itemSelected, _cloudModified;
    bool _showInfoMsgs, _showInitialMsg;
    bool _firstsave;

    /** \brief Function to initialize
      */
    void init();

    /** \brief Open a point cloud
      */
    void open_pcd_file();

    /** \brief Load the point cloud file
      */
    void load_pcd_file(QString fileName);

    /** \brief Asks to the user if wants to exit
      */
    bool okToExit();

    /** \brief Visualizes the point cloud
      */
    void visualize();

    /** \brief Actualizes the pose information widget
      */
    void actualizePoseInfo();

    /** \brief Clear the pose information displayed
      */
    void clearPoseInfo();

    /** \brief Draws the bounding box, its points insed and show the pose information
      */
    void drawBoxPointsAndNewInfo();

    /** \brief Set the pose information to be displayed
      \param[in] boxPose Coordinate x,y,z of the bounding box
      \param[in] boxRoll Roll angle of the bounding box
      \param[in] boxPitch Pitch angle of the bounding box
      \param[in] boxYaw Yaw angle of the bounding box
      */
    void setPoseInfo(pcl::PointXYZ boxPose, float boxRoll, float boxPitch, float boxYaw);

    /** \brief Actualize the pose of the current object
      */
    void actualizePose();

    /** \brief  Actualize the rotation of the current object
      */
    void actualizeRotation();

    /** \brief Actualize the width of the bounding box of the current object
      */
    void actualizeBoxWidth();

    /** \brief Actualize the length of the bounding box of the current object
      */
    void actualizeBoxLength();

    /** \brief Actualize the height of the bounding box of the current object
      */
    void actualizeBoxHeight();

    /** \brief  Draw the bounding box and the points inside
      */
    void drawBoxAndPoints();

    /** \brief  Actualize the data in the information tree widget
      */
    void actualizeInformationTreeWidget();

    /** \brief Displays the desk's length in the information tree widget
      */
    void displayDeskLengthInfo();

    /** \brief Displays the desk's width in the information tree widget
      */
    void displayDeskWidthInfo();

    /** \brief  Display the objects' information in the information tree widget
      */
    void displayObjectsInfo();

    /** \brief Clear all the information tree widget
      */
    void clearInfoTreeWidget();

    /** \brief Confirm the object position
      */
    void confirmObjectPosition();

    void closeEvent(QCloseEvent *);

};

#endif // MAINWINDOW_H
