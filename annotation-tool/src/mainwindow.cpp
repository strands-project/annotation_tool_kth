#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "newobjectdialog.h"
#include "selectobjectdialog.h"
#include "addobjectdialog.h"
#include "initialmessagedialog.h"
#include "objectsinformation.h"
#include "filtervaluesdialog.h"
#include "newobjectdialog.h"
#include "qsr.h"

#include "QMessageBox"
#include "QCloseEvent"
#include "QFileDialog"
#include "QDesktopServices"
#include "QUrl"
#include "QListWidgetItem"

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>

#include <math.h>
#include <eigen3/Eigen/Dense>

#include <iostream>
#include <fstream>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem/path.hpp>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    _ui(new Ui::MainWindow),
    _cloud (new pcl::PointCloud<pointT>),
    _cloudUndo (new pcl::PointCloud<pointT>)
{
    init();
}

MainWindow::~MainWindow(){
    delete _ui;
}

///////////////////////////////////////////////////////////////
// The following functions are all the slots' functions used //
// when the user interact with the application               //
///////////////////////////////////////////////////////////////

// Open button action
void MainWindow::on_actionOpen_triggered(){
    // Open the pcd file
    open_pcd_file();

    //////////++++++++++++///////////////////
    // Crear xml parser per fer-ho ///////////
    //////++++++++++/////////////////
    // Save the current directory as a historial
    if(!_fileName.isEmpty()){
        QString info_doc = qApp->applicationDirPath();
        info_doc.remove("bin");
        info_doc.append("/info_app.xml");


        boost::filesystem::path p(_fileName.toStdString());
        std::string path = p.parent_path().string();
        _lastDir = QString::fromStdString(path);

        boost::property_tree::ptree root;
        read_xml(info_doc.toStdString(), root);

        bool showInitial = root.get<bool>("showInitial");

        root.clear();
        root.put("showInitial", showInitial);
        root.put("lastDirectory", path);
        boost::property_tree::xml_writer_settings<char> settings(' ', 3);
        write_xml(info_doc.toStdString(), root, std::locale(), settings);
    }
}

// Exit button action
void MainWindow::on_actionExit_triggered(){
    if(_cloudModified){
        int r = QMessageBox::warning(this,
                                     tr("Changes not saved"),
                                     tr("The point cloud has been changed. Do you want to save the changes?"),
                                     QMessageBox::Yes , QMessageBox::No);
        if (r == QMessageBox::Yes) {
            on_actionSave_PCD_File_triggered();
            //if(!_cloudModified) MainWindow::close();
        }
        else
            MainWindow::close();
    }
    else{
        MainWindow::close();
        }
    }

// Next 6 functions: Slots for the buttons to
// modify the x,y,z coordinate of the pose
void MainWindow::on_buttonXMore_clicked(){
    _objectPose.x += 0.01;
    actualizePose();
}

void MainWindow::on_buttonXLess_clicked(){
    _objectPose.x -= 0.01;
    actualizePose();
}

void MainWindow::on_buttonYMore_clicked(){
    _objectPose.y += 0.01;
    actualizePose();
}

void MainWindow::on_buttonYLess_clicked(){
    _objectPose.y -= 0.01;
    actualizePose();
}

void MainWindow::on_buttonZMore_clicked(){
    _objectPose.z += 0.01;
    actualizePose();
}

void MainWindow::on_buttonZLess_clicked(){
    _objectPose.z -= 0.01;
    actualizePose();
}

// Next 6 functions: Slots for the buttons to modify
// the roll, pitch, yaw angles of the pose
void MainWindow::on_buttonYawMore_clicked(){
    _boxYaw += M_PI/180;
    if (_boxYaw > 2*M_PI) _boxYaw = 0;
    actualizeRotation();
}

void MainWindow::on_buttonYawLess_clicked(){
    _boxYaw -= M_PI/180;
    if (_boxYaw < 0) _boxYaw = 2*M_PI;
    actualizeRotation();
}

void MainWindow::on_buttonPitchMore_clicked(){
    _boxPitch += M_PI/180;
    if (_boxPitch > 2*M_PI) _boxPitch = 0;
    actualizeRotation();
}

void MainWindow::on_buttonPitchLess_clicked(){
    _boxPitch -= M_PI/180;
    if (_boxPitch < 0) _boxPitch = 2*M_PI;
    actualizeRotation();
}

void MainWindow::on_buttonRollMore_clicked(){
    _boxRoll += M_PI/180;
    if (_boxRoll > 2*M_PI) _boxRoll = 0;
    actualizeRotation();
}

void MainWindow::on_buttonRollLess_clicked(){
    _boxRoll -= M_PI/180;
    if (_boxRoll < 0) _boxRoll = 2*M_PI;
    actualizeRotation();
}


// Next 9 functions: Slots to modify the width, length
// and height value of the bounding box
void MainWindow::on_boxWidth_editingFinished(){
    if(_insertingObject){
        _boxWidth = _ui->boxWidth->value()/100;
        actualizeBoxWidth();
    }

    if(!_itemSelected && !_insertingObject){
        _ui->boxWidth->setValue(0);
    }
}

void MainWindow::on_boxWidthMore_clicked(){
    if(_insertingObject){
        _boxWidth += 0.01;
        _ui->boxWidth->setValue(_boxWidth*100);
        actualizeBoxWidth();
    }
}

void MainWindow::on_boxWidthLess_clicked(){
    if(_insertingObject){
        _boxWidth -= 0.01;
        _ui->boxWidth->setValue(_boxWidth*100);
        actualizeBoxWidth();
    }
}

void MainWindow::on_boxLength_editingFinished(){
    if(_insertingObject){
    _boxLength = _ui->boxLength->value()/100;
       actualizeBoxLength();
    }

    if(!_itemSelected && !_insertingObject){
        _ui->boxLength->setValue(0);
    }
}

void MainWindow::on_boxLengthMore_clicked(){
    if(_insertingObject){
        _boxLength += 0.01;
        _ui->boxLength->setValue(_boxLength*100);
        actualizeBoxLength();
    }
}

void MainWindow::on_boxLengthLess_clicked(){
    if(_insertingObject){
        _boxLength -= 0.01;
        _ui->boxLength->setValue(_boxLength*100);
        actualizeBoxLength();
    }
}

void MainWindow::on_boxHeight_editingFinished(){
    if(_insertingObject){
        _boxHeight = _ui->boxHeight->value()/100;
        actualizeBoxHeight();
    }

    if(!_itemSelected && !_insertingObject){
        _ui->boxHeight->setValue(0);
    }
}


void MainWindow::on_boxHeightMore_clicked(){
     if(_insertingObject){
         _boxHeight += 0.01;
         _ui->boxHeight->setValue(_boxHeight*100);
         actualizeBoxHeight();
     }
}

void MainWindow::on_boxHeightLess_clicked(){
    if(_insertingObject){
        _boxHeight -= 0.01;
        _ui->boxHeight->setValue(_boxHeight*100);
        actualizeBoxHeight();
    }
}

// Save pcd file action
void MainWindow::on_actionSave_PCD_File_triggered(){

    if(_pcdLoaded){

    if (_firstsave){
        save_Option(1); // Save
    }
    else
    {
            // Save the .pcd file

            QString fileNamePCD = _fileName;

            std::string fileName = _fileName.toStdString();
            fileName.erase(fileName.find_last_of(".")+1, 3);
            fileName.append("xml");

            QString fileNameXML =  QString::fromStdString(fileName);

            pcl::io::savePCDFileBinary(fileNamePCD.toStdString(), *_cloud);
            _cloudModified = false;

            // Save the objects' information
            if(objectsInfo.getDeskLength() != -1){
                objectsInfo.exportObjectsInformation(fileNameXML, _fileName, _scenario);
            }
            else
                QMessageBox::warning(this,
                                     "XML file not saved",
                                     "The objects' information has not been saved because there is no information.");

        }
    }
    else
        QMessageBox::warning(this,
                             "No PCD file opened",
                             "Open a PCD file and try again..");
}


// Actions to take when an item in the information tree
// widget is selected
void MainWindow::on_treeWidget_itemSelectionChanged(){
    // If the object has been changed without confirm
    // ask if the user want to lose the changes
    if(_insertingObject && _objectModifed){
        int r = QMessageBox::warning(this,
                                     tr("Changes not confirmed"),
                                     tr("You have changed the object without confirm. Do you want to save the changes introduced in the object?"),
                                     QMessageBox::Yes , QMessageBox::No);
        if (r == QMessageBox::Yes) {
            _boxLength = _ui->boxLength->value()/100;
            _boxWidth = _ui->boxWidth->value()/100;
            _boxHeight = _ui->boxHeight->value()/100;
            confirmObjectPosition();
        }
    }

    _insertingObject = false;
    _itemSelected = false;

    QString text = _ui->treeWidget->currentItem()->text(0);

    // If objects is selected, all the objects are shown
    if(text.toStdString() == "Objects"){
        _ui->treeWidget->currentItem()->setExpanded(true);
        viewInteractor.removeBoundingBox();
        std::vector<object> objectList = objectsInfo.getObjectList();
        viewInteractor.highligthAllObjects(_cloud, objectList);
        viewInteractor.render();
        clearPoseInfo();
    }
    // If only one object is selected it is shown with the bounding box
    else if (objectsInfo.getIndex(text) != -1){
        _itemSelected = true;
        // Get the information about the current object selected
        int index = objectsInfo.getIndex(text);
        pcl::PointIndices indices;
        pcl::PointXYZ pose;
        float roll, pitch, yaw, boxLength, boxWidth, boxHeight;
        objectsInfo.getGeometry(index, pose, roll, pitch, yaw, boxLength, boxWidth, boxHeight);
        viewInteractor.defineBoundingBox(pose, roll, pitch, yaw, boxLength, boxWidth, boxHeight);
        objectsInfo.getIndices(index, &indices);

        // Draw the box and highlight the points inside
        drawBoxAndPoints();

        // Actualize the information displayed
        _ui->boxLength->setValue(boxLength*100);
        _ui->boxWidth->setValue(boxWidth*100);
        _ui->boxHeight->setValue(boxHeight*100);
        setPoseInfo(pose, roll, pitch, yaw);

        // Prepare to be modify while is selected
        _objectName = objectsInfo.nameOfObject(index);
        _objectPose = pose;
        _boxRoll = roll;
        _boxPitch = pitch;
        _boxYaw = yaw;
        _insertingObject = true;
        _objectModifed = false;
    }
    else if(text.toStdString() == _scenario.toStdString()){
        _ui->treeWidget->currentItem()->setExpanded(true);
    }


    else {
        // Don't shown any object
        viewInteractor.cleanViewer();
        //clearPoseInfo();
    }
}

// Automatic plane detection
void MainWindow::on_actionAutomatic_plane_detection_triggered(){
    if(_pcdLoaded){

        viewInteractor.getPose();

        // Automatic detection of the desk plane and move to the plane x-y
        cloudModifier.automaticTableDetection(_cloud, _cloud);

        visualize();
        _cloudModified = true;


        // If the automatic detection of the plane is not correct, it can be
        // calculated manually by picking three points.
        if(_showInfoMsgs) QMessageBox::information(this,
                                                   "Correct plane?",
                                                   "If the plane is not correct proceed to do the manual plane detection or apply z rotation.");
        _planeDefined=true;
    }
    else{
        QMessageBox::warning(this, "Error", "Firstly, you must load a pcd file.");
    }
}

// Rotate the point cloud 180º in the z axis
void MainWindow::on_actionRotate_z_180_triggered(){
    if(!_pcdLoaded)
    {
        QMessageBox::warning(this, "Error", "Firstly, you must load a pcd file.");
    }
    else{
        cloudModifier.rotate_z_180(_cloud, _cloud);
        visualize();
        _cloudModified = true;
    }
}

// Manual plane definition
void MainWindow::on_actionManual_plane_definition_triggered(){
    std::vector<pointT> tablePoints;

    //  First is checked if the pcd file is loaded.
    if(!_pcdLoaded)
    {
        QMessageBox::warning(this, "Error", "Firstly, you must load a pcd file.");
    }
    else{
        // First pick three points of the table plane
        if(_showInfoMsgs) QMessageBox::information(this,
                                                   "Pick points",
                                                   "Pick three points of the table with shift+left mouse button.");

        viewInteractor.getPointsPicked(3, &tablePoints);

        // Manual plane definition of the desk plane and move to the plane x-y
        cloudModifier.manualTableDetection(_cloud, _cloud, tablePoints);
        _planeDefined = true;
        _ui->actionDesk_segmentation->setEnabled(true);
        visualize();
        _cloudModified = true;
    }
}

// Desk segmentation
void MainWindow::on_actionDesk_segmentation_triggered(){
    if(!_pcdLoaded | !_planeDefined)
    {
        if(!_pcdLoaded)
        {
            QMessageBox::warning(this, "Error", "Firstly, you must load a pcd file.");
        }
        else
        {
            QMessageBox::warning(this, "Error", "Firstly, you must define the plane.");
        }
    }
    else{
        // Ask if it will be a desk, floor, etc
        chooseObjectDialog env(0,1);
        env.exec();
        _scenario = env.getObjectName();
        if(!_scenario.isEmpty()){
            QTreeWidgetItem *itm_scenario = new QTreeWidgetItem(_ui->treeWidget);
            itm_scenario->setText(0, _scenario);
            _ui->treeWidget->insertTopLevelItem(0,itm_scenario);

            // Insert the objects items
            QTreeWidgetItem *itm_objects = new QTreeWidgetItem(_ui->treeWidget);
            itm_objects->setText(0, QString::fromStdString("Objects"));
            _ui->treeWidget->insertTopLevelItem(0,itm_objects);

            pointT pointPicked;
            float table_length, table_width;

            // Copy the point cloud if the user want to undo the segmentation
            pcl::copyPointCloud(*_cloud, *_cloudUndo);

            // Segmentate the table given three points picked by the user

            // Pick the first point needed: lower left corner
            if(_showInfoMsgs) QMessageBox::information(this,
                                                       "Pick a point",
                                                       "Pick the lower left corner of the table with shift+left mouse button.");

            viewInteractor.getPointPicked(&pointPicked);
		
            // Move the pointcloud to the lower left corner of the table
            cloudModifier.translate_on_plane_x_y(_cloud, _cloud, pointPicked);
            visualize();
	
            // Pick the second point needed: lower right corner
            if(_showInfoMsgs) QMessageBox::information(this,
                                                       "Pick a point",
                                                       "Pick the lower right corner of the table with shift+left mouse button");
            viewInteractor.getPointPicked(&pointPicked);

            // Align the x axis with the lower edge of the table and calculate the table length
            cloudModifier.align_x_with_edge(_cloud, _cloud, pointPicked);
            pcl::PointXYZ origin(0,0,0);
            table_length = pcl::euclideanDistance(origin, pointPicked);
            visualize();

            // Pick the third point needed: one of the upper edge of the table
            if(_showInfoMsgs) QMessageBox::information(this,
                                                       "Pick a point",
                                                       "Pick a point on the upper edge of the table with shift+left mouse button");
            viewInteractor.getPointPicked(&pointPicked);


            // Eliminate points below the table

            // Min value of 0 remove points from the table -> set to -0.02
            //cloudModifier.filter_axis(_cloud, _cloud, "z", -0.02, 5);

            //Eliminate points at right and left side of the table
             //cloudModifier.filter_axis(_cloud, _cloud, "x", 0, table_length);

            //Eliminate points upper and lower side the table
            table_width = pointPicked.y;
            //cloudModifier.filter_axis(_cloud, _cloud, "y", 0, table_width);


            //Pass the table dimensions
            objectsInfo.setDeskDimensions(table_length, table_width);

            //Insert length information in the QtreeWidget
            displayDeskLengthInfo();

            //Insert width information in the QtreeWidget
            displayDeskWidthInfo();

            visualize();
            _cloudModified = true;
            _planeSegmentated = true;
            _ui->actionUndo->setEnabled(true);

            // Remind to save it
            if(_showInfoMsgs) QMessageBox::information(this,
                                                       "Save the progress",
                                                       "It is recommended to save now the point cloud and the plane information.");

            // Desactivate this action
            _ui->actionDesk_segmentation->setEnabled(false);
        }
    }
}

// Insert new annotation of an object
void MainWindow::on_actionInsert_new_object_triggered(){
    //  First is checked if the pcd file is loaded.
    if(!_pcdLoaded | !_planeSegmentated){

        if(!_pcdLoaded)
            QMessageBox::warning(this, "Error", "Firstly, you must load a pcd file.");
        else
            QMessageBox::warning(this, "Error", "Firstly, you must segmentate the plane.");
    }

    else{

        if(_insertingObject && _objectModifed){
            int r = QMessageBox::warning(this,
                                         tr("Changes not confirmed"),
                                         tr("You have changed the object without confirm. Do you want to save the changes introduced in the object?"),
                                         QMessageBox::Yes , QMessageBox::No);
            if (r == QMessageBox::Yes) {
                confirmObjectPosition();
            }

            else{

               viewInteractor.removeBoundingBox();
            }
        }

        // Select which object is desired to introduce
        AddObject addobj;
        addobj.exec();
        _objectName = addobj.getObjectName();

        if(!_objectName.isEmpty()){
            if(objectsInfo.existsObject(_objectName)){
                QMessageBox::warning(this, "Error", "This object already exists. Insert another name.");
            }
            else{
                _insertingObject = true;

                // Select the points for the initial object position
                if(_showInfoMsgs) QMessageBox::information(this,
                                         "Pick a point",
                                         "Pick the next 4 points of the object with shift+left mouse button:\n - Left corner\n - Right corner\n - Upper left corner\n - Highest part ");
                std::vector<pointT> clickedPoints;
                viewInteractor.getPointsPicked(4, &clickedPoints);

                // First guess is that the object is lying on the table
                _objectPose.x = clickedPoints[0].x;
                _objectPose.y = clickedPoints[0].y;
                _objectPose.z = 0.0;

                // To messure distance I am not using the z component of the picked points
                pcl::PointXYZ point1_x_y, point2_x_y, point3_x_y;
                point1_x_y.x = clickedPoints[0].x; point1_x_y.y = clickedPoints[0].y;
                point2_x_y.x = clickedPoints[1].x; point2_x_y.y = clickedPoints[1].y;
                point3_x_y.x = clickedPoints[2].x; point3_x_y.y = clickedPoints[2].y;

                // Object size
                _boxLength = pcl::euclideanDistance(point1_x_y, point2_x_y);
                _boxWidth = pcl::euclideanDistance(point1_x_y, point3_x_y);
                _boxHeight = clickedPoints[3].z;

                // Calculte the angle between the x axis and the object front edge
                Eigen::Vector4f line_dir(clickedPoints[1].x - clickedPoints[0].x,
                                         clickedPoints[1].y - clickedPoints[0].y,
                                         0, 0);
                Eigen::Vector4f x_axis(1,0,0,0);
                _boxYaw = pcl::getAngle3D (x_axis, line_dir);
                if(clickedPoints[1].y < clickedPoints[0].y){
                    _boxYaw = 2*M_PI-_boxYaw;
                }

                // Define the intial bounding box
                // New feature
                _boxRoll = 0;
                _boxPitch = 0;
                viewInteractor.defineBoundingBox(_objectPose, _boxRoll, _boxPitch, _boxYaw,
                                                 _boxLength, _boxWidth, _boxHeight);
                drawBoxAndPoints();

                // Set the value at the adecuated spin Box
                _ui->boxLength->setValue(_boxLength*100);
                _ui->boxWidth->setValue(_boxWidth*100);
                _ui->boxHeight->setValue(_boxHeight*100);

                // Actualize pose info
                actualizePoseInfo();
             }
        }
    }
}

// Confirm object position
void MainWindow::on_actionConfirm_position_triggered(){
    confirmObjectPosition();
}

// Delete object in the object list
void MainWindow::on_actionDelete_object_triggered(){
    if(objectsInfo.numberOfObjects()>0){
        // Select the object to delete from the annotated objects
        selectObject chooseObjectToDelete(0,2);
        QStringList listOfCurrentObjects = objectsInfo.getListOfObjects();
        chooseObjectToDelete.setObjectList(listOfCurrentObjects);
        chooseObjectToDelete.exec();
        QString objectToDelete = chooseObjectToDelete.getObjectName();

        if(!objectToDelete.isEmpty()){
            // Get the index and delete it
            int index = objectsInfo.getIndex(objectToDelete);
            objectsInfo.deleteObject(objectToDelete);

            // Remove it from the tree widget
            QTreeWidgetItem *child = _ui->treeWidget->topLevelItem(1)->takeChild(index);
            _ui->treeWidget->topLevelItem(1)->removeChild(child);
        }
    }
    else{
        QMessageBox::warning(this, "No objects", "No objects to remove.");
    }
}

// Import objects' information from a .xml file
void MainWindow::on_actionImport_objects_info_triggered(){
    // Name of the .xml file to import equal as the cloud file loaded
    std::string fileName = _fileName.toStdString();
    fileName.erase(fileName.find_last_of(".")+1, 3);
    fileName.append("xml");

    QString loadFileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open file"),
                                                        QString::fromStdString(fileName),
                                                        tr("Xml Files (*.xml)"));
    if(!loadFileName.isEmpty()){
        // Remove all the information saved in the tree widget
        clearInfoTreeWidget();

        // Remove all the object's information saved
        objectsInfo.clear();

        // Clean the viewer
        viewInteractor.cleanViewer();

        // Import the information
        _scenario = objectsInfo.importObjectsInformation(loadFileName);

        // Insert the items in the tree widget
        QTreeWidgetItem *itm_scenario = new QTreeWidgetItem(_ui->treeWidget);
        itm_scenario->setText(0, _scenario);
        _ui->treeWidget->insertTopLevelItem(0,itm_scenario);

        // Insert the objects items
        QTreeWidgetItem *itm_objects = new QTreeWidgetItem(_ui->treeWidget);
        itm_objects->setText(0, QString::fromStdString("Objects"));
        _ui->treeWidget->insertTopLevelItem(0,itm_objects);

        // Actualize tree widget with the loaded objects and the table size
        actualizeInformationTreeWidget();

        // Allow the user insert new objects
        _planeSegmentated = true;
    }
    else{
        QMessageBox::warning(this, "Error", "File not load.");
    }
}

// Export the current information about the objects annotated
void MainWindow::on_actionExport_objects_info_triggered(){
    if(objectsInfo.getDeskLength() != -1){
        // Set the same name of the cloud file but with the extension xml
        std::string fileName = _fileName.toStdString();
        fileName.erase(fileName.find_last_of(".")+1, 3);
        fileName.append(".xml");

        QString saveFileName = QFileDialog::getSaveFileName(this,
                                                            tr("Export objects' information"),
                                                            QString::fromStdString(fileName),
                                                            tr("Xml Files(*.xml)"));
        if(!saveFileName.isEmpty()){
            objectsInfo.exportObjectsInformation(saveFileName, _fileName, _scenario);
        }
        else{
            QMessageBox::warning(this, "Error", "File not saved.");
        }
    }
    else {
        QMessageBox::warning(this,
                             "Error",
                             "Length and width of the plane not defined. Please, proceed to do the plane segmentation");
    }
}

// Help button. Open the user guide
void MainWindow::on_actionHelp_triggered(){
    QString help_doc = qApp->applicationDirPath();
    help_doc.remove("bin");
    help_doc.append("documentation/user_guide.pdf");
    QDesktopServices::openUrl(QUrl(help_doc,  QUrl::TolerantMode));
}

// Remove or insert coordinate system
void MainWindow::on_actionCoordinate_system_toggled(bool arg1){
    if(arg1)
        viewInteractor.addCoordinateSystem();
    else
        viewInteractor.removeCoordinateSystem();
}

// Next 5 functions: Slots to change the position of
// the camera view
void MainWindow::on_actionUp_triggered(){
    viewInteractor.setCameraPose(0.5, 0.5, 3, 0.5, 0.5, 0, 0, 1, 0);
}

void MainWindow::on_actionFront_triggered(){
    viewInteractor.setCameraPose(0.5, -2, 0, 0.5, 0, 0, 0, 1, 1);
}

void MainWindow::on_actionLeft_triggered(){
    viewInteractor.setCameraPose(-2, 0.5, 0, 0, 0.5, 0, 1, 0, 1);
}

void MainWindow::on_actionRight_triggered(){
    viewInteractor.setCameraPose(2, 0.5, 0, 0, 0.5, 0, -1, 0, 1);
}

void MainWindow::on_actionBack_triggered(){
    viewInteractor.setCameraPose(1, 3, 0, 1, 0, 0, 0, 1, 1);
}

// Action to use when a value on the pose widget has been
// changed manualy
void MainWindow::on_poseInfo_itemChanged(QTreeWidgetItem *item, int column){
    if(_insertingObject){
        bool isNumber;
        float insertedNumber = item->text(1).toFloat(&isNumber);

        if(isNumber){
            if(item->text(0).startsWith(QString::fromStdString("X (m)"))){
                _objectPose.x = insertedNumber;
                actualizePose();
            }
            else if(item->text(0).startsWith(QString::fromStdString("Y (m)"))){
                _objectPose.y = insertedNumber;
                actualizePose();
            }
            else if(item->text(0).startsWith(QString::fromStdString("Z (m)"))){
                _objectPose.z = insertedNumber;
                actualizePose();
            }
            else if(item->text(0).startsWith(QString::fromStdString("Roll"))){
                if(insertedNumber <= 360.0 && insertedNumber >=0.0){
                    _boxRoll = pcl::deg2rad(insertedNumber);
                    actualizeRotation();
                }
                else
                    QMessageBox::warning(this, "Incorrect value", "Insert an angle value between 0 and 360.");
            }
            else if(item->text(0).startsWith(QString::fromStdString("Pitch"))){
                if(insertedNumber <= 360.0 && insertedNumber >=0.0){
                    _boxPitch = pcl::deg2rad(insertedNumber);
                    actualizeRotation();
                }
                else
                    QMessageBox::warning(this, "Incorrect value", "Insert an angle value between 0 and 360.");
            }
            else if(item->text(0).startsWith(QString::fromStdString("Yaw"))){
                if(insertedNumber <= 360.0 && insertedNumber >=0.0){
                    _boxYaw = pcl::deg2rad(insertedNumber);
                    actualizeRotation();
                }
                else
                    QMessageBox::warning(this, "Incorrect value", "Insert an angle value between 0 and 360.");
            }
        }
        else{
            QMessageBox::warning(this, "Format error", "Value incorrect. Please, insert a number.");
            item->setText(1, QString::number(0));
        }
    }
    else if(!_itemSelected)
        item->setText(1, QString::number(0));

    //    if(!_insertingObject && !_itemSelected)
    //        item->setText(1, QString::number(0));
}

// Save the pcd file and the objects' information
void MainWindow::save_Option(int type)
{
    QString fileNamePCD = "";
 
if(_pcdLoaded){
    
    if (type == 1){
    fileNamePCD = QFileDialog::getSaveFileName(this,
                                                       tr("Save PCD and XML files"));
                                                       //_fileName.remove(_fileName.size()-4,4),
                                                       //tr(""));
    }

    else{
    fileNamePCD = QFileDialog::getSaveFileName(this,
                                                        tr("Save As PCD and XML files"));
                                                         //_fileName.remove(_fileName.size()-4,4),
                                                         //tr(""));
    }

    
    if(fileNamePCD != ""){
    _firstsave = false;
    }

    if (!fileNamePCD.contains(".", Qt::CaseInsensitive))
        /*fileNamePCD = _fileName.remove(_fileName.indexOf("."),_fileName.length()-fileName.indexOf("."));
    else*/
    fileNamePCD.append(".pcd");

    std::string fileName = _fileName.remove(_fileName.indexOf("."),_fileName.length()-_fileName.indexOf(".")).toStdString();
    fileName.append(".xml");

    QString fileNameXML =  QString::fromStdString(fileName);

    if(!fileNamePCD.isEmpty()){
        // Save the .pcd file
         pcl::io::savePCDFileBinary(fileNamePCD.toStdString(), *_cloud);
        _fileName = fileNamePCD;
        _cloudModified = false;

        // Save the objects' information
        //fileNameXML.append(".xml");
        if(objectsInfo.getDeskLength() != -1){
            objectsInfo.exportObjectsInformation(fileNameXML, _fileName, _scenario);
        }
        else
            QMessageBox::warning(this,
                                 "XML file not saved",
                                 "The objects' information has not been saved because there is no information.");

        
    }
    else{
        QMessageBox::warning(this, "Error", "Files not saved.");
    }
}

else
    QMessageBox::warning(this,
                         "No PCD file opened",
                         "Open a PCD file and try again..");
}

// Enable or disable info messages
void MainWindow::on_actionShow_info_messages_toggled(bool arg1)
{
    if(arg1)
        _showInfoMsgs = true;
    else
        _showInfoMsgs = false;
}

// Undo function
void MainWindow::on_actionUndo_triggered()
{
    // Reload the previous point cloud and visualize it
    pcl::copyPointCloud(*_cloudUndo, *_cloud);
    visualize();

    // If the undo is done after the segmentation, delete the information
    // of the tree widget and allow the user to do the segmentation again
    if(!_ui->actionDesk_segmentation->isEnabled()){
        clearInfoTreeWidget();
        _ui->actionDesk_segmentation->setEnabled(true);
        objectsInfo.clear();
    }

    // Disable this action
    _ui->actionUndo->setEnabled(false);
}

// Downsampling function
void MainWindow::on_actionDownsample_point_cloud_triggered()
{
    // Call the dialog to introduce the filter values
    filtervaluesdialog filterDialog;
    filterDialog.exec();
    float leaf = filterDialog.getLeafSize();

    // Copy the filter to Undo function
    pcl::copyPointCloud(*_cloud, *_cloudUndo);

    // Create the filtering object
    pcl::VoxelGrid<pointT> sor;
    sor.setInputCloud (_cloud);
    sor.setLeafSize(leaf, leaf, leaf);
    //    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*_cloud);
    visualize();

    // Enable Undo function
    _ui->actionUndo->setEnabled(true);
}

// QSR values displayed in the terminal
void MainWindow::on_actionQSR_values_triggered()
{
    if(objectsInfo.numberOfObjects()>1){
        qsr QSR(objectsInfo.getObjectList());
        QSR.calculateQSRRight();
        QSR.calculateQSRLeft();
        QSR.calculateQSRInFront();
        QSR.calculateQSRBehind();
    }
    else QMessageBox::warning(this,
                              "Error",
                              "It is not possible to calculate the QSR values. Add more objects or load an annotation.");
}

// Obtain a description of the scene using QSR
void MainWindow::on_actionDescription_of_scene_using_QSR_triggered()
{
    if(objectsInfo.numberOfObjects()>1){
        qsr QSR(objectsInfo.getObjectList());
        QString description = QSR.getDescription();

        QMessageBox::information(this,
                                 "Description of the scene",
                                 description);

    }
    else QMessageBox::warning(this,
                              "Error",
                              "It is not possible to obtain a description of the scene. Annotate some objects or load an annotation.");
}

// Save all the QSR values in a .txt file
void MainWindow::on_actionSave_QSR_in_txt_file_triggered()
{
    if(objectsInfo.numberOfObjects()>1){
        qsr QSR(objectsInfo.getObjectList());
        std::string fileName = _fileName.toStdString();
        fileName.erase(fileName.find_last_of(".")+1, 3);
        fileName.append("txt");

        QString file = QFileDialog::getSaveFileName(this,
                                                        tr("Save file"),
                                                        QString::fromStdString(fileName),
                                                        tr("Text File(*.txt)"));
        if(!file.isEmpty()){
            std::ofstream outputFile;
            outputFile.open (file.toAscii(), ios::trunc);
            QString allValues = QSR.getAllValues();
            outputFile << allValues.toStdString();
            outputFile.close();
        }
        else{
            QMessageBox::warning(this, "Error", "Text file not saved.");
        }

    }
    else QMessageBox::warning(this,
                              "Error",
                              "It is not possible to obtain a description of the scene. Annotate some objects or load an annotation.");
}

///////////////////////////////////////////////////////
// The following functions are used inside the above //
// functions (slots)                                 //
///////////////////////////////////////////////////////

// Initialization
void MainWindow::init(){
    //Initialization of the UI
    _ui->setupUi(this);

    // Start the bool variables
    _pcdLoaded = false;
    _pcdLoadError = false;
    _planeDefined = false;
    _planeSegmentated = false;
    _insertingObject = false;
    _objectModifed = false;
    _cloudModified = false;
    _itemSelected = false;
    _showInfoMsgs = true;

    _firstsave = false;

    //Set to black the background color of the QVTKWidget
    QPalette palette = _ui->qvtkWidget->palette();
    palette.setColor(QPalette::Background, QColor("black"));
    _ui->qvtkWidget->setPalette(palette);
    _ui->qvtkWidget->setAutoFillBackground(true);

    //Windows title
    setWindowTitle("3D Annotation Tool");

    // Clear tree widget
    _ui->treeWidget->clear();

    // Read the information left for the user in the previous session
    QString info_doc = qApp->applicationDirPath();
    info_doc.remove("bin");
    info_doc.append("/info_app.xml");;

    boost::property_tree::ptree root;
    read_xml(info_doc.toStdString(), root);


    _showInitialMsg = root.get<bool>("showInitial");
    _lastDir = QString::fromStdString(root.get<std::string>("lastDirectory"));

    // Set the render windowsn setup the interactor and register the callback
    _ui->qvtkWidget->SetRenderWindow(viewInteractor.getRenderWindow(_ui->qvtkWidget->width(), _ui->qvtkWidget->height()));
    viewInteractor.setupInteractor(_ui->qvtkWidget->GetInteractor(), _ui->qvtkWidget->GetRenderWindow());
    viewInteractor.registerCallback();
}


// Used to visualized the point cloud
void MainWindow::visualize(){
    if(_pcdLoaded || _pcdLoadError){
        // Clean the viewer if another pcd has been loaded before
        viewInteractor.cleanAll();
    }
    // Visualize the point cloud and set the render windows
    viewInteractor.visualizePointCloud(_cloud);

    // Add the coordinate system if it's requiered
    if(_ui->actionCoordinate_system->isChecked()){
        viewInteractor.addCoordinateSystem();
    }
}


// Exit
bool MainWindow::okToExit(){
    int r = QMessageBox::warning(this, tr("Exit"),
                                 tr("Are you sure you want to exit?"),
                                 QMessageBox::Yes , QMessageBox::No);
    if (r == QMessageBox::Yes) {
        on_actionExit_triggered();
    } else
        return false;
}


// Open the pcd file
void MainWindow::open_pcd_file(){
    if (_pcdLoaded){
        int r = QMessageBox::warning(this,
                                     tr("PCD open"),
                                     tr("Do you want to open another pcd file?"),
                                     QMessageBox::Yes , QMessageBox::No);
        if (r == QMessageBox::Yes) {
            _fileName = QFileDialog::getOpenFileName(this,
                                                     tr("Open file"),
                                                     _lastDir,
                                                     tr("PCD Files(*.pcd)"));
            if(!_fileName.isEmpty()){
                load_pcd_file(_fileName);
                _firstsave = true;

            }
            else{
                QMessageBox::warning(this, "Error", "PCD file not loaded, do it again.");
                _pcdLoadError=true;
            }
        }
    }
    else{
        _fileName = QFileDialog::getOpenFileName(this,
                                                 tr("Open file"),
                                                 _lastDir,
                                                 tr("PCD Files(*.pcd)"));
        if(!_fileName.isEmpty()){
            load_pcd_file(_fileName);
            _firstsave = true;
        }
        else{
            QMessageBox::warning(this, "Error", "PCD file not loaded, do it again.");
        }
    }
}

void MainWindow::load_pcd_file(QString fileName){
    // Point cloud reader
    pcl::PCDReader reader;

    // Remove all the information saved of the tree widget
    clearInfoTreeWidget();

    // Clear the pose information
    clearPoseInfo();

    // Remove all the objects information saved
    objectsInfo.clear();
    _planeDefined = false;
    _planeSegmentated = false;

    // Read the point cloud
    reader.read(fileName.toStdString(), *_cloud);

    // Remove Nan points
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*_cloud, *_cloud, index);

    // Visualize the point cloud loaded
    visualize();
    _pcdLoaded=true;
}

// Draws the bounding box and the points inside it. Also actualize the pose info
void MainWindow::drawBoxPointsAndNewInfo(){
    if(_insertingObject){
        drawBoxAndPoints();
        actualizePoseInfo();
    }
}

// Set the values of the pose in the pose info tree widget
void MainWindow::setPoseInfo(pcl::PointXYZ boxPose, float boxRoll, float boxPitch, float boxYaw){
    std::stringstream x, y, z , roll, pitch, yaw;

    x << setprecision(3) << boxPose.x;
    y << setprecision(3) << boxPose.y;
    z << setprecision(3) << boxPose.z;
    roll << setprecision(3) << pcl::rad2deg(boxRoll);
    pitch << setprecision(3) << pcl::rad2deg(boxPitch);
    yaw << setprecision(3) << pcl::rad2deg(boxYaw);

    _ui->poseInfo->topLevelItem(0)->setText(1, QString::fromStdString(x.str()));
    _ui->poseInfo->topLevelItem(1)->setText(1, QString::fromStdString(y.str()));
    _ui->poseInfo->topLevelItem(2)->setText(1, QString::fromStdString(z.str()));
    _ui->poseInfo->topLevelItem(3)->setText(1, QString::fromStdString(roll.str()));
    _ui->poseInfo->topLevelItem(4)->setText(1, QString::fromStdString(pitch.str()));
    _ui->poseInfo->topLevelItem(5)->setText(1, QString::fromStdString(yaw.str()));
}

// Clears the information displayed in the pose info tree widget
void MainWindow::clearPoseInfo(){
    _ui->poseInfo->topLevelItem(0)->setText(1, "0");
    _ui->poseInfo->topLevelItem(1)->setText(1, "0");
    _ui->poseInfo->topLevelItem(2)->setText(1, "0");
    _ui->poseInfo->topLevelItem(3)->setText(1, "0");
    _ui->poseInfo->topLevelItem(4)->setText(1, "0");
    _ui->poseInfo->topLevelItem(5)->setText(1, "0");

    _ui->boxLength->setValue(0);
    _ui->boxWidth->setValue(0);
    _ui->boxHeight->setValue(0);
}

// Actualizes the pose values of the pose info tree widget
void MainWindow::actualizePoseInfo(){
    std::stringstream x, y, z , roll, pitch, yaw;

    x << setprecision(3) << _objectPose.x;
    y << setprecision(3) << _objectPose.y;
    z << setprecision(3) << _objectPose.z;
    roll << setprecision(3) << pcl::rad2deg(_boxRoll);
    pitch << setprecision(3) << pcl::rad2deg(_boxPitch);
    yaw << setprecision(3) << pcl::rad2deg(_boxYaw);

    _ui->poseInfo->topLevelItem(0)->setText(1, QString::fromStdString(x.str()));
    _ui->poseInfo->topLevelItem(1)->setText(1, QString::fromStdString(y.str()));
    _ui->poseInfo->topLevelItem(2)->setText(1, QString::fromStdString(z.str()));
    _ui->poseInfo->topLevelItem(3)->setText(1, QString::fromStdString(roll.str()));
    _ui->poseInfo->topLevelItem(4)->setText(1, QString::fromStdString(pitch.str()));
    _ui->poseInfo->topLevelItem(5)->setText(1, QString::fromStdString(yaw.str()));
}

void MainWindow::actualizePose(){
    viewInteractor.redefinePose(_objectPose);
    if(_insertingObject) _objectModifed = true;
    drawBoxPointsAndNewInfo();
}

void MainWindow::actualizeRotation(){
    viewInteractor.redefineRotation(_boxRoll, _boxPitch, _boxYaw);
    if(_insertingObject) _objectModifed = true;
    drawBoxPointsAndNewInfo();
}

void MainWindow::actualizeBoxWidth(){
    viewInteractor.redefineWidth(_boxWidth);
    if(_insertingObject) _objectModifed = true;
    drawBoxAndPoints();
}

void MainWindow::actualizeBoxLength(){
    viewInteractor.redefineLength(_boxLength);
    if(_insertingObject) _objectModifed = true;
    drawBoxPointsAndNewInfo();
}

void MainWindow::actualizeBoxHeight(){
    viewInteractor.redefineHeight(_boxHeight);
    if(_insertingObject) _objectModifed = true;
    drawBoxPointsAndNewInfo();
}

void MainWindow::drawBoxAndPoints(){
    viewInteractor.drawBoundingBox();
    viewInteractor.highligthPointsinBox(_cloud);
}

void MainWindow::actualizeInformationTreeWidget(){
    // Actualize tree widget with the loaded objects and the table size
    // Insert length information
    displayDeskLengthInfo();

    // Insert width information
    displayDeskWidthInfo();

    // Object's information
    displayObjectsInfo();
}

void MainWindow::displayDeskLengthInfo(){
    QTreeWidgetItem *itm = new QTreeWidgetItem(_ui->treeWidget->topLevelItem(0));
    std::stringstream info;
    info << "Length: " << setprecision(3) << objectsInfo.getDeskLength() << " m";
    itm->setText(0,QString::fromStdString(info.str()));
    _ui->treeWidget->insertTopLevelItem(0,itm);
}

void MainWindow::displayDeskWidthInfo(){
    QTreeWidgetItem *itm = new QTreeWidgetItem(_ui->treeWidget->topLevelItem(0));
    std::stringstream info;
    info << "Width: " << setprecision(3) << objectsInfo.getDeskWidth() << " m";
    itm->setText(0,QString::fromStdString(info.str()));
    _ui->treeWidget->insertTopLevelItem(0,itm);
}

void MainWindow::displayObjectsInfo(){
    for(int i = 0; i < objectsInfo.numberOfObjects(); i++){
        QTreeWidgetItem *itm = new QTreeWidgetItem(_ui->treeWidget->topLevelItem(1));
        itm->setText(0,objectsInfo.nameOfObject(i));
        _ui->treeWidget->addTopLevelItem(itm);
    }
}

void MainWindow::clearInfoTreeWidget(){
    _ui->treeWidget->clear();
}

void MainWindow::confirmObjectPosition(){
    _insertingObject = false;
    _objectModifed = false;

    if(!_objectName.isEmpty()){
        // Get the indices inside the current bounding box
        pcl::PointIndices indices;
        viewInteractor.getPointsInBoundingBox(_cloud, &indices);


        if(objectsInfo.existsObject(_objectName)){
            // If the object already exists, modify it
            objectsInfo.modifyObject(_objectName,
                                     _objectPose,
                                     _boxLength,
                                     _boxWidth,
                                     _boxHeight,
                                     _boxRoll,
                                     _boxPitch,
                                     _boxYaw,
                                     &indices);
        }
        else{
            // Insert the object
            objectsInfo.insertObject(_objectName,
                                     _objectPose,
                                     _boxLength,
                                     _boxWidth,
                                     _boxHeight,
                                     _boxRoll,
                                     _boxPitch,
                                     _boxYaw,
                                     &indices);

            // Insert the object in the Information tree Widget
            QTreeWidgetItem *itm = new QTreeWidgetItem(_ui->treeWidget->topLevelItem(1));
            itm->setText(0,QString::fromStdString(_objectName.toStdString()));
            _ui->treeWidget->addTopLevelItem(itm);
        }

        // Clear the pose information displayed
        clearPoseInfo();

        // Remove the box and the lighted points
        viewInteractor.cleanViewer();
    }
}

void MainWindow::showInitialMessage(){
    if(_showInitialMsg){
        initialmessagedialog msg;
        msg.exec();
        if(msg.getBoolValue()){
            // Write false in the output file
            QString info_doc = qApp->applicationDirPath();
            info_doc.remove("bin");
            info_doc.append("info_app.xml");;

            boost::property_tree::ptree root;

            root.put("showInitial", false);
            root.put("lastDirectory", _lastDir.toStdString());

            boost::property_tree::xml_writer_settings<char> settings(' ', 3);
            write_xml(info_doc.toStdString(), root, std::locale(), settings);
        }
    }
}


void MainWindow::on_actionSaveAs_PCD_xml_triggered()
{
    save_Option(2); //Save As
}


void MainWindow::closeEvent(QCloseEvent *event)
{
    if (okToExit()){
      event->accept();
    } else {
      event->ignore();
    }
}

void MainWindow::on_actionShiftorigin_triggered()
{
    if(_pcdLoaded){

    pointT pointPicked;
    if (objectsInfo.numberOfObjects()==0){
    // Pick the origin
    if(_showInfoMsgs) QMessageBox::information(this,
                                               "Pick the origin point",
                                               "Pick a desired origin point with shift+left mouse button.");
    viewInteractor.getPointPicked(&pointPicked);

    // Move the pointcloud to the new origin
    cloudModifier.translate_on_plane_x_y(_cloud, _cloud, pointPicked);
    visualize();
    }
    else
        QMessageBox::warning(this,
                             "Cannot shift origin",
                             "There are already some objects with the current origin as reference.");
    }

    else
        QMessageBox::warning(this,
                             "No PCD file opened",
                             "Open a PCD file and try again..");
}

void MainWindow::on_actionUndoPoints_triggered()
{
    viewInteractor.undo_points();
}
