#include "qsr.h"

#include <math.h>
#include <iomanip>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>

using namespace std;

qsr::qsr(std::vector<object> objectList)
{
    _objectList = objectList;
    _distanceThreshold = 1.5;
}


// This function calculate all the values for the relations right of
// and display its results in the terminal
void qsr::calculateQSRRight(){
    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    // **** Need to add that if the landmark is a Mouse or Mug do not calculate *****
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrRight();
            }
        }
    }

    stringstream ss2;

    getStringStream(&values[0][0], ss2, "Right");

    cout << ss2.str() << endl;
}

// This function calculate all the values for the relations left of
// and display its results in the terminal
void qsr::calculateQSRLeft(){
    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrLeft();
            }
        }
    }

    stringstream ss2;

    getStringStream(&values[0][0], ss2, "Left");

    cout << ss2.str() << endl;
}


// This function calculate all the values for the relations front of
// and display its results in the terminal
void qsr::calculateQSRInFront(){

    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrInFront();
            }
        }
    }

    stringstream ss2;

    getStringStream(&values[0][0], ss2, "Front");

    cout << ss2.str() << endl;
}

// This function calculate all the values for the relations behind of
// and display its results in the terminal
void qsr::calculateQSRBehind(){

    float values[_objectList.size()][_objectList.size()];

    // Calculate values
    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i) values[j][i] = -1;
            else{
                calculatePointTrajector(_objectList[j]);
                values[j][i] = qsrBehind();
            }
        }
    }

    stringstream ss2;

    getStringStream(&values[0][0], ss2, "Behind");

    cout << ss2.str() << endl;
}

// Calculate the center of mass of an object as the centroid of the bounding box
pcl::PointXYZ qsr::getCenterOfMass(object obj){
    pcl::PointXYZ centerOfMass;
    centerOfMass.x = obj.geometry.length/2;
    centerOfMass.y = obj.geometry.width/2;
    centerOfMass.z = obj.geometry.height/2;

    Eigen::Affine3f transformation = pcl::getTransformation (obj.geometry.pose.x,
                                                             obj.geometry.pose.y,
                                                             obj.geometry.pose.z,
                                                             obj.geometry.roll,
                                                             obj.geometry.pitch,
                                                             obj.geometry.yaw);

    centerOfMass = pcl::transformPoint(centerOfMass, transformation);

    return centerOfMass;
}

// Calculate the needed parameters of the landmark.
void qsr::calculatePointsLandmark(object landmark){
    // Firstly, get the transformation of the landmark
    Eigen::Affine3f transformationLandmark = pcl::getTransformation (landmark.geometry.pose.x,
                                                                     landmark.geometry.pose.y,
                                                                     landmark.geometry.pose.z,
                                                                     landmark.geometry.roll,
                                                                     landmark.geometry.pitch,
                                                                     landmark.geometry.yaw);
    // Calculate the points in the center of each face
    // Right center point
    _RCPoint.x = landmark.geometry.length;
    _RCPoint.y = landmark.geometry.width/2;
    _RCPoint.z = landmark.geometry.height/2;
    _RCPoint = pcl::transformPoint(_RCPoint, transformationLandmark);

    // Left center point
    _LCPoint.x = 0;
    _LCPoint.y = landmark.geometry.width/2;
    _LCPoint.z = landmark.geometry.height/2;
    _LCPoint = pcl::transformPoint(_LCPoint, transformationLandmark);

    // Front center point
    _FCPoint.x = landmark.geometry.length/2;
    _FCPoint.y = 0;
    _FCPoint.z = landmark.geometry.height/2;
    _FCPoint = pcl::transformPoint(_FCPoint, transformationLandmark);

    // Back center point
    _BCPoint.x = landmark.geometry.length/2;
    _BCPoint.y = landmark.geometry.width;
    _BCPoint.z = landmark.geometry.height/2;
    _BCPoint = pcl::transformPoint(_BCPoint, transformationLandmark);

    // Calculate the lower corners points
    // Front face, left lower point
    _FLDPoint.x = 0;
    _FLDPoint.y = 0;
    _FLDPoint.z = 0;
    _FLDPoint = pcl::transformPoint(_FLDPoint, transformationLandmark);

    // Front face, right lower point
    _FRDPoint.x = landmark.geometry.length;
    _FRDPoint.y = 0;
    _FRDPoint.z = 0;
    _FRDPoint = pcl::transformPoint(_FRDPoint, transformationLandmark);

    // Back face, left lower point
    _BLDPoint.x = landmark.geometry.length;
    _BLDPoint.y = landmark.geometry.width;
    _BLDPoint.z = 0;
    _BLDPoint = pcl::transformPoint(_BLDPoint, transformationLandmark);

    // Left face, right lower point
    _BRDPoint.x = 0;
    _BRDPoint.y = landmark.geometry.width;
    _BRDPoint.z = 0;
    _BRDPoint = pcl::transformPoint(_BRDPoint, transformationLandmark);

    // Calculate the perimeter of the landmark
    _perimeterLandmark = 2*landmark.geometry.length + 2*landmark.geometry.width;
}

// Calculate the needed parameters for the trajector
void qsr::calculatePointTrajector(object trajector){
    _centerOfMassTrajector = getCenterOfMass(trajector);
}

// Calculation of angles and distance for the relation right of
void qsr::calculateAnglesAndDistanceRight(){
    // Front face direction
    Eigen::Vector4f frontDirection4f(_FRDPoint.x - _FLDPoint.x, _FRDPoint.y - _FLDPoint.y, 0, 0);
    Eigen::Vector3f frontDirection3f = frontDirection4f.segment(0,3);

    // Line FRD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _FRDPoint.x,
                                            _centerOfMassTrajector.y - _FRDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line BLD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _BLDPoint.x,
                                           _centerOfMassTrajector.y - _BLDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = frontDirection3f.cross(rightCornerCenterLine3f);

    // Calculate the angle and modify depending the orientation
    _angleRightRight = pcl::getAngle3D (rightCornerCenterLine4f, frontDirection4f);
    if(crossProductRight(2)<0) _angleRightRight = 2*M_PI-_angleRightRight;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = frontDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftRight = pcl::getAngle3D (leftCornerCenterLine4f, frontDirection4f);
    if(crossProductLeft(2)>0) _angleLeftRight = 2*M_PI-_angleLeftRight;

    // Calculate the distance
    _distanceRight = pcl::euclideanDistance(_centerOfMassTrajector, _RCPoint);
}

// Calculation of angles and distance for the relation left of
void qsr::calculateAnglesAndDistanceLeft(){
    // Front face direction
    Eigen::Vector4f backDirection4f(_BRDPoint.x - _BLDPoint.x, _BRDPoint.y - _BLDPoint.y, 0, 0);
    Eigen::Vector3f backDirection3f = backDirection4f.segment(0,3);

    // Line BRD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _BRDPoint.x,
                                            _centerOfMassTrajector.y - _BRDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line FLD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _FLDPoint.x,
                                           _centerOfMassTrajector.y - _FLDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = backDirection3f.cross(rightCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleRightLeft = pcl::getAngle3D (rightCornerCenterLine4f, backDirection4f);
    if(crossProductRight(2)<0) _angleRightLeft = 2*M_PI-_angleRightLeft;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = backDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftLeft = pcl::getAngle3D (leftCornerCenterLine4f, backDirection4f);
    if(crossProductLeft(2)>0) _angleLeftLeft = 2*M_PI-_angleLeftLeft;

    // Calculate the distance
    _distanceLeft = pcl::euclideanDistance(_centerOfMassTrajector, _LCPoint);
}

// Calculation of angles and distance for the relation in front
void qsr::calculateAnglesAndDistanceInFront(){
    // Left face direction
    Eigen::Vector4f leftDirection4f(_FLDPoint.x - _BRDPoint.x, _FLDPoint.y - _BRDPoint.y, 0, 0);
    Eigen::Vector3f leftDirection3f = leftDirection4f.segment(0,3);

    // Line FLD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _FLDPoint.x,
                                            _centerOfMassTrajector.y - _FLDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line FRD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _FRDPoint.x,
                                           _centerOfMassTrajector.y - _FRDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = leftDirection3f.cross(rightCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleRightInFront = pcl::getAngle3D (rightCornerCenterLine4f, leftDirection4f);
    if(crossProductRight(2)<0) _angleRightInFront = 2*M_PI-_angleRightInFront;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = leftDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftInFront = pcl::getAngle3D (leftCornerCenterLine4f, leftDirection4f);
    if(crossProductLeft(2)>0) _angleLeftInFront = 2*M_PI-_angleLeftInFront;

    // Calculate the distance
    _distanceInFront = pcl::euclideanDistance(_centerOfMassTrajector, _FCPoint);
}

// Calculation of angles and distance for the relation behind
void qsr::calculateAnglesAndDistanceBehind(){
    // Left face direction
    Eigen::Vector4f rightDirection4f(_BLDPoint.x - _FRDPoint.x, _BLDPoint.y - _FRDPoint.y, 0, 0);
    Eigen::Vector3f rightDirection3f = rightDirection4f.segment(0,3);

    // Line BLD and center of mass
    Eigen::Vector4f rightCornerCenterLine4f(_centerOfMassTrajector.x - _BLDPoint.x,
                                            _centerOfMassTrajector.y - _BLDPoint.y,
                                            0,
                                            0);

    Eigen::Vector3f rightCornerCenterLine3f = rightCornerCenterLine4f.segment(0,3);

    // Line BRD and center of mass
    Eigen::Vector4f leftCornerCenterLine4f(_centerOfMassTrajector.x - _BRDPoint.x,
                                           _centerOfMassTrajector.y - _BRDPoint.y,
                                           0,
                                           0);

    Eigen::Vector3f leftCornerCenterLine3f = leftCornerCenterLine4f.segment(0,3);

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductRight = rightDirection3f.cross(rightCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleRightBehind = pcl::getAngle3D (rightCornerCenterLine4f, rightDirection4f);
    if(crossProductRight(2)<0) _angleRightBehind = 2*M_PI-_angleRightBehind;

    // Cross product to know which is the orientation
    Eigen::Vector3f crossProductLeft = rightDirection3f.cross(leftCornerCenterLine3f);
    // Calculate the angle and modify depending the orientation
    _angleLeftBehind = pcl::getAngle3D (leftCornerCenterLine4f, rightDirection4f);
    if(crossProductLeft(2)>0) _angleLeftBehind = 2*M_PI-_angleLeftBehind;

    // Calculate the distance
    _distanceBehind = pcl::euclideanDistance(_centerOfMassTrajector, _BCPoint);
}

// Angle function
// Maybe is too much restrictive. Needed to anylize human behaviour when looking for a relation
float qsr::angleFunction(float angle){
    if(angle >= 0 && angle < M_PI_2) return 1.0;
    else if(angle >= M_PI_2 && angle < 2*M_PI/3) return 1-(36/(M_PI*M_PI))*(angle - M_PI_2)*(angle - M_PI_2);
    else if(angle >= 2*M_PI/3 && angle < 11*M_PI/6) return 0.0;
    else return 1 - (36/(M_PI*M_PI))*(angle - 2*M_PI)*(angle - 2*M_PI);
}


// Distance function version
float qsr::distanceFunction(float distance, float perimeter){
    float factor;

    // For small objects the distance function is more restrictive
    if(perimeter < 0.4) factor = 0.1;
    else factor = 0.2;

    if(distance < factor) return 1;
    else return exp(-((distance-factor)/0.4)*log(2));
}

float qsr::qsrRight(){
    calculateAnglesAndDistanceRight();
    return angleFunction(_angleRightRight)*angleFunction(_angleLeftRight)*distanceFunction(_distanceRight, _perimeterLandmark);
}

float qsr::qsrLeft(){
    calculateAnglesAndDistanceLeft();
    return angleFunction(_angleRightLeft)*angleFunction(_angleLeftLeft)*distanceFunction(_distanceLeft, _perimeterLandmark);
}

float qsr::qsrInFront(){
    calculateAnglesAndDistanceInFront();
    return angleFunction(_angleRightInFront)*angleFunction(_angleLeftInFront)*distanceFunction(_distanceInFront, _perimeterLandmark);
}

float qsr::qsrBehind(){
    calculateAnglesAndDistanceBehind();
    return angleFunction(_angleRightBehind)*angleFunction(_angleLeftBehind)*distanceFunction(_distanceBehind, _perimeterLandmark);
}

QString qsr::getDescription(){
    float values_right[_objectList.size()][_objectList.size()];
    float values_left[_objectList.size()][_objectList.size()];
    float values_front[_objectList.size()][_objectList.size()];
    float values_behind[_objectList.size()][_objectList.size()];

    stringstream ss;


    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i){
                values_left[j][i] = -1;
                values_right[j][i] = -1;
                values_front[j][i] = -1;
                values_behind[j][i] = -1;
            }
            else{
                calculatePointTrajector(_objectList[j]);
                values_right[j][i] = qsrRight();
                values_left[j][i] = qsrLeft();
                values_front[j][i] = qsrInFront();
                values_behind[j][i] = qsrBehind();
            }
            if(values_right[j][i] >= 0.5 && ( _objectList[i].name.toStdString() != "mug" && _objectList[i].name.toStdString() != "mouse" &&  _objectList[i].name.toStdString() != "Mouse" &&  _objectList[i].name.toStdString() != "Mug")){
                ss << "- The " << _objectList[j].name.toStdString() << " is to the right of the " << _objectList[i].name.toStdString() << "." << endl << endl;
            }
            if(values_left[j][i] >= 0.5 && ( _objectList[i].name.toStdString() != "mug" && _objectList[i].name.toStdString() != "mouse" &&  _objectList[i].name.toStdString() != "Mouse" &&  _objectList[i].name.toStdString() != "Mug")){
                ss << "- The " << _objectList[j].name.toStdString() << " is to the left of the " << _objectList[i].name.toStdString() << "." << endl << endl;
            }
            if(values_front[j][i] >= 0.5 && ( _objectList[i].name.toStdString() != "mug" && _objectList[i].name.toStdString() != "mouse" &&  _objectList[i].name.toStdString() != "Mouse" &&  _objectList[i].name.toStdString() != "Mug")){
                ss << "- The " << _objectList[j].name.toStdString() << " is in front of the " << _objectList[i].name.toStdString() << "." << endl << endl;
            }
            if(values_behind[j][i] >= 0.5 && ( _objectList[i].name.toStdString() != "mug" && _objectList[i].name.toStdString() != "mouse" &&  _objectList[i].name.toStdString() != "Mouse" &&  _objectList[i].name.toStdString() != "Mug")){
                ss << "- The " << _objectList[j].name.toStdString() << " is behind the " << _objectList[i].name.toStdString() << "." << endl << endl;
            }
        }
    }

    return QString::fromStdString(ss.str());
}

QString qsr::getAllValues(){
    float values_right[_objectList.size()][_objectList.size()];
    float values_left[_objectList.size()][_objectList.size()];
    float values_front[_objectList.size()][_objectList.size()];
    float values_behind[_objectList.size()][_objectList.size()];

    for(int i=0; i < _objectList.size(); i++){
        calculatePointsLandmark(_objectList[i]);
        for(int j=0; j < _objectList.size(); j++){
            if(j == i){
                values_left[j][i] = -1;
                values_right[j][i] = -1;
                values_front[j][i] = -1;
                values_behind[j][i] = -1;
            }
            else{
                calculatePointTrajector(_objectList[j]);
                values_right[j][i] = qsrRight();
                values_left[j][i] = qsrLeft();
                values_front[j][i] = qsrInFront();
                values_behind[j][i] = qsrBehind();
            }
        }
    }

    stringstream ss;

    getStringStream(&values_right[0][0], ss, "Right");
    getStringStream(&values_left[0][0], ss, "Left");
    getStringStream(&values_front[0][0], ss, "Front");
    getStringStream(&values_behind[0][0], ss, "Behind");

    return QString::fromStdString(ss.str());
}



void qsr::getStringStream(float *values, stringstream &ss, string relation){

    // Print the relation header
    if(relation == "Right"){
        ss << "/////////////////////////" << endl;
        ss << "/         Right         /" << endl;
        ss << "/////////////////////////\n" << endl;
    }
    else if(relation == "Left"){
        ss << "/////////////////////////" << endl;
        ss << "/         Left          /" << endl;
        ss << "/////////////////////////\n" << endl;
    }
    else if(relation == "Front"){
        ss << "/////////////////////////" << endl;
        ss << "/         Front         /" << endl;
        ss << "/////////////////////////\n" << endl;
    }
    else if(relation == "Behind"){
        ss << "/////////////////////////" << endl;
        ss << "/         Behind        /" << endl;
        ss << "/////////////////////////\n" << endl;
    }

    // Print the table
    for(int i =0; i < _objectList.size()+1; i++){
        ss << "|" << setw(10) << "----------";
    }

    ss << "|" << endl << "|" << setw(10) << "";
    for(int i = 0; i < _objectList.size(); i++){
        ss << "|" << setw(10) << _objectList[i].name.toStdString();
    }

    ss << "|" << endl;

    for(int i =0; i < _objectList.size()+1; i++){
        ss << "|" << setw(10) << "----------";
    }

    ss << "|" << endl;

    for(int i = 0; i < _objectList.size(); i++){
        ss << "|" << setw(10) << right << _objectList[i].name.toStdString() << "|";
        for(int j = 0; j < _objectList.size(); j++){
            if(i==j) ss << setw(10) << right << "X" << "|";
            else ss << setw(10) << right << setprecision(2)  << values[i*_objectList.size()+j] << "|";
        }
        ss << endl;

        for(int i =0; i < _objectList.size()+1; i++){
            ss << "|" << setw(10) << "----------";
        }

        ss << "|" << endl;
    }
}
