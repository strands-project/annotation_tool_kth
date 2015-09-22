#include "objectsinformation.h"
#include <fstream>
#include <iostream>
#include <ios>

#include <sstream>
#include <boost/property_tree/xml_parser.hpp>
#include <QStringList>

using namespace boost::property_tree;

static const std::string TAG_SCENARIO = "scenario";
static const std::string TAG_NAME = "name";
static const std::string TAG_OBJECT = "object";
static const std::string TAG_NOBJECTS = "numberOfObjects";
static const std::string TAG_ALLOBJECTS = "allObjects";
static const std::string TAG_POSE = "pose";
static const std::string TAG_DIMENSIONS = "dimensions";
static const std::string TAG_LENGTH = "length";
static const std::string TAG_WIDTH = "width";
static const std::string TAG_HEIGHT = "height";
static const std::string TAG_COLOR = "color";
static const std::string TAG_INDICES = "indices";

objectsInformation::objectsInformation()
{
    //Asign the name of the colors
    std::string colors[]={"red",
                          "green",
                          "blue",
                          "yellow",
                          "pink",
                          "turquoise",
                          "orange",
                          "purple",
                          "dark green",
                          "beige",
                          "brown",
                          "gold",
                          "salmon",
                          "grey",
                          "blueViolet",
                          "red",
                          "green",
                          "blue",
                          "yellow",
                          "pink",
                          "turquoise",
                          "orange",
                          "purple",
                          "dark green",
                          "beige",
                          "brown",
                          "gold",
                          "salmon",
                          "grey",
                          "blueViolet"};
    _nameColors.assign(colors, colors+29);
}


void objectsInformation::insertObject(QString name,
                                      pcl::PointXYZ pose,
                                      float length,
                                      float width,
                                      float height,
                                      float roll,
                                      float pitch,
                                      float yaw,
                                      pcl::PointIndices *indices){

    // Set the parameters of the current object and save it into the object list
    object currentObject;
    currentObject.name = name;
    currentObject.geometry.pose = pose;
    currentObject.geometry.length = length;
    currentObject.geometry.width = width;
    currentObject.geometry.height = height;
    currentObject.geometry.roll = roll;
    currentObject.geometry.pitch = pitch;
    currentObject.geometry.yaw = yaw;

    for(int i=0; i < indices->indices.size(); i++){
        currentObject.indices.indices.push_back(indices->indices[i]);
    }
    _objectList.push_back(currentObject);
}

void objectsInformation::deleteObject(QString objectName){
    int index = getIndex(objectName);
    _objectList.erase(_objectList.begin()+index);
}

void objectsInformation::clear(){
    _objectList.clear();
    _tableLength = -1;
    _tableWidth = -1;
}

int objectsInformation::getIndex(QString objectName){
    int index;

    for(index = 0; index < _objectList.size(); index++){
        if(_objectList[index].name.toStdString() == objectName.toStdString()) break;
    }
    if(index == _objectList.size()) index=-1;
    return index;
}

void objectsInformation::getIndices(int indexObject, pcl::PointIndices *indices){
    for(int i=0; i < _objectList[indexObject].indices.indices.size() ; i++){
        indices->indices.push_back(_objectList[indexObject].indices.indices[i]);
    }
}

void objectsInformation::setDeskDimensions(float length, float width){
    _tableLength = length;
    _tableWidth = width;
}

QString objectsInformation::nameOfObject(int index){
    return _objectList[index].name;
}

void objectsInformation::exportObjectsInformation(QString xmlFile, QString pcdFile, QString scenario ){
    ptree root;

    ptree& node = root.add("scenario", "");
    node.add("type", scenario.toStdString());
    node.add("annotatedFrom", pcdFile.toStdString());
    ptree* data = &node.add(TAG_DIMENSIONS, "");
    data->put(TAG_LENGTH, _tableLength);
    data->put(TAG_WIDTH, _tableWidth);
    ptree* allObjects = &node.add(TAG_ALLOBJECTS,"");
    allObjects->put(TAG_NOBJECTS, _objectList.size());
    for(int index = 0; index < _objectList.size(); index++){
        addObject(*allObjects, index);
    }

    //Write
    xml_writer_settings<char> settings(' ', 3);
    write_xml(xmlFile.toStdString(), root, std::locale(), settings);
}

void objectsInformation::addObject(ptree &parent, int index){
    ptree* objectData = &parent.add(TAG_OBJECT, "");
    objectData->put(TAG_NAME, _objectList[index].name.toStdString());
    objectData->put(TAG_COLOR, _nameColors[index]);
    ptree* pose = &objectData->add(TAG_POSE, "");
    addPose(*pose, index);
    ptree* boundingBox = &objectData->add(TAG_DIMENSIONS, "");
    addBoundingBox(*boundingBox, index);

    std::ostringstream str;
    for(int j = 0; j <_objectList[index].indices.indices.size(); j++){
        str << _objectList[index].indices.indices[j] << " ";
    }
    objectData->add(TAG_INDICES, str.str());
}

void objectsInformation::addPose(ptree &node, int index){
    node.put("x", _objectList[index].geometry.pose.x);
    node.put("y", _objectList[index].geometry.pose.y);
    node.put("z", _objectList[index].geometry.pose.z);
    node.put("roll", _objectList[index].geometry.roll);
    node.put("pitch",_objectList[index].geometry.pitch);
    node.put("yaw", _objectList[index].geometry.yaw);
}

void objectsInformation::addBoundingBox(ptree &node, int index){
    node.add(TAG_LENGTH, _objectList[index].geometry.length);
    node.add(TAG_WIDTH, _objectList[index].geometry.width);
    node.add(TAG_HEIGHT, _objectList[index].geometry.height);
}


QString objectsInformation::importObjectsInformation(QString xmlFile){
    ptree root;
    read_xml(xmlFile.toStdString(), root);
    QString scenarioType = QString::fromStdString(root.get<std::string>(TAG_SCENARIO + "." + "type"));
    ptree& tableDimensions = root.get_child(TAG_SCENARIO + "." + TAG_DIMENSIONS);
    _tableLength = tableDimensions.get<float>(TAG_LENGTH);
    _tableWidth = tableDimensions.get<float>(TAG_WIDTH);

    ptree& allObjects = root.get_child(TAG_SCENARIO + "." + TAG_ALLOBJECTS);

    ptree::iterator it = allObjects.begin();
    it++;
    for(; it != allObjects.end(); it++){
        parseObject(it->second);
    }

    return scenarioType;
}

bool objectsInformation::existsObject(QString objectName){
    bool exists = false;
    for(int i = 0; i < _objectList.size(); i++){
        if(!objectName.compare(_objectList[i].name))
            exists = true;
    }
    return exists;
}

QStringList objectsInformation::getListOfObjects(){
    QStringList list;
    for(int i = 0; i < _objectList.size(); i++){
        list +=_objectList[i].name;
    }
    return list;
}

void objectsInformation::parseObject(ptree &parent){
    object newObject;
    // Name and color
    newObject.name = QString::fromStdString(parent.get<std::string>(TAG_NAME));
    newObject.color = QString::fromStdString(parent.get<std::string>(TAG_COLOR));

    // Pose
    ptree& pose = parent.get_child(TAG_POSE);
    newObject.geometry.pose.x = pose.get<float>("x");
    newObject.geometry.pose.y = pose.get<float>("y");
    newObject.geometry.pose.z = pose.get<float>("z");
    newObject.geometry.roll = pose.get<float>("roll");
    newObject.geometry.pitch = pose.get<float>("pitch");
    newObject.geometry.yaw = pose.get<float>("yaw");

    // Dimensions bounding box
    ptree& dimensions = parent.get_child(TAG_DIMENSIONS);
    newObject.geometry.length = dimensions.get<float>(TAG_LENGTH);
    newObject.geometry.width = dimensions.get<float>(TAG_WIDTH);
    newObject.geometry.height = dimensions.get<float>(TAG_HEIGHT);

    // Get indices
    std::istringstream str(parent.get<std::string>(TAG_INDICES));
    newObject.indices.indices.clear();
    int i;
    while(str >> i){
        newObject.indices.indices.push_back(i);
    }
    _objectList.push_back(newObject);
}

void objectsInformation::getGeometry(int indexObject,
                                     pcl::PointXYZ &pose,
                                     float &roll,
                                     float &pitch,
                                     float &yaw,
                                     float &boxlength,
                                     float &boxWidth,
                                     float &boxHeight){

    pose.x = _objectList[indexObject].geometry.pose.x;
    pose.y = _objectList[indexObject].geometry.pose.y;
    pose.z = _objectList[indexObject].geometry.pose.z;
    roll = _objectList[indexObject].geometry.roll;
    pitch = _objectList[indexObject].geometry.pitch;
    yaw = _objectList[indexObject].geometry.yaw;
    boxlength = _objectList[indexObject].geometry.length;
    boxWidth = _objectList[indexObject].geometry.width;
    boxHeight = _objectList[indexObject].geometry.height;
}

void objectsInformation::modifyObject(QString name,
                                      pcl::PointXYZ pose,
                                      float length,
                                      float width,
                                      float height,
                                      float roll,
                                      float pitch,
                                      float yaw,
                                      pcl::PointIndices *indices){
    int indexObject = getIndex(name);
    _objectList[indexObject].geometry.pose.x = pose.x;
    _objectList[indexObject].geometry.pose.y = pose.y;
    _objectList[indexObject].geometry.pose.z = pose.z;
    _objectList[indexObject].geometry.roll = roll;
    _objectList[indexObject].geometry.pitch = pitch;
    _objectList[indexObject].geometry.yaw = yaw;
    _objectList[indexObject].geometry.length = length;
    _objectList[indexObject].geometry.width = width;
    _objectList[indexObject].geometry.height = height;
    _objectList[indexObject].indices.indices.clear();
    for(int i=0; i < indices->indices.size(); i++){
        _objectList[indexObject].indices.indices.push_back(indices->indices[i]);
    }
}
