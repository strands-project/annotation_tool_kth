#ifndef OBJECTSINFORMATION_H
#define OBJECTSINFORMATION_H

#include "pcl/point_types.h"
#include <QString>
#include <boost/property_tree/ptree.hpp>
#include <pcl/filters/passthrough.h>


// The object geometry
class objectGeometry{
public:
    pcl::PointXYZ pose;
    float roll;
    float pitch;
    float yaw;
    float length;
    float width;
    float height;
};

// Object class
class object{
public:
    QString name;
    objectGeometry geometry;
    pcl::PointIndices indices;
    QString color;
};

class objectsInformation
{
public:
    objectsInformation();

    /** \brief Insert an object in the list of objects
      * \param[in] name Name of the object
      * \param[in] pose Pose of the bounding box
      * \param[in] length Length of the bounding box
      * \param[in] width Width of the bounding box
      * \param[in] height Height of the bounding box
      * \param[in] roll Roll angle of the bounding box
      * \param[in] pitch Pitch angle of the bounding box
      * \param[in] yaw Yaw angle of the bounding box
      * \param[in] indices Indices of the points belonging to the object
      */
    void insertObject(QString name,
                      pcl::PointXYZ pose,
                      float length,
                      float width,
                      float height,
                      float roll,
                      float pitch,
                      float yaw,
                      pcl::PointIndices *indices);

    /** \brief Deletes an object in the list of objects
      * \param[in] objectName Name of the object to be deleted
      */
    void deleteObject(QString objectName);

    /** \brief Returns the index corresponding to the object in the objects' list if it's found, if not return -1
      * \param[in] objectName Name of the object to find
      */
    int getIndex(QString objectName);

    /** \brief Get the point indices of and object
      * \param[in] indexObject The index of the object in the object list
      * \param[out] indices The indices of the object
      */
    void getIndices(int indexObject, pcl::PointIndices *indices);

    /** \brief Get all the values related with the current object's bounding box
      * \param[in] indexObject The index of the object to obtain the data
      * \param[out] pose The pose of the bounding box
      * \param[out] roll The roll angle of the bounding box
      * \param[out] pitch The pitch angle of the bounding box
      * \param[out] yaw The yaw angle of the bounding box
      * \param[out] boxlength The length of the bounding box
      * \param[out] boxWidth The width of the bounding box
      * \param[out] boxHeight The height of the bounding box
      */
    void getGeometry(int indexObject,
                     pcl::PointXYZ &pose,
                     float &roll,
                     float &pitch,
                     float &yaw,
                     float &boxlength,
                     float &boxWidth,
                     float &boxHeight);

    /** \brief Returns the length of the desk
      */
    float getDeskLength(){
        return _tableLength;
    }

    /** \brief Returns the width of the desk
      */
    float getDeskWidth(){
        return _tableWidth;
    }

    /** \brief Set the dimensions of the table
      * \param[in] length Table length
      * \param[in] width Table width
      */
    void setDeskDimensions(float length, float width);

    /** \brief Get the list of the objects
      */
    std::vector<object> getObjectList(){
        return _objectList;
    }

    /** \brief Returns the number of objects in the object list
      */
    int numberOfObjects(){
        return _objectList.size();
    }

    /** \brief Remove all the objects in the object list
      */
    void clear();

    /** \brief Return the name of the object
      * \param[in] index Position of the object in the objects' list
      */
    QString nameOfObject(int index);

    /** \brief Export the objects' information in a .xml file
      * \param[in] file The name of the file to export
      */
    void exportObjectsInformation(QString xmlFile, QString pcdFile, QString scenario);

    /** \brief Import the objects' information loading a .xml file and returns de type of scenario loaded
      * \param[in] file The name of the file to be imported
      */
    QString importObjectsInformation(QString file);

    /** \brief Returns true if the object exists, else otherwise
      * \param[in] object Name of the object to check
      */
    bool existsObject(QString objectName);

    /** \brief Modifies and existing object
      * \param[in] name Name of the object
      * \param[in] pose Pose of the bounding box
      * \param[in] length Length of the bounding box
      * \param[in] width Width of the bounding box
      * \param[in] height Height of the bounding box
      * \param[in] roll Roll angle of the bounding box
      * \param[in] pitch Pitch angle of the bounding box
      * \param[in] yaw Yaw angle of the bounding box
      * \param[in] indices Indices of the points belonging to the object
      */
    void modifyObject(QString name,
                      pcl::PointXYZ pose,
                      float length,
                      float width,
                      float height,
                      float roll,
                      float pitch,
                      float yaw,
                      pcl::PointIndices *indices);

    /** \brief Get the list of all the names in the objects' list
      */
    QStringList getListOfObjects();

private:
    // The next four functions are used to write and read a .xml file
    void addObject(boost::property_tree::ptree &parent, int index);
    void addBoundingBox(boost::property_tree::ptree &node, int index);
    void addPose(boost::property_tree::ptree &node, int index);
    void parseObject(boost::property_tree::ptree &parent);

    // The objects' list
    std::vector<object> _objectList;

    // The name of the colors used to show the objects
    std::vector<std::string> _nameColors;

    // Table dimensions
    float _tableLength, _tableWidth;
};

#endif // OBJECTSINFORMATION_H
