#ifndef QSR_H
#define QSR_H

#include <pcl/point_types.h>
#include "objectsinformation.h"

class qsr
{
public:
    qsr(std::vector<object> objectList);

    void calculateQSRRight();

    void calculateQSRLeft();

    void calculateQSRInFront();

    void calculateQSRBehind();

    QString getDescription();

    QString getAllValues();

private:
    pcl::PointXYZ getCenterOfMass(object obj);

    void calculatePointsLandmark(object landmark);

    void calculatePointTrajector(object trajector);

    void calculateAnglesAndDistanceRight();

    void calculateAnglesAndDistanceLeft();

    void calculateAnglesAndDistanceInFront();

    void calculateAnglesAndDistanceBehind();

    float angleFunction(float angle);

    float distanceFunction(float distance, float perimeter);

    float qsrRight();

    float qsrLeft();

    float qsrInFront();

    float qsrBehind();

    void getStringStream(float *values, std::stringstream& ss, std::string relation);

    // I need to explain what is the meaning of each point.
    pcl::PointXYZ _FLDPoint, _FRDPoint, _BRDPoint, _BLDPoint;
    pcl::PointXYZ _FCPoint, _RCPoint, _BCPoint, _LCPoint;
    pcl::PointXYZ _centerOfMassTrajector;

    float _angleRightRight, _angleLeftRight, _distanceRight;
    float _angleRightLeft, _angleLeftLeft, _distanceLeft;
    float _angleRightInFront, _angleLeftInFront, _distanceInFront;
    float _angleRightBehind, _angleLeftBehind, _distanceBehind;
    float _distanceThreshold;
    float _perimeterLandmark;

    std::vector<object> _objectList;
};

#endif // QSR_H
