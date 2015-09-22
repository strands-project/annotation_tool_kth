#ifndef RGBD_GRABBER_HH
#define RGBD_GRABBER_HH

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "ros/ros.h"

#include <string>

#include <iostream>
#include <fstream>
#include <deque>
#include <queue>

#include <cv_bridge/cv_bridge.h>

struct Frame{
    double m_Timestamp;
    bool m_RGB;
    cv_bridge::CvImagePtr m_Image;

    Frame(const Frame&);
    Frame()
    {

    }

};



class RGBDGrabber
{
public:




    RGBDGrabber();
    ~RGBDGrabber();

    void pointCloudCallback (const sensor_msgs::PointCloud2::ConstPtr& msg);
    void colorImageCallback (const sensor_msgs::Image::ConstPtr& img);
    void depthImageCallback (const sensor_msgs::Image::ConstPtr& img);

    void setSaveMismatchedImages(const bool&);
    void setSaveOneFrame(const bool&);
    void setSaveFrameSeq(const bool&);
    void setFrameSkip(const int&);
    int  getFrameSkip();
    void increaseFrameSkip();
    void decreaseFrameSkip();
    std::string getFolderName();

    bool createNewFolder();


    static std::string getDateTime();

private:

    void                                        syncAndSave();

    volatile bool                               m_bSaveOneFrame, m_bSaveFrameSequence, m_bCreateCVWindow, m_bLensCovered, m_bSaveMismatchedImages;
    int                                         m_iSequenceNumber;
    int                                         m_iFrameSkip;
    std::string                                 m_sCurrentFolder;
    std::ofstream                               m_fIndexFile;
    double                                      m_dLastTimestamp;
    std::deque<std::pair<int, double> >         m_DepthSequenceAndTimestamp;
    std::deque<std::pair<int, double> >         m_RGBSequenceAndTimestamp;

    std::priority_queue<Frame, std::vector<Frame>, std::greater<Frame> > m_FrameBuffer;

    std::deque<std::pair<cv_bridge::CvImagePtr, double> >         m_DepthImageAndTimestamp;
    std::deque<std::pair<cv_bridge::CvImagePtr, double> >         m_RGBImageAndTimestamp;

    int                                         m_iImageSyncQueueLength;
    double                                      m_dImageSyncTimout; // im seconds
    double                                      m_dImageSyncBetweenFrames; // im seconds
    double                                      m_dAvgPixelValueWhenLensCovered;
    int                                         m_iDarkDepthImageThreshold;
    int                                         m_iCurrentDarkDepthImageCount;
};




#endif // DATA_COMPRESION_NODE_HH
