#include "rgbd_grabber.h"

#include <sstream>
#include <stdio.h>
#include <time.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

bool operator>(const Frame& rhs, const Frame& lhs)
{
    return lhs.m_Timestamp <rhs.m_Timestamp;
}

Frame::Frame(const Frame& cpy)
{
    this->m_Timestamp = cpy.m_Timestamp;
    this->m_RGB = cpy.m_RGB;
    this->m_Image = cpy.m_Image;
}

RGBDGrabber::RGBDGrabber() : m_bSaveOneFrame(false), m_bSaveFrameSequence(false), m_bCreateCVWindow(false), m_bLensCovered(true),
    m_iSequenceNumber(0), m_dLastTimestamp(0.0), m_iImageSyncQueueLength(20), m_dImageSyncTimout(2.0), m_dImageSyncBetweenFrames(0.01),
    m_iFrameSkip(0), m_bSaveMismatchedImages(true)
{
    bool folderCreated = createNewFolder(); // to initialize m_sCurrentFolder (also to actually create the folder)
    if (!folderCreated)
    {
        m_sCurrentFolder = "";
        m_fIndexFile.open("index.txt");
    }

    m_dAvgPixelValueWhenLensCovered = 100;
    m_iDarkDepthImageThreshold = 20;
    m_iCurrentDarkDepthImageCount = 0;

    cout.precision(15);
}

//---------------------------------------------------------------------------------------------
void RGBDGrabber::pointCloudCallback (const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::cout << "New colored point cloud received " << std::endl;

}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::colorImageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
//    std::cout << "New color camera image received " << std::endl;

    if ((m_bSaveFrameSequence || m_bSaveOneFrame) && (!m_bLensCovered))
    {
        // save this frame
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(imgMsg, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // identify the sequence number of this frame by matching its timestamp with the timestamps of previously saved Depth images
        double currentTimestamp = imgMsg->header.stamp.toSec();
        m_RGBImageAndTimestamp.push_front(std::make_pair(cv_ptr, currentTimestamp));        

        if (m_bSaveOneFrame)
        {
            char buffer[50];
            sprintf(buffer,"RGB%010d.png",m_iSequenceNumber);

            string completeName = m_sCurrentFolder + string("/") + string(buffer);

            imwrite(completeName.c_str(),cv_ptr->image);
            m_fIndexFile<<setprecision(15)<<buffer<<" "<<imgMsg->header.stamp<<"\n";

            std::cout<<"RGBDGrabber :: saved color file "<<buffer<<"   time stamp  "<<imgMsg->header.stamp<<std::endl;
            m_bSaveOneFrame = false;

            m_iSequenceNumber++;
        } else {
            Frame newFrame;
            newFrame.m_Timestamp = currentTimestamp;
            newFrame.m_RGB = true;
            newFrame.m_Image = cv_ptr;
            m_FrameBuffer.push(newFrame);

            syncAndSave();
        }
    }
}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::depthImageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{
//    std::cout << "New depth camera image received " << std::endl;

    if (m_bSaveFrameSequence || m_bSaveOneFrame)
    {
        // save this frame
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(imgMsg, "16UC1");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv_ptr->image*=16;


        // compute the pixel average, in order to detect whether the lens is covered or not
        double avg = 0.0;
        for (size_t i=0; i<cv_ptr->image.rows;i++)
            for (size_t j=0; j<cv_ptr->image.cols;j++)
            {
                avg+=cv_ptr->image.at<uint16_t>(i,j);
            }

//        cout<<"Depth image average "<<avg/(cv_ptr->image.rows*cv_ptr->image.cols)<<endl;


        avg/=(cv_ptr->image.rows*cv_ptr->image.cols);

        if (avg<m_dAvgPixelValueWhenLensCovered)
        {
            m_iCurrentDarkDepthImageCount++;
            if (m_iCurrentDarkDepthImageCount > m_iDarkDepthImageThreshold)
            {
                m_bLensCovered = true;
                // check whether we need to create a new folder
                cout<<"Lens covered; skipping image."<<endl;
                if (m_bSaveFrameSequence && (m_iSequenceNumber != 0))
                {
                    createNewFolder(); // this also sets m_iSequenceNumber to 0
                    cout<<"Lens covered; skipping image. Creating new folder."<<endl;
                }
                return;
            } else {
                // return without saving this image (no depth)
                return;
            }
        } else {
            m_iCurrentDarkDepthImageCount = 0;
            m_bLensCovered = false;
        }

        double currentTimestamp = imgMsg->header.stamp.toSec();

        if (m_bSaveOneFrame)
        {
            char buffer[50];
            sprintf(buffer,"Depth%010d.png",m_iSequenceNumber);
            string completeName = m_sCurrentFolder + string("/") + string(buffer);
            m_fIndexFile<<setprecision(15)<<buffer<<" "<<currentTimestamp<<"\n";

            imwrite(completeName.c_str(),cv_ptr->image);
            std::cout<<"RGBDGrabber :: saved depth file "<<buffer<<"   time stamp  "<<currentTimestamp<<std::endl;

            /*************** TEST SAVING **************************/
//            cv::Mat savedImage;
//            savedImage = cv::imread(completeName.c_str(),-1);
//            imshow("Input",cv_ptr->image);
//            imshow("Output",savedImage);
//            cv::waitKey(0);

            m_bSaveOneFrame = false;
            m_iSequenceNumber++;
        } else {
            Frame newFrame;
            newFrame.m_Timestamp = currentTimestamp;
            newFrame.m_RGB = false;
            newFrame.m_Image = cv_ptr;
            m_FrameBuffer.push(newFrame);

            syncAndSave();
        }

    }

}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::syncAndSave()
{
    if (m_FrameBuffer.size()>m_iImageSyncQueueLength*0.75)
    {
        while (m_FrameBuffer.size()>m_iImageSyncQueueLength*0.25)
        {
            Frame currentFrame = m_FrameBuffer.top();
            m_FrameBuffer.pop();
//            cout<<"Current frame "<<currentFrame.m_Timestamp<<" queue size "<<m_FrameBuffer.size()<<" type "<<currentFrame.m_RGB<<endl;
            Frame nextFrame = m_FrameBuffer.top();
            double tsDiff = abs(currentFrame.m_Timestamp - nextFrame.m_Timestamp);
            m_FrameBuffer.pop();
            Frame nextnextFrame = m_FrameBuffer.top();
            double nexttsDiff = abs(nextnextFrame.m_Timestamp - nextFrame.m_Timestamp);

            if (currentFrame.m_RGB == nextFrame.m_RGB || (tsDiff>m_dImageSyncTimout) || (nexttsDiff<tsDiff))
            {
                // same image type -> currentFrame doesn't have a match, save it directly
                m_FrameBuffer.push(nextFrame); // this was popped to see what comes after

                if ((m_iSequenceNumber%(m_iFrameSkip+1) == 0) && (m_bSaveMismatchedImages))
                {
                    char buffer[50];
                    if (currentFrame.m_RGB)
                    {
                        sprintf(buffer,"RGB%010d.png",m_iSequenceNumber);
                    }  else
                    {
                        sprintf(buffer,"Depth%010d.png",m_iSequenceNumber);
                    }

                    m_fIndexFile<<setprecision(15)<<buffer<<" "<<currentFrame.m_Timestamp<<"\n";
                    string completeName = m_sCurrentFolder + string("/") + string(buffer);
                    if (currentFrame.m_RGB)
                    {
                        cv::imwrite(completeName.c_str(),currentFrame.m_Image->image);
                        std::cout<<"RGBDGrabber :: saved color file "<<buffer<<"   time stamp  "<<currentFrame.m_Timestamp<<std::endl;
                    } else {
                        imwrite(completeName.c_str(),currentFrame.m_Image->image);
                        std::cout<<"RGBDGrabber :: saved depth file "<<buffer<<"   time stamp  "<<currentFrame.m_Timestamp<<std::endl;
                    }
                }
            } else {
                char buffer1[50], buffer2[50];

                if (m_iSequenceNumber%(m_iFrameSkip+1) == 0)
                {
                    if (currentFrame.m_RGB)
                    {
                        sprintf(buffer1,"RGB%010d.png",m_iSequenceNumber);
                        sprintf(buffer2,"Depth%010d.png",m_iSequenceNumber);
                    }  else
                    {
                        sprintf(buffer1,"Depth%010d.png",m_iSequenceNumber);
                        sprintf(buffer2,"RGB%010d.png",m_iSequenceNumber);
                    }

                    m_fIndexFile<<setprecision(15)<<buffer1<<" "<<currentFrame.m_Timestamp<<"\n";
                    m_fIndexFile<<setprecision(15)<<buffer2<<" "<<nextFrame.m_Timestamp<<"\n";
                    string completeName1 = m_sCurrentFolder + string("/") + string(buffer1);
                    string completeName2 = m_sCurrentFolder + string("/") + string(buffer2);
                    if (currentFrame.m_RGB)
                    {
                        imwrite(completeName1.c_str(),currentFrame.m_Image->image);
                        std::cout<<"RGBDGrabber :: saved color file "<<buffer1<<"   time stamp  "<<currentFrame.m_Timestamp<<std::endl;

                        imwrite(completeName2.c_str(),nextFrame.m_Image->image);
                        std::cout<<"RGBDGrabber :: saved depth file "<<buffer2<<"   time stamp  "<<nextFrame.m_Timestamp<<std::endl;
                    } else {
                        imwrite(completeName1.c_str(),currentFrame.m_Image->image);
                        std::cout<<"RGBDGrabber :: saved depth file "<<buffer1<<"   time stamp  "<<currentFrame.m_Timestamp<<std::endl;

                        imwrite(completeName2.c_str(),nextFrame.m_Image->image);
                        std::cout<<"RGBDGrabber :: saved color file "<<buffer2<<"   time stamp  "<<nextFrame.m_Timestamp<<std::endl;
                    }
                }

            }
            m_iSequenceNumber++;
        }
    }

}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::setSaveOneFrame(const bool& saveOneFrame)
{
    m_bSaveOneFrame = saveOneFrame;
}

//---------------------------------------------------------------------------------------------
void RGBDGrabber::setSaveFrameSeq(const bool& saveFrameSeq)
{
    m_bSaveFrameSequence = saveFrameSeq;
    m_FrameBuffer = std::priority_queue<Frame, std::vector<Frame>, std::greater<Frame> >();
}
//---------------------------------------------------------------------------------------------
RGBDGrabber::~RGBDGrabber()
{

}
//---------------------------------------------------------------------------------------------
bool RGBDGrabber::createNewFolder()
{
    string newFolderName = RGBDGrabber::getDateTime();

    struct stat st = {0};

    if (stat(newFolderName.c_str(), &st) == -1) {
        // create new folder
        mkdir(newFolderName.c_str(), 0700);
        m_sCurrentFolder = newFolderName;
        m_iSequenceNumber = 0; // reset

        // create new index file
        string indexFilename = m_sCurrentFolder+string("/")+string("index.txt");
        if (m_fIndexFile.is_open())
        {
            m_fIndexFile.close();
        }
        m_fIndexFile.open(indexFilename.c_str());

        return true;
    }

    return false;
}
std::string RGBDGrabber::getFolderName()
{
    return m_sCurrentFolder;
}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::setFrameSkip(const int& fs)
{
    if (fs >= 0)
    {
        m_iFrameSkip = fs;
    } else {
        cout<<"Cannot set frameskip as the value provided is negative "<<fs<<endl;
    }
}
//---------------------------------------------------------------------------------------------
std::string RGBDGrabber::getDateTime()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
//---------------------------------------------------------------------------------------------
int  RGBDGrabber::getFrameSkip()
{
    return m_iFrameSkip;
}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::increaseFrameSkip()
{
    m_iFrameSkip++;
}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::decreaseFrameSkip()
{
    if (m_iFrameSkip > 0)
    {
        m_iFrameSkip--;
    }
}
//---------------------------------------------------------------------------------------------
void RGBDGrabber::setSaveMismatchedImages(const bool& saveMismatched)
{
    m_bSaveMismatchedImages = saveMismatched;
}
