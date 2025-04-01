/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <exception>
#include <stacktrace>
#include <execinfo.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"
#include "ImageHelper.cpp"

using namespace std;

bool bVis = false; // visualize results
bool bFocusOnVehicle = true; // only keep keypoints on the vehicle (assumes vehicle is in the center of the image)

int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
int imgEndIndex = 9;   // last file index to load
int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png) 
int dataBufferSize = 2; // no. of images which are held in memory (ring buffer) at the same time
ImageHelper imgHelper;
string detectorTypeArray[7] = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE","SIFT"}; 
string descriptorTypeArray[6] = {"BRISK", "BRIEF", "ORB", "FREAK","AKAZE","SIFT" };   

string keypointCountMsg;
string matchedKeypointCountMsg;
string executionTimeMsg;

string keypointCountMsgFinal;
string matchedKeypointCountMsgFinal;
string executionTimeMsgFinal; 

vector<cv::KeyPoint> GetKeypoints(DataFrame &frame,string detectorType)
{
    vector<cv::KeyPoint> keypoints; // create empty feature list for current image   
    if (detectorType.compare("SHITOMASI") == 0)
    {
        detKeypointsShiTomasi(keypoints, frame.cameraImg, bVis);
    }
    else if (detectorType.compare("HARRIS") == 0)
    {
        detKeypointsHarris(keypoints, frame.cameraImg, bVis);
    }
    else
    {
        detKeypointsModern(keypoints, frame.cameraImg, detectorType, bVis);
    }

    // Only keypoints inside vehicleRect            
    cv::Rect vehicleRect(535, 180, 180, 150);

    if (bFocusOnVehicle)
    {
        
        std::vector<cv::KeyPoint> filtered_keypoints;
        
        for (auto it=keypoints.begin(); it != keypoints.end(); it++ ) 
        {
            if (vehicleRect.contains(it->pt)) {                 
                filtered_keypoints.push_back(*it);
            }
        }
        
        keypoints = filtered_keypoints;
    }

    
    // optional : limit number of keypoints (helpful for debugging and learning)
    bool bLimitKpts = false;
    if (bLimitKpts)
    {
        int maxKeypoints = 50;

        if (detectorType.compare("SHITOMASI") == 0)
        { // there is no response info, so keep the first 50 as they are sorted in descending quality order
            keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
        }
        cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
        
    }
    return keypoints;

}
vector<cv::DMatch>  GetMatches(vector<DataFrame> &dataBuffer,string descriptor)
{
    /* MATCH KEYPOINT DESCRIPTORS */

    vector<cv::DMatch> matches;
    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
    string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
    string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN   

    if(descriptor=="SIFT")
    {
        matcherType = "MAT_FLANN";           
        selectorType = "SEL_KNN";      
    }

    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                    matches, descriptorType, matcherType, selectorType);

       
    if (bVis)
    {
        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                        matches, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = "Matching keypoints between two camera images";
        cv::namedWindow(windowName, 7);
        cv::imshow(windowName, matchImg);
        cout << "Press key to continue to next image" << endl;
        cv::waitKey(0); // wait for key to be pressed
    } 
    return matches;
}
void run ()
{    
  
    for (std::string detectorType : detectorTypeArray) 
    {        
        std::cout << "Detector Type: " << detectorType << std::endl;      
         
        matchedKeypointCountMsg = "|"+ detectorType;
        executionTimeMsg = "|"+detectorType;
        for (std::string descriptorType : descriptorTypeArray) 
        {          
            auto start = std::chrono::high_resolution_clock::now();      
           try
           {                 
                
                if(detectorType != "AKAZE" && descriptorType == "AKAZE")
                {
                    //std::cout << "Skipping - Detector/Descriptor: " << detectorType <<"/"<< descriptorType << std::endl;    
                    matchedKeypointCountMsg += "|N/A ";                 
                }
                else if(detectorType == "SIFT" && descriptorType == "ORB")
                {
                    //std::cout << "Skipping - SIFT/ORB" << std::endl;  
                    matchedKeypointCountMsg += "|N/A ";                     
                }                
                else
                {            
                    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time                  
                    keypointCountMsg =  "|"+ detectorType;
                    int matchPointCount=0;
                    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
                    {    
                        DataFrame frame; 
                        // push image into data frame buffer   
                        frame.cameraImg =imgHelper.GetGrayImage(imgIndex); 
                        if (dataBuffer.size() < dataBufferSize) 
                        {
                            dataBuffer.push_back(frame);
                        }
                        else
                        {
                            // ring buffer  - if the buffer is full, remove the oldest frame and add the new frame to the end of the buffer
                            dataBuffer.erase(dataBuffer.begin());
                            dataBuffer.push_back(frame);
                        }  

                        // Extract 2D keypoints from current image and   push keypoints and descriptor for current frame to end of data buffer
                        (dataBuffer.end() - 1)->keypoints = GetKeypoints(frame,detectorType);  
                        keypointCountMsg += "|" + std::to_string((dataBuffer.end() - 1)->KeyPointsCount()) + " ";
                        
                        cv::Mat descriptors;
                        /* EXTRACT KEYPOINT DESCRIPTORS */        
                        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType); 
                        // push descriptors for current frame to end of data buffer
                        (dataBuffer.end() - 1)->descriptors = descriptors;                  
                        if (dataBuffer.size() > 1) // wait until at least two images have been processed
                        {
                            (dataBuffer.end() - 1)->kptMatches = GetMatches(dataBuffer,descriptorType);
                            matchPointCount += (dataBuffer.end() - 1)->kptMatches.size();
                        }       
                        
                    }// eof loop over all images   
                    keypointCountMsg += "|" + std::to_string((dataBuffer.end() - 1)->NeighbourhoodSize()) + " ";
                    matchedKeypointCountMsg +="|"+ std::to_string(matchPointCount) + " ";
                } 
               
            }
            catch(const std::exception& e)
            {
             std::cerr << e.what() << '\n';
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            executionTimeMsg +=  "|" + (duration.count()==0? "N/A": std::to_string(duration.count())) + " ";
           
        } // eof loop over all descriptor   
        keypointCountMsgFinal+=keypointCountMsg +'\n';
        matchedKeypointCountMsgFinal+=matchedKeypointCountMsg +'\n';
        executionTimeMsgFinal+=executionTimeMsg +'\n';
        
    } // eof loop over all detectors    

    keypointCountMsgFinal = "| Detector | image0 | image1 | image2 | image3 | image4 | image5 | image6 | image7 | image8 | image9 | Neighborhood size |\n| :---:    | :---:  | :---:  | :---:  |  :---: | :---:  | :---:  | :---:  | :---:  | :---:  | :---:  | :---: | \n" + keypointCountMsgFinal;
    std::cout <<  keypointCountMsgFinal << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;
    
    matchedKeypointCountMsgFinal = "| Detector,Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |\n| --- | --- | --- |--- |--- |--- |--- | \n" + matchedKeypointCountMsgFinal;
    std::cout <<  matchedKeypointCountMsgFinal << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;

    executionTimeMsgFinal = "| Detector,Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |\n| --- | --- | --- |--- |--- |--- |--- | \n" + executionTimeMsgFinal;
    std::cout <<  executionTimeMsgFinal << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{ 
    imgHelper = ImageHelper();
    run(); 
    return 0;
    
}


