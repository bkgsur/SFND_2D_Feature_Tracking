#include <stdio.h>
#include <iostream>
#include <opencv2/core.hpp>
using namespace std;
class ImageHelper 
{
    string dataPath = "../";
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    public:
    ImageHelper() 
    {
        // Constructor
        // Initialize any member variables if needed
    }
    ~ImageHelper() 
    {
        // Destructor
        // Clean up any resources if needed
    }
    cv::Mat GetGrayImage(int imgIndex)
    {
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(4) << imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
        cv::Mat img = cv::imread(imgFullFilename);
        cv::Mat imgGray;   
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        return imgGray;
    }
};