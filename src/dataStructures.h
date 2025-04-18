#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    

    float _neighbourhoodSize=-1.00; // size of the neighbourhood for keypoint detection   
    cv::Mat cameraImg; // camera image    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    float NeighbourhoodSize() { 
        
        if(keypoints.size()>0) 
        {
            _neighbourhoodSize=0;
            for (auto it = keypoints.begin(); it!= keypoints.end(); ++it) {
                _neighbourhoodSize += (*it).size;
            }
            _neighbourhoodSize /= keypoints.size();
        }
        return  _neighbourhoodSize; 
    } 
    int KeyPointsCount() { 
        
        return keypoints.size(); 
    }
};


#endif /* dataStructures_h */
