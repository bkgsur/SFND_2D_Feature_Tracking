# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1
 * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
 * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Writeup
### MP.1 Data Buffer Optimization
The implemention removes the oldest frame and add the new frame to the end of the buffer, if it buffer size is bigger than the size expected.

### MP.2 Keypoint Detection
The HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT keypoint detectors were instaniated based on selection.

### MP.3 Keypoint Removal
The `.contains` method of the VehicleRect object is used to detect and remove keypoints outside of the rectangle.

### MP.4 Keypoint Descriptors
The BRIEF, ORB, FREAK, AKAZE and SIFT descriptors were implemented. The methods were made selectable by changing the string's definition.

### MP.5 Descriptor Matching
The FLANN matcher and kNN matchers are implemented. The methods were made selectable by changing the string's definition

### MP.6 Descriptor Distance Ratio
Lowe's distance ratio test was implemented by comparing the best and second best matches to decide if a pair of matched keypoints should be stored.

### MP.7 Performance Evaluation 1
Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

| Detector | image0 | image1 | image2 | image3 | image4 | image5 | image6 | image7 | image8 | image9 | Neighborhood size |
| :---:    | :---:  | :---:  | :---:  |  :---: | :---:  | :---:  | :---:  | :---:  | :---:  | :---:  | :---: | 
|SHITOMASI|125 |118 |123 |120 |120 |113 |114 |123 |111 |112 |4.000000 
|HARRIS|10 |13 |17 |17 |20 |18 |17 |25 |20 |19 |6.000000 
|FAST|121 |115 |127 |122 |111 |113 |107 |103 |112 |117 |7.000000 
|BRISK|264 |282 |282 |277 |297 |279 |289 |272 |267 |254 |22.035887 
|ORB|92 |102 |106 |113 |109 |125 |130 |129 |127 |128 |54.388531 
|AKAZE|166 |157 |161 |155 |163 |164 |173 |175 |177 |179 |7.885763 
|SIFT|138 |132 |124 |137 |134 |140 |137 |148 |159 |137 |5.625093 


### MP.7 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

Some combinations of detector and descriptor doesn't make sense, those results are N/A.

| Detector,Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- | 
|SHITOMASI|1067 |1067 |1067 |1067 |N/A |926 
|HARRIS|157 |157 |157 |157 |N/A |147 
|FAST|1031 |1031 |1031 |1031 |N/A |825 
|BRISK|2509 |2509 |2509 |2327 |N/A |1665 
|ORB|950 |1033 |1033 |549 |N/A |765 
|AKAZE|1491 |1491 |1491 |1491 |1491 |1282 
|SIFT|1248 |1249 |N/A |1239 |N/A |804 


### MP.7 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

Some combinations of detector and descriptor doesn't make sense, those results are N/A.
Time in milliseconds.
| Detector,Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- | 
|SHITOMASI|1067 |1067 |1067 |1067 |N/A |926 
|HARRIS|157 |157 |157 |157 |N/A |147 
|FAST|1031 |1031 |1031 |1031 |N/A |825 
|BRISK|2509 |2509 |2509 |2327 |N/A |1665 
|ORB|950 |1033 |1033 |549 |N/A |765 
|AKAZE|1491 |1491 |1491 |1491 |1491 |1282 
|SIFT|1248 |1249 |N/A |1239 |N/A |804 


The top 3 detector/descriptor combinations are found by evaluating the tables above.

In terms of number of matched keypoints (More is better)
  1. BRISK/BRISK
  2. BRISK/BRIEF
  3. BRISK/ORB

In terms of execution time (Small is better)
  1. SHITOMASI/BRIEF
  2. SHITOMASI/ORB
  3. HARRIS/BRIEF
