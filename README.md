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
| SHITOMASI | 125 | 118 | 123 | 120 | 120 | 113 | 114 | 123 | 111 | 112 | 4
| HARRIS | 17 | 14 | 18 | 21 | 26 | 43 | 18 | 30 | 26 | 34 | 6
| FAST | 121 | 115 | 127 | 122 | 111 | 113 | 107 | 103 | 112 | 117 | 7
| BRISK | 264 | 282 | 282 | 277 | 297 | 279 | 289 | 272 | 266 | 254 | 21
| ORB | 92 | 102 | 106 | 113 | 109 | 125 | 130 | 129 | 127 | 128 | 57
| AKAZE | 166 | 157 | 161 | 155 | 163 | 164 | 173 | 175 | 177 | 179 | 7.8
| SIFT | 138 | 132 | 124 | 137 | 134 | 140 | 137 | 148 | 159 | 137 | 5.6

### MP.7 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

Some combinations of detector and descriptor doesn't make sense, those results are N/A.

| Detector,Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| SHITOMASI | 686 |922|866|688|N/A|900|
| HARRIS | 138|169 |164|134|N/A|167|
| FAST | 638 |805|831|645|N/A|763|
| BRISK | **1426** | **1512** |1379|1386|N/A| **1529** |
| ORB | 678 |486|691|395|N/A|742|
| AKAZE | 1020 |1169|1096|1002|1199|1176|
| SIFT | 491 |695|N/A|492|N/A|759|

### MP.7 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

Some combinations of detector and descriptor doesn't make sense, those results are N/A.

| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
| --- | --- | --- |--- |--- |--- |--- |
| SHITOMASI| 17.98 |21.38|18.8|52.4079|N/A| 31.82|
| HARRIS | 13.28|14.50 |14.24|32.07| N/A| 22.31|
| FAST| **3.98** | **3.72** | **3.12** |22.91|N/A|14.72|
| BRISK| 34.84 |33.42|40.72|54.59|N/A|54.73|
| ORB| 5.82 |6.91|11.12|22.52|N/A|23.83|
| AKAZE| 51.55|53.93 |57.92|75.42|93.73|67.89|
| SIFT| 68.54 |88.18|N/A|113.33|N/A|137.67|

The top 3 detector/descriptor combinations are found by evaluating the tables above.

In terms of number of matched keypoints (More is better)
  1. BRISK/SIFT
  2. BRISK/BRIEF
  3. BRISK/BRISK

In terms of execution time (Small is better)
  1. FAST/ORB
  2. FAST/BRIEF
  3. FAST/BRISK
