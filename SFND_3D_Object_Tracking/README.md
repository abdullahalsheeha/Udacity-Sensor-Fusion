# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.


In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Steps 
### FP.1 : Match 3D Objects
In this task, please implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property)“. Matches must be the ones with the highest number of keypoint correspondences.

```c++
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
  	multimap<int,int> boxmap{};
    for (auto match: matches)
    {
      cv::KeyPoint currPoints = currFrame.keypoints[match.trainIdx];
      cv::KeyPoint prevPoints = prevFrame.keypoints[match.queryIdx];
      int currBoxId = 0;
      int prevBoxId = 0;
      
      for (auto bbox: currFrame.boundingBoxes)
      {
        if (bbox.roi.contains(currPoints.pt))
            {
             currBoxId = bbox.boxID;
        }
      }
      for (auto bbox: prevFrame.boundingBoxes)
      {
        if (bbox.roi.contains(prevPoints.pt))
            {
             prevBoxId = bbox.boxID;
        }
      }
      boxmap.insert({currBoxId, prevBoxId});
//       std::cout << "currBoxId: " << currBoxId << "prevBoxId: " << prevBoxId << std::endl;
    }
  	int prevBoxSize = prevFrame.boundingBoxes.size();
    // find the best matched previous boundingbox for each current boudingbox
    for(int i=0;i<prevBoxSize;++i){
        auto boxmapPair=boxmap.equal_range(i);
        vector<int> currBoxCount(prevBoxSize,0);
        for(auto pr=boxmapPair.first;pr!=boxmapPair.second;++pr){
            if((*pr).second>=0) currBoxCount[(*pr).second]+=1;
        }
        // find the position of best prev box which has highest number of keypoint correspondences.
        int maxPosition = std::distance(currBoxCount.begin(), std::max_element(currBoxCount.begin(), currBoxCount.end()));
        bbBestMatches.insert({maxPosition,i});
    }
  
  
}
```

### FP.2 : Compute Lidar-based TTC

compute the time-to-collision for all matched 3D objects based on Lidar measurements from the matched bounding boxes between current and previous frame. I used the median of x-distance to reduce the impact of outliers of lidar points.
```c++
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dT = 1/frameRate;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane
	std::vector<double> prev_x, curr_x;
    
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            prev_x.push_back(it->x);
        }
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
      if (abs(it->y) <= laneWidth / 2.0)
        {
        curr_x.push_back(it->x);
      }
    }
  
  	std::sort(prev_x.begin(),prev_x.end());
    long medIndex = floor(prev_x.size() / 2.0);
    double XPrev = prev_x.size() % 2 == 0 ? (prev_x[medIndex - 1] + prev_x[medIndex]) / 2.0 : prev_x[medIndex];

    std::sort(curr_x.begin(),curr_x.end());
    long medInd = floor(curr_x.size() / 2.0);
    double XCurr = curr_x.size() % 2 == 0 ? (curr_x[medInd - 1] + curr_x[medInd]) / 2.0 : curr_x[medInd];
    
    // compute TTC from both measurements
    TTC = XCurr * dT / (XPrev - XCurr);
}

```

### FP.3 Associate Keypoint Correspondences with Bounding Boxes

Implemented the method: `clusterKptMatchesWithROI()` which associates keypoints with the bounding box that encloses them. All matches between current/previous frame are pushed to a vector within the respective bounding box.

```c++
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<double> euclideanDistance;
  
  	for (auto match: kptMatches)
    {
      if (boundingBox.roi.contains(kptsCurr.at(match.trainIdx).pt))
      {
        euclideanDistance.push_back(cv::norm(kptsCurr.at(match.trainIdx).pt - kptsPrev.at(match.queryIdx).pt));
      }
    }
  
    
    double euclideanDistanceMean = std::accumulate(euclideanDistance.begin(), euclideanDistance.end(), 0.0) / euclideanDistance.size();

    for(auto match: kptMatches)
    {

        if (boundingBox.roi.contains(kptsCurr.at(match.trainIdx).pt))
        {
            double temp = cv::norm(kptsCurr.at(match.trainIdx).pt - kptsPrev.at(match.queryIdx).pt);

            double euclideanDistanceMean_Augment = euclideanDistanceMean * 1.3;
            if(temp < euclideanDistanceMean_Augment)
            {
                boundingBox.keypoints.push_back(kptsCurr.at(match.trainIdx));
                boundingBox.kptMatches.push_back(match);
            }
        }
    }
}


```

### FP.4 Compute Camera-based TTC

Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame. I used the median of distance Ratios to reduce the impact of outliers of keypoints. To note, the constant-velocity-model was used for TTC calculation.

```c++
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
  std::sort(distRatios.begin(), distRatios.end());
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
// 	cout << distRatios.size()/2 << endl;
  	double medianDistRatio = distRatios[distRatios.size()/2];
  	
  	long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];
    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}
```

### FP5: Performance Evaluation

If you see the consencutive pictures by manually, you can catch that front vehicle is getting closer to camera and Lidar. Lidar TTC suddenly goes up to 16 seconds. Maybe the reason for that is...

1- Sensor erroneous (ex - light reflection)
2- Velocity of front vehicle goes up actually (since this TTC model is based on Constant velocity model)

### FP.6 : Performance Evaluation 2

| Detector/Descriptor | BRISK   | BRIEF   | ORB     | FREAK   | AKAZE   |  SIFT   |
| ------------------- | ------- | ------- | ------- | ------- | ------- | ------- |
| SHITOMASI           | 12.1862 | 12.2931 | 11.8945 | 12.1063 | n/a     | 11.8684 |
| HARRIS              | 50.2428 | 14.6938 | 11.8230 | 13.2377 | n/a     | 52.2985 |
| FAST                | 14.0946 | 12.3699 | 12.8516 | 13.7727 | n/a     | 12.5366 |
| BRISK               | 13.3382 | 14.6678 | 13.6681 | 14.1093 | n/a     | 14.2372 |
| ORB                 | 74.4193 | 17.2307 | 20.3159 | 17.9876 | n/a     | 29.6350 |
| AKAZE               | 11.9969 | 12.0047 | 12.1279 | 11.8781 | 12.2313 | 12.1403 |
|  SIFT               | 11.9195 | 12.0494 | n/a     | 12.3924 | n/a     | 12.5008 |


### Results

<img src="https://github.com/abdullahalsheeha/Udacity-Sensor-Fusion/blob/main/SFND_3D_Object_Tracking/results/FinalResults.jpg" width="1000" height="400" />
<img src="https://github.com/abdullahalsheeha/Udacity-Sensor-Fusion/blob/main/SFND_3D_Object_Tracking/results/3D.jpg" width="1000" height="400" />
