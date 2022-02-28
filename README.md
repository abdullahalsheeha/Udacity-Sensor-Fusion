# Udacity-Sensor-Fusion

This repository houses my solutions for projects completed as part of Udacity's Sensor fusion Nanodegree.

## Projects
### Lidar Obstacle Detection
This project involved processing point cloud data using the C++ library, PCL. The data was first segemented using a linear Ransac model to determine which parts were part of the road, and which were not. KD-Trees were then used to conduct nearest neigbour search for clustering. A bounding box could then be drawn around the points - showing the location of vehicles. 

<img src="https://video.udacity-data.com/topher/2019/March/5c8599e3_obstacledetectionfps/obstacledetectionfps.gif" width="700" height="400" />



### 2D Feature Tracking
The OpenCV library was used to conduct 2D feature tracking using a variety of keypoint detectors and descriptors. The descriptors were used to conduct keypoint matching from one video frame to the next - ultimately this could be used to calculate the time-to-collision for an autonomous vehicle. 

<img src="https://github.com/abdullahalsheeha/Udacity-Sensor-Fusion/blob/main/SFND_2D_Feature_Tracking/images/keypoints.png" width="820" height="248" />



### 3D Object Tracking
The object tracking project utilised lidar data in conjunction with the 2D feature tracking in order to match the bounding boxes for vehicles and provide a much more representative estimate of the time-to-collision. 

<img src="https://github.com/abdullahalsheeha/Udacity-Sensor-Fusion/blob/main/SFND_3D_Object_Tracking/results/FinalResults.jpg" width="1000" height="400" />
<img src="https://github.com/abdullahalsheeha/Udacity-Sensor-Fusion/blob/main/SFND_3D_Object_Tracking/results/3D.jpg" width="1000" height="400" />



### Radar Target Detection
MATLAB code was developed to detect targets using a FMCW radar module. A FFT was applied to the radar data in order to produce a Range-Doppler estimation - CFAR was then applied to remove noise and gain an accurate signal. Once a clean Range-Doppler map had been acquired then clustering could be applied to track an individual object. 



### Unscented Kalman Filter
An Unscented Kalman Filter was developed which took noisy lidar and radar readings in order to estimate the state of multiple cars on a highway. 

<img src="https://github.com/abdullahalsheeha/Udacity-Sensor-Fusion/blob/main/SFND_Unscented_Kalman_Filter/media/ukf_highway_tracked.gif" width="700" height="400" />

