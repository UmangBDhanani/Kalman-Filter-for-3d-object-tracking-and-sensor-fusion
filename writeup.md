The project is the extension of the previous project on object detection in lidar point cloud. In here a target tracking module is set up to track multiple targets using extended Kalman filter where camera and lidar data are fused together. Initially a single-track module is used to track single vehicle in front with only lidar data and is built upon later with camera data to track multiple objects.


### 1. short recap of the four tracking steps  (filter, track management, association, camera fusion)

Track-initialization: Here as seen from the figure below a track is initialized when a new measurement is arrived from the lidar.
![single target tracking result-constant velocity ](https://user-images.githubusercontent.com/84092636/201680138-4a78f4a2-12e0-4b7c-8206-e3faa066f37a.png)

Track-management: This module comprises of managing the tracks after their initialization. The tracks that are updated with the measurement from the sensor are updated and tracking score is increased depending upon the detection in consecutive frames. Tracks with high covariance or low scores are deleted to avoid the vehicle from taking unnecessary maneuverers or braking.

Track-association: Herein an association is set up to update the tracks with the relative measurements. Mahalanobis distances are used to associate various measurements with appropriate track in the management module. The figure below shows the multiple objects (vehicles) being tracked using fusion.

The figuere shows the RMSE plot for the singel vehicle tracked overtime with mean loss.
![step2 rmse](https://user-images.githubusercontent.com/84092636/201682309-29b87dce-04ae-489b-9089-93d17e4b90b6.png)

Later, after the track management module was implemeneted an environment with multiple vehicles was used and the RMSE for multi traget tracking was produced as shown below.

![rmse step 3](https://user-images.githubusercontent.com/84092636/201682846-d44ab7a4-6658-4406-b1aa-6f4f980e93c1.png)

### 2 Benefits in camera-lidar fusion compared to lidar-only tracking

At the end the camera measurement data is used to update the track along with lidar data and as shown in the movie below the vehicles are tracked over time. The figure shows the RMSE plot for the 3 vehicles tracked over time and the system was able to generate result with lower losses as compared to only single lidar data from the above graph.

![rmse step 4](https://user-images.githubusercontent.com/84092636/201683204-8e1e7903-db04-4d2e-87b4-72df283fbfd7.png)

The video shows the tracking over time by extended kalman filter for multi object tracking. The confirmation of the vehicle tracks are done when the vehicle appears in sensor'S FOV for consecutive 5 or more frames and then the track is set to confirmed which is evident from the video when a new vehicle approaches from side and track initializaion is done, after tentative over some frames the track was confirmed and the vehicle was tracked over time.
{green - confirmed tracks, yellow - tentative tracks, red - deleted tracks}

https://user-images.githubusercontent.com/84092636/201684070-d1890654-423a-4c6c-8f1a-d13d01c0900e.mp4

The fusion of the data always output better results than the single sensor itself. As seen from the RMSE plot, the difference is visible from the fact that mean loss between lidar and fused lidar and camera data, the later one is comparatively lesser. The fusion data for more than one sensor always has benefits as the confidence of the object detection increases when seen from different sensors simultaneously. 

### 3 What challenges will a sensor fusion system face in real-life scenarios?
The main challenge of the fusion systems relates with the association of the sensor data with its measurement at given point of time. The data must be recognized correctly for specific sensors and treated accordingly in taking decision.

### 4 Ways to improve your tracking results in the future?

More complex motion models can be used to track highly maneuver driving as the situation on the road isn’t the ideal every time with straight line driving. ALso more robust association algorithm  has to be developed to designate the vehicles to track accurately.









