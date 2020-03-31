# ROSBAG to POINTSMAP

CG空間上で取得したLiDAR点群，深度マップを記録したROSBAGから三次元地図を生成します．

## 依存

- ROS
    - cv_bridge
    - geometry_msgs
    - image_transport
    - pcl_conversions
    - pcl_ros
    - rosbag
    - roscpp
    - rospy
    - sensor_msgs
    - std_msgs
    - tf

- OpenCV
- Point Cloud Library

## コンパイル

```bash
cd ~/catkin_ws/src
git clone http://data.tasakilab:8080/gitbucket/git/shikishima/rosbag_to_pointsmap.git
cd ..
catkin_make
```

## 使い方

### LiDAR点群で三次元地図を作成する
1. [./launch/carla_lidar.launch](http://data.tasakilab:8080/gitbucket/shikishima/rosbag_to_pointsmap/blob/master/launch/carla_lidar.launch) をテキストエディタで開く．
2. コメント文に従って，入力するROSBAGファイルのパス，出力する三次元地図のパス，使用するLiDARのトピック名を書き換える．
3. roslaunchを実行する．**Ctrl+C**で終了．
    ```bash
    roslaunch rosbag_to_pointsmap carla_lidar.launch
    ```

### 深度マップで三次元地図を作成する．
1. [./launch.carla_depth.launch](http://data.tasakilab:8080/gitbucket/shikishima/rosbag_to_pointsmap/blob/master/launch/carla_depth.launch)をテキストエディタで開く．
2. コメント文に従って，入力するROSBAGファイルのパス，出力する三次元地図のパス，使用する深度マップ，CameraParamのトピック名を書き換える．
3. roslaunchを実行する．**Ctrl+C**で終了．
    ```bash
    roslaunch rosbag_to_pointsmap carla_depth.launch
    ```
