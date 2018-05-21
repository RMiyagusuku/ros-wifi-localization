# ros-wifi-localization
Wireless signal strength based localization for ROS

## Installation

clone this git repository and add it to your catkin path:

    git clone https://github.com/RMiyagusuku/ros-wifi-localization.git /tmp/ros-wifi

Copy rss and tests to your catkin folder
    
    scp -r /tmp/ros-wifi/rss   ~/catkin_ws/src/rss
    scp -r /tmp/ros-wifi/tests ~/catkin_ws/src/tests
  
Rosbags for testing available at

    http://www.robot.t.u-tokyo.ac.jp/~miyagusuku/datasets

Unzip and copy downloaded or original rosbags to
    
    ~/catkin_ws/src/tests/bags


## Tested Platform

    Ubuntu 14.04.5
    ROS indigo
    Python 2.7

## Libraries dependencies

    wifi-localization at https://github.com/RMiyagusuku/wifi-localization
    GPy at https://github.com/SheffieldML/GPy
    Numpy 1.11.1+
    Scipy 0.18.0+

## Questions

Please mail directly to 

    miyagusuku at robot.t.u-tokyo.ac.jp

