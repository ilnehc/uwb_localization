# Design and development of UWB based localization system for self-driving cars_poster
**Authors:** Li Chen, Yidong He, Zehao Li, Zhichao Chen

This project is the college-company co-sponsored undergraduates graduation project at School of Mechanical Engineering, Shanghai Jiao Tong University in 2019. It is co-directed by Porfessor [Yafei Wang](http://me.sjtu.edu.cn/teacher_directory1/wangyafei.html) at SJTU and PhD. Dong at SUPERG.AI (Shenzhen Shuxiang Technology Co., Ltd.). It aims at designing an ultra-wideband (UWB) localization system for AGV at autonomous ports, which solves challenges including trilateration localization algorithm, base stations configuration and virtual simulation.

Our final project poster is shown below and located here: [poster](https://github.com/ilnehc/uwb_localization/blob/master/Design%20and%20development%20of%20UWB%20based%20localization%20system%20for%20self-driving%20cars_poster.pdf)

Our final presentation slides is located here: [slides](https://github.com/ilnehc/uwb_localization/blob/master/Design%20and%20development%20of%20UWB%20based%20localization%20system%20for%20self-driving%20cars_poster_pre%20slides.pptx)

![avatar](https://github.com/ilnehc/uwb_localization/blob/master/Design%20and%20development%20of%20UWB%20based%20localization%20system%20for%20self-driving%20cars_poster.jpeg)


# Prerequisites
This repository only contains the localization part, which includes trilateration algorithm and KF filter. The following libraries are used:

## ROS
We use ROS to store data and communicate with MATLAB/Simulink for simulation.

## Eigen3
Download and install instructions can be found at: http://eigen.tuxfamily.org.

## Serial
One UWB label is connected to the computer and use serial library to read data.
