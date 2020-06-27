/*
finaltest.cpp用来做最后的实车实验。车上放置一个UWB模块，地面放置四个。
*/

#include <ros/ros.h>
#include <serial/serial.h>//这个包用来读取串口数据
#include <iostream>
#include <fstream>//将结果写入txt文件中
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <sstream>
#include <Eigen/Eigen>//Eigen这个包用来做矩阵运算
#include <Eigen/Dense>//Eigen3下载地址：http://eigen.tuxfamily.org/index.php?title=Main_Page
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


using namespace std;
using namespace Eigen;

//解算后的坐标值放在Coordinate中
struct Coordinate
{
    float XCoordinate;
    float YCoordinate;
};

//三边测距算法
//输入：四个基站的坐标、标签到四个基站的距离
//输出：标签坐标
Coordinate trilateration_LS(float xa, float ya, float xb, float yb, float xc, float yc, float da, float db, float dc)
{
  Coordinate result;
  float M_inter = 0; float N_inter = 0; float O_inter = 0;
  M_inter = da*da - xa*xa - ya*ya;
  N_inter = db*db - xb*xb - yb*yb;
  O_inter = dc*dc - xc*xc - yc*yc;

  MatrixXd A(2,2);
  A(0,0) = 2* (xb-xa);
  A(0,1) = 2* (yb-ya);
  A(1,0) = 2* (xc-xa);
  A(1,1) = 2* (yc-ya);
  VectorXd B(2);
  B(0) = M_inter - N_inter;
  B(1) = M_inter - O_inter;

  VectorXd X(2);
  X(0) = 0;
  X(1) = 0;

  //The least-square solution is:
  X = A.bdcSvd(ComputeThinU | ComputeThinV).solve(B);
  result.XCoordinate = X(0);
  result.YCoordinate = X(1);

  return result;


};

//对于每一个UWB标签，需要做静态测距实验测量他的零漂等，将测距数据做线性修正
float distance_cali(float distance)
{
    float result;
    float a,b;
    a = 1;
    b = 0;
    result = distance * a + b;

    return result;
};

int main(int argc, char **argv)
{
        float xa, ya, xb, yb, xc, yc, xd, yd;
        //distance表示标签到基站的距离
        float distance = 0;

        float Xcoordinate0 = 0;
        float Ycoordinate0 = 0;

        Coordinate tag0;


        //设置基站坐标 单位：mm
        xa = 0; ya = -20000; xb=17321 ;yb=10000; xc=-17321; yc=10000;xd = 0;yd=0;

        int i = 0;

        //初始化ROS节点 
        ros::init(argc, argv, "finaltest");
	ros::NodeHandle nh;


	ros::Rate loop_rate(100);
        //将测距结果写入文件中
        //ios::app表示在文件末尾接着写
    ofstream datawrite1("/home/yidong/catkin_ws/src/location_uwb/src/textfiles/finaltest/test2/distance1.txt", ios::app);
	ofstream datawrite2("/home/yidong/catkin_ws/src/location_uwb/src/textfiles/finaltest/test2/distance2.txt", ios::app);
	ofstream datawrite3("/home/yidong/catkin_ws/src/location_uwb/src/textfiles/finaltest/test2/distance3.txt", ios::app);
	ofstream datawrite4("/home/yidong/catkin_ws/src/location_uwb/src/textfiles/finaltest/test2/distance4.txt", ios::app);	
	if(!datawrite1.is_open())
    {
        cout << "Cannot open text file" << endl;
    }
    else
    {
        cout<<"Successfully Open!"<< endl;
    }	
        //从串口读取数据
        serial::Serial sp;
	serial::Timeout to = serial::Timeout::simpleTimeout(100);
	// 设置读取的COM口
    // 在此之前最好更改串口权限：sudo 666 /dev/ttyACM0
	sp.setPort("/dev/ttyACM0");
	// 设置波特率
	sp.setBaudrate(115200);

	sp.setTimeout(to);

	try
	{
		sp.open();
	}
	catch(serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port.");
		return -1;
	}

	if (sp.isOpen())
	{
		ROS_INFO_STREAM("/dev/ttyACM0 is opened.");
	}
	else
	{
		return -1;
	}

	while(ros::ok())
	{
            if(sp.available()){
                ROS_INFO_STREAM("Reading from serial port\n");
                //从串口读取的数据放入result中
                std_msgs::String result;
                result.data = sp.read(sp.available());

                //ROS_INFO_STREAM("The length of result.data is:" << result.data.length());
                //ROS_INFO_STREAM("READ:" << result.data);

                if(result.data.length() != 130)
                {
                    continue;
                }

                ROS_INFO_STREAM("The length of result.data is:" << result.data.length());

                //依次读取distance等等
                //八位十六进制数转十进制
                i=78;
                distance = 0;
                for(i; i > 70; i--)
                {
                    if(result.data[i] >= '0' && result.data[i] <= '9')
                    {
                        distance += (result.data[i]-48)*pow(16,78-i);
                    }
                    else if(result.data[i] >= 'a' && result.data[i] <= 'f')
                    {
                        distance += (result.data[i]-87)*pow(16,78-i);
                    }
                }

		if(result.data[60] == '0')
			datawrite1<< distance<<endl;
		else if(result.data[60] == '1')
			datawrite2<< distance<<endl;
		else if(result.data[60] == '2')
			datawrite3<< distance<<endl;
		else if(result.data[60] == '3')
			datawrite4<< distance<<endl;


            }


		ros::spinOnce();

		loop_rate.sleep();
	}

	sp.close();

	datawrite1.close();
	datawrite2.close();
	datawrite3.close();
	datawrite4.close();
	return 0;
}
