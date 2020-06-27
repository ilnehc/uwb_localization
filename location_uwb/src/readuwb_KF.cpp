/*
带卡尔曼滤波的坐标解算
*/

#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <sstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;
using namespace Eigen;

struct Coordinate
{
    float XCoordinate;
    float YCoordinate;
};

double gaussrand()
{
    static double V1, V2, S;
    static int phase = 0;
    double X;

    if (phase == 0){
        do{
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;

            V1 = 2*U1 -1;
            V2 = 2*U2 -1;
            S = V1 * V1 + V2 * V2;

        }while(S >= 1 || S == 0);

        X = V1 * sqrt(-2 * log(S) / S);

    }else
        X = V2 * sqrt(-2 * log(S) / S);

    phase = 1 - phase;

    return X;
}

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

float distance_cali(float distance)
{
    float result;
    float a,b;
    a = 1;
    b = -200;
    result = distance * a + b;

    return result;
};

int main(int argc, char **argv)
{
    Coordinate tag0;
    Coordinate tag0_vani;
    int i, j;
    float xa, ya, xb, yb, xc, yc;
    float distance_a = 0;
    float distance_a_old = 0;

    float distance_b = 0;
    float distance_b_old = 0;

    float distance_c = 0;
    float distance_c_old = 0;

    float Xcoordinate0 = 0;
    float Ycoordinate0 = 0;
    float Xcoordinate0_vani = 0;
    float Ycoordinate0_vani = 0;
    float delta_t = 0.1;

    MatrixXd Kalman_A(2,2);
    Kalman_A << 1, delta_t,
                0, 1;


    MatrixXd Kalman_H(2,2);
    Kalman_H << 1, 0,
                0, 1;

    VectorXd Kalman_wk(2);
    Kalman_wk << 0,
                 0;

    VectorXd Kalman_vk(2);
    Kalman_vk << 0,
                 0;

    VectorXd input_state_a(2);
    input_state_a << 0,
                   0;

    VectorXd input_state_b(2);
    input_state_b << 0,
                   0;

    VectorXd input_state_c(2);
    input_state_c << 0,
                   0;

    VectorXd inter_state_a(2);
    inter_state_a << 0,
                   0;

    VectorXd inter_state_b(2);
    inter_state_b << 0,
                   0;

    VectorXd inter_state_c(2);
    inter_state_c << 0,
                   0;

    VectorXd output_state_a(2);
    output_state_a << 0,
                    0;

    VectorXd output_state_b(2);
    output_state_b << 0,
                    0;

    VectorXd output_state_c(2);
    output_state_c << 0,
                    0;

    MatrixXd input_p_a(2,2);
    input_p_a << 1, 0,
               0, 1;

    MatrixXd input_p_b(2,2);
    input_p_b << 1, 0,
               0, 1;

    MatrixXd input_p_c(2,2);
    input_p_c << 1, 0,
               0, 1;

    MatrixXd inter_p_a(2,2);
    inter_p_a << 0, 0,
               0, 0;

    MatrixXd inter_p_b(2,2);
    inter_p_b << 0, 0,
               0, 0;

    MatrixXd inter_p_c(2,2);
    inter_p_c << 0, 0,
               0, 0;

    MatrixXd output_p_a(2,2);
    output_p_a << 0, 0,
                0, 0;

    MatrixXd output_p_b(2,2);
    output_p_b << 0, 0,
                0, 0;

    MatrixXd output_p_c(2,2);
    output_p_c << 0, 0,
                0, 0;

    MatrixXd Kalman_Q(2,2);
    Kalman_Q << 0.01, 0,
                0, 0.01;//Need tune;

    MatrixXd Kalman_R(2,2);
    Kalman_R << 5, 0,
                0, 5;

    MatrixXd Kalman_gain(2,2);
    Kalman_gain << 0, 0,
                   0, 0;

    VectorXd Kalman_Z(2);
    Kalman_Z << 0,
                0;

    MatrixXd temp(2,2);
    temp << 0, 0,
            0, 0;

    MatrixXd Kalman_I(2,2);
    Kalman_I << 1, 0,
                0, 1;


    //Set the coordinate of A, B and C:
    xa = 0; ya = 0; xb=3820; yb=0; xc=0; yc=1960;

    //Initialize the node
    ros::init(argc, argv, "readuwb_KF");
	ros::NodeHandle nh;


	ros::Rate loop_rate(10);
/*

       ofstream datawrite("/home/yidong/catkin_ws/src/location_uwb/src/textfiles/point4KF.txt",ios::app);
       if(!datawrite.is_open())
       {
           cout << "Cannot open text file" << endl;
       }
       else
       {
           datawrite<<"Xcoordinate:        "<<"Ycoordinate:        "<< "Point:          "<<endl;
       }

*/
       ofstream datawrite1("/home/yidong/catkin_ws/src/location_uwb/src/textfiles/moveraw.txt",ios::app);
       if(!datawrite1.is_open())
       {
           cout << "Cannot open moveraw.txt" << endl;
       }

       ofstream datawrite2("/home/yidong/catkin_ws/src/location_uwb/src/textfiles/moveKF.txt",ios::app);
       if(!datawrite2.is_open())
       {
           cout << "Cannot open moveKF.txt" << endl;
       }

    serial::Serial sp;
	serial::Timeout to = serial::Timeout::simpleTimeout(100);
	// Set the COM port
	sp.setPort("/dev/ttyACM0");
	// Set Baudrate
	sp.setBaudrate(115200);
	// Set Timeout
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
                std_msgs::String result;
                result.data = sp.read(sp.available());

                if(result.data.length() != 130)
                {
                    continue;
                }

                ROS_INFO_STREAM("The length of result.data is:" << result.data.length());

                i=78;
                distance_a = 0;
                for(i; i > 70; i--)
                {
                    if(result.data[i] >= '0' && result.data[i] <= '9')
                    {
                        distance_a += (result.data[i]-48)*pow(16,78-i);
                    }
                    else if(result.data[i] >= 'a' && result.data[i] <= 'f')
                    {
                        distance_a += (result.data[i]-87)*pow(16,78-i);
                    }
                }
                distance_a = distance_cali(distance_a);
                ROS_INFO_STREAM("The distance_a is:" << distance_a);


                i=87;
                distance_b = 0;
                for(i; i > 79; i--)
                {
                    if(result.data[i] >= '0' && result.data[i] <= '9')
                    {
                        distance_b += (result.data[i]-48)*pow(16,87-i);
                    }
                    else if(result.data[i] >= 'a' && result.data[i] <= 'f')
                    {
                        distance_b += (result.data[i]-87)*pow(16,87-i);
                    }
                }
                distance_b = distance_cali(distance_b);
                ROS_INFO_STREAM("The distance_b is:" << distance_b);

                i=96;
                distance_c = 0;
                for(i; i > 88; i--)
                {
                    if(result.data[i] >= '0' && result.data[i] <= '9')
                    {
                        distance_c += (result.data[i]-48)*pow(16,96-i);
                    }
                    else if(result.data[i] >= 'a' && result.data[i] <= 'f')
                    {
                        distance_c += (result.data[i]-87)*pow(16,96-i);
                    }
                }
                distance_c = distance_cali(distance_c);
                ROS_INFO_STREAM("The distance_c is:" << distance_c);

                tag0_vani = trilateration_LS(xa,ya,xb,yb,xc,yc,distance_a,distance_b,distance_c);
                Xcoordinate0_vani = tag0_vani.XCoordinate;
                Ycoordinate0_vani = tag0_vani.YCoordinate;
                datawrite1 << Xcoordinate0_vani << " " << Ycoordinate0_vani << endl;



                // Kalman Filter:

                Kalman_wk(0) = 0;
                Kalman_wk(1) = 1*gaussrand();
                Kalman_vk(0) = 0;
                Kalman_vk(1) = 1*gaussrand();

                // For distance_a:

                inter_state_a = Kalman_A * input_state_a + Kalman_wk; //Prediction
                inter_p_a = (Kalman_A * input_p_a) * Kalman_A.transpose() + Kalman_Q; //Prediction
                Kalman_Z(0) = distance_a + Kalman_vk(0);
                Kalman_Z(1) = (distance_a - distance_a_old) / 0.3 + Kalman_vk(1);

                temp = (Kalman_H * inter_p_a) * Kalman_H.transpose() + Kalman_R;
                Kalman_gain = (inter_p_a * Kalman_H.transpose()) * temp.inverse();//Calculate Kalman Gain

                output_state_a = inter_state_a + Kalman_gain * (Kalman_Z - Kalman_H * inter_state_a); //Update
                output_p_a = (Kalman_I - Kalman_gain * Kalman_H) * inter_p_a; //Update

                distance_a_old = output_state_a(0);

                input_state_a = output_state_a;
                input_p_a = output_p_a;

                ROS_INFO_STREAM("The distance_a after KF is :" << output_state_a(0));

                // For distance_b:
                inter_state_b = Kalman_A * input_state_b + Kalman_wk; //Prediction
                inter_p_b = (Kalman_A * input_p_b) * Kalman_A.transpose() + Kalman_Q; //Prediction
                Kalman_Z(0) = distance_b + Kalman_vk(0);
                Kalman_Z(1) = (distance_b - distance_b_old) / 0.3 + Kalman_vk(1);

                temp = (Kalman_H * inter_p_b) * Kalman_H.transpose() + Kalman_R;
                Kalman_gain = (inter_p_b * Kalman_H.transpose()) * temp.inverse();//Calculate Kalman Gain

                output_state_b = inter_state_b + Kalman_gain * (Kalman_Z - Kalman_H * inter_state_b); //Update
                output_p_b = (Kalman_I - Kalman_gain * Kalman_H) * inter_p_b; //Update

                distance_b_old = output_state_b(0);

                input_state_b = output_state_b;
                input_p_b = output_p_b;

                ROS_INFO_STREAM("The distance_b after KF is :" << output_state_b(0));

                //For distance_c:
                inter_state_c = Kalman_A * input_state_c + Kalman_wk; //Prediction
                inter_p_c = (Kalman_A * input_p_c) * Kalman_A.transpose() + Kalman_Q; //Prediction
                Kalman_Z(0) = distance_c + Kalman_vk(0);
                Kalman_Z(1) = (distance_c - distance_c_old) / 0.3 + Kalman_vk(1);

                temp = (Kalman_H * inter_p_c) * Kalman_H.transpose() + Kalman_R;
                Kalman_gain = (inter_p_c * Kalman_H.transpose()) * temp.inverse();//Calculate Kalman Gain

                output_state_c = inter_state_c + Kalman_gain * (Kalman_Z - Kalman_H * inter_state_c); //Update
                output_p_c = (Kalman_I - Kalman_gain * Kalman_H) * inter_p_c; //Update

                distance_c_old = output_state_c(0);

                input_state_c = output_state_c;
                input_p_c = output_p_c;

                ROS_INFO_STREAM("The distance_c after KF is :" << output_state_c(0));


                tag0 = trilateration_LS(xa,ya,xb,yb,xc,yc,output_state_a(0),output_state_b(0),output_state_c(0));
                Xcoordinate0 = tag0.XCoordinate;
                Ycoordinate0 = tag0.YCoordinate;
                datawrite2<< Xcoordinate0 << " " << Ycoordinate0 << endl;

            }

		ros::spinOnce();

		loop_rate.sleep();
	}

	sp.close();
    
    datawrite1.close();
    datawrite2.close();
    

	return 0;
}
