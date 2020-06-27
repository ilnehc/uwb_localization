/*
与Simulink和PreScan对接
*/
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <sstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>


using namespace std;
using namespace Eigen;

//卡尔曼滤波参数声明
MatrixXd Kalman_A(2,2);
MatrixXd Kalman_H(2,2);
VectorXd Kalman_wk(2);
VectorXd Kalman_vk(2);
VectorXd input_state_a(2);
VectorXd input_state_b(2);
VectorXd input_state_c(2);
VectorXd inter_state_a(2);
VectorXd inter_state_b(2);
VectorXd inter_state_c(2);
VectorXd output_state_a(2);
VectorXd output_state_b(2);
VectorXd output_state_c(2);
MatrixXd input_p_a(2,2);
MatrixXd input_p_b(2,2);
MatrixXd input_p_c(2,2);
MatrixXd inter_p_a(2,2);
MatrixXd inter_p_b(2,2);
MatrixXd inter_p_c(2,2);
MatrixXd output_p_a(2,2);
MatrixXd output_p_b(2,2);
MatrixXd output_p_c(2,2);
MatrixXd Kalman_Q(2,2);
MatrixXd Kalman_R(2,2);
MatrixXd Kalman_gain(2,2);
VectorXd Kalman_Z(2);
MatrixXd temp(2,2);
MatrixXd Kalman_I(2,2);
float delta_t;





struct Coordinate
{
    float XCoordinate;
    float YCoordinate;
};

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
//生成高斯随机数，服从均值为0，方差为1的高斯分布
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


class SubscribeandPublish
{
private:
    ros::NodeHandle nh;
    ros::Publisher tag0pub;
    ros::Subscriber tag0sub;

    struct Coordinate tag0;
    int i, j;
    float xa, ya, xb, yb, xc, yc, xd, yd;
    float distance_a;

    float distance_b;

    float distance_c;

    float distance_d;

    float Xcoordinate0;
    float Xcoordinate0_old;

    float Ycoordinate0;
    float Ycoordinate0_old;

    float delta_t;

public:
        SubscribeandPublish()
        {
            distance_a = 0;


            distance_b = 0;

            distance_c = 0;

            distance_d = 0;

            Xcoordinate0 = 0;
            Ycoordinate0 = 0;
            delta_t = 0.1;
            //Simulink中，距离信息发布到infotag0上，订阅这个topic
            //计算结果发到coortag0上，Simulink中可以订阅这个topic
            tag0pub = nh.advertise<geometry_msgs::Twist>("coortag0", 1000);
            tag0sub = nh.subscribe("infotag0", 1000, &SubscribeandPublish::mycallback, this);
    }
        //回调函数，卡尔曼滤波参数声明若写到回调函数中可能会出错
        void mycallback(const nav_msgs::Odometry& message)
    {
        geometry_msgs::Twist twist0;

        xa = message.pose.pose.position.x;
        ya = message.pose.pose.position.y;
        distance_a = message.pose.pose.position.z;

        xb = message.pose.pose.orientation.x;
        yb = message.pose.pose.orientation.y;
        distance_b = message.pose.pose.orientation.z;

        xc = message.twist.twist.linear.x;
        yc = message.twist.twist.linear.y;
        distance_c = message.twist.twist.linear.z;

        xd = message.twist.twist.angular.x;
        yd = message.twist.twist.angular.y;
        distance_d = message.twist.twist.angular.z;

        tag0 = trilateration_LS(xa,ya,xb,yb,xc,yc,distance_a,distance_b,distance_c);
        Xcoordinate0 =tag0.XCoordinate;
        Ycoordinate0 =tag0.YCoordinate;

        //Kalman Filter:
        Kalman_wk(0) = 0;
        Kalman_wk(1) = 0;//1*gaussrand();
        Kalman_vk(0) = 0;
        Kalman_vk(1) = 0;//*gaussrand();

        // For Xcoordinate0:

        inter_state_a = Kalman_A * input_state_a + Kalman_wk; //Prediction
        inter_p_a = (Kalman_A * input_p_a) * Kalman_A.transpose() + Kalman_Q; //Prediction
        Kalman_Z(0) = Xcoordinate0 + Kalman_vk(0);
        Kalman_Z(1) = (Xcoordinate0 - Xcoordinate0_old) / 0.02 + Kalman_vk(1);

        temp = (Kalman_H * inter_p_a) * Kalman_H.transpose() + Kalman_R;
        Kalman_gain = (inter_p_a * Kalman_H.transpose()) * temp.inverse();//Calculate Kalman Gain

        output_state_a = inter_state_a + Kalman_gain * (Kalman_Z - Kalman_H * inter_state_a); //Update
        output_p_a = (Kalman_I - Kalman_gain * Kalman_H) * inter_p_a; //Update

        Xcoordinate0_old = output_state_a(0);

        input_state_a = output_state_a;
        input_p_a = output_p_a;

        ROS_INFO_STREAM("The Xcoordinate0 after KF is :" << output_state_a(0));

        // For Ycoordinate0:
        inter_state_b = Kalman_A * input_state_b + Kalman_wk; //Prediction
        inter_p_b = (Kalman_A * input_p_b) * Kalman_A.transpose() + Kalman_Q; //Prediction
        Kalman_Z(0) = Ycoordinate0 + Kalman_vk(0);
        Kalman_Z(1) = (Ycoordinate0 - Ycoordinate0_old) / 0.02 + Kalman_vk(1);

        temp = (Kalman_H * inter_p_b) * Kalman_H.transpose() + Kalman_R;
        Kalman_gain = (inter_p_b * Kalman_H.transpose()) * temp.inverse();//Calculate Kalman Gain

        output_state_b = inter_state_b + Kalman_gain * (Kalman_Z - Kalman_H * inter_state_b); //Update
        output_p_b = (Kalman_I - Kalman_gain * Kalman_H) * inter_p_b; //Update

        Ycoordinate0_old = output_state_b(0);

        input_state_b = output_state_b;
        input_p_b = output_p_b;

        ROS_INFO_STREAM("The Ycoordinate0 after KF is :" << output_state_b(0));

        twist0.linear.x = output_state_a(0);
        twist0.linear.y = output_state_b(0);


        tag0pub.publish(twist0);

    }




};



int main(int argc, char **argv)
{
    //卡尔曼滤波参数赋值，放在了main函数中，不太美观但是找不到替代方法了
    delta_t = 0.02;

    Kalman_A << 1, delta_t,
                0, 1;

    Kalman_H << 1, 0,
                0, 1;


    Kalman_wk << 0,
                 0;


    Kalman_vk << 0,
                 0;


    input_state_a << 0,
                   0;


    input_state_b << 0,
                   0;


    input_state_c << 0,
                   0;


    inter_state_a << 0,
                   0;


    inter_state_b << 0,
                   0;

    inter_state_c << 0,
                   0;


    output_state_a << 0,
                    0;


    output_state_b << 0,
                    0;


    output_state_c << 0,
                    0;


    input_p_a << 1, 0,
               0, 1;


    input_p_b << 1, 0,
               0, 1;


    input_p_c << 1, 0,
               0, 1;


    inter_p_a << 0, 0,
               0, 0;


    inter_p_b << 0, 0,
               0, 0;


    inter_p_c << 0, 0,
               0, 0;


    output_p_a << 0, 0,
                0, 0;

    output_p_b << 0, 0,
                0, 0;

    output_p_c << 0, 0,
                0, 0;

    Kalman_Q << 0.01, 0,
                0, 0.01;//Need tune;


    Kalman_R << 50, 0,
                0, 50;


    Kalman_gain << 0, 0,
                   0, 0;


    Kalman_Z << 0,
                0;


    temp << 0, 0,
            0, 0;


    Kalman_I << 1, 0,
                0, 1;
        //Initialize the node
        ros::init(argc, argv, "readsimuKF");

        SubscribeandPublish readsimuKF;

        ros::Rate loop_rate(10);
        ros::spin();

        return 0;
}
