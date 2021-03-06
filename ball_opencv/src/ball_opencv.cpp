#include <ros/ros.h>
//#include "ball_direction.h"
#include "ball_opencv/ball_direction.h"

//OpenCV specific includes
#include <opencv2/opencv.hpp>
//get clouds and images
#include"cloudTraver.h"
//tools to detect
//#include "picture.h"

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <string>
#include <cmath>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

const char *pstrWindowsSrcTitle = "原图(http://blog.csdn.net/MoreWindows)";
const char *pstrWindowsLineName = "圆检测";


int direction;
bool  serviceCallBack(ball_opencv::ball_direction::Request &req, ball_opencv::ball_direction::Response &rep){
	rep.direction = direction;
}
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ball_opencv");
    ros::NodeHandle n;
    ros::Rate waiting_rate(30);
     //ros::NodeHandle nh_ ;
    ros::ServiceServer ball_direction_service_ = n.advertiseService("ball_direction",serviceCallBack);


    int x_position[20];//用于存储每10帧的x像素坐标

    //strat a traver and wait for its ready
    cloudTraver ct(n);
    while(!ct.isReady())
    {
        ros::spinOnce();
        waiting_rate.sleep();
    }

    //cvNamedWindow("CurrentImage",CV_WINDOW_AUTOSIZE);
    //cv::Mat image;
    Mat src, dst, color_dst;

    std::string objectNameTemp = "TEMP";
    int count = 0;
    int mid_value = 0;
    while(ros::ok())
    {

        while(!ct.isReady())
        {
            ros::spinOnce();
        }

        src = ct.getImage();
		Mat image;
		image = src;
		cvtColor(image, image, CV_BGR2GRAY);

		//sharpenImage1(image, image);

		GaussianBlur(image, image, Size(5, 5), 1.5);

		//sharpenImage1(image, image);
		vector<Vec3f> circles;
		HoughCircles(image, circles, CV_HOUGH_GRADIENT,
			2,
			120,
			200,
			70,
			25, 230);

		vector<Vec3f>::
				const_iterator itc = circles.begin();
		while(itc != circles.end()) {
			cout << "x:" << (*itc)[0] << "y:" << (*itc)[1] << endl;
			circle(image,
				Point((*itc)[0], (*itc)[1]),//圆心
				(*itc)[2],	//半径
				Scalar(255), //颜色
				2); //厚度

			x_position[count] = (*itc)[0];
			//cout << "x_position" << count << ":" << x_position << endl;
			count++;
			if (count == 5) {
				sort(x_position, x_position + 10);
				int mid_value = x_position[4];//每10帧的中位数

				cout << "mid_value:" << mid_value << endl;
				cout << "............................." << endl;

				if (mid_value > 365) {
					cout << "need to turn right" << endl;
					direction = 1;
				} else if (mid_value < 315) {
					cout << "need to turn left" << endl;
					direction = -1;
				} else {
					cout << "succeed" << endl;
					direction = 0;
				}

				count = 0;
				mid_value = 0;
			}
			++itc;
		}

		imshow("Detected Circles", image);



        char temp=cvWaitKey(33);
        if(temp == 27)
        {
            break;
        }
    }



}




