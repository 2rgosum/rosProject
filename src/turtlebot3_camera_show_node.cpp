#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;

ros::Subscriber sub_image;

Mat src;

void getImage(const sensor_msgs::CompressedImageConstPtr& msg){

       	cv_bridge::CvImageConstPtr cv_ptr;

       	try{
	       	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       	}
	
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}
	
	src = cv_ptr->image;
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "turtlebot3_camera_show");
	ros::NodeHandle nh;
	
	sub_image = nh.subscribe("/usb_cam/image_raw/compressed", 1, getImage);
	
	while(ros::ok()){
		ros::spinOnce();
		
		if(!src.empty()){
			cv::Mat dst;
			cv::Mat dst2;
			cv::Mat i_canny;
			vector<Vec2f> lines;

			cv::cvtColor(src, dst, cv::COLOR_BGR2GRAY);
			cv::threshold(dst,dst2,220,255,cv::THRESH_BINARY);
			cv::Canny(dst,i_canny,150,255);
			cv::HoughLines(i_canny,lines,1,CV_PI/180,150);
			
			cv::Mat img_hough;
			i_canny.copyTo(img_hough);
			cv::Mat img_lane;
			
			cv::threshold(i_canny,img_lane,150,255,CV_THRESH_MASK);
			
			for (size_t i = 0; i < lines.size(); i++){
				
				float rho = lines[i][0], theta = lines[i][1];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a * rho, y0 = b * rho;
				
				pt1.x = cvRound(x0 + 1000 * (-b));
				pt1.y = cvRound(y0 + 1000 * (a));
				pt2.x = cvRound(x0 - 1000 * (-b));
				pt2.y = cvRound(y0 - 1000 * (a));
				line(img_hough, pt1, pt2, Scalar(0,0,255), 2, 8);
				line(img_lane, pt1, pt2, Scalar::all(255), 1, 8);
			}
			
			cv::imshow("src", dst);
			cv::imshow("threshold",dst2);
			cv::imshow("Canny",i_canny);
			cv::imshow("line",img_lane);
			waitKey(1);
		}
	}
	return 0;
}
