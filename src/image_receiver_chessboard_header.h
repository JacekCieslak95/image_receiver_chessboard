#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void createTrackbars();
void imageFilters(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded, char color);
std::vector<cv::Vec3f> circleFinding(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded);
