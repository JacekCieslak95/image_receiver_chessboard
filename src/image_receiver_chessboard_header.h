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

//definicje stałych
#define PI 3.14159
//definicje danych dotyczących planszy
#define pattern_colls 7
#define pattern_rows 4
#define bok_kwad 8.0
#define l_punktow (pattern_colls*pattern_rows)

using namespace cv;
using namespace std;

double deg2rad(double angle_in_degrees);
double rad2deg(double angle_in_radians);

void drawGrid(Mat &imgThresholded);
void writeMsg(Mat &imgThresholded);
void chessboardParam();
void findControl(cv_bridge::CvImagePtr &cv_ptr);

void imageCb(const sensor_msgs::ImageConstPtr& msg);
static int gcd (int a, int b);
