#include "image_receiver_chessboard_header.h"


int fontFace = cv::FONT_HERSHEY_DUPLEX;
//początkowe wartości do filtra. Po dobraniu można to wywalić i w ich miejsce wsadziś stałe

using namespace cv;
using namespace std;

Mat frame;
double focal_length;
Point2d center;
Mat camera_matrix;
Mat dist_coeffs;
Mat imgOriginal; //obiekt, w ktorym beda przechowane kolejne klatki z kamery
Mat imgOriginal_8bit; //obiekt, w którym bedzie przechowana skonwertowana klatka - wymaga tego jedna z funkcji

vector<Point2f> corners; //wektor elementow point2f - przechowa wspolrzedne znalezionych krancow pol szachownicy

Mat rotation_vector(3,1,cv::DataType<double>::type); // Rotation in axis-angle form
Mat translation_vector(3,1,cv::DataType<double>::type);
//cv::Mat rvec(3,1,cv::DataType<double>::type);
int l_punktow;
int pattern_colls=4;
int pattern_rows=7;
Size patternsize(pattern_colls,pattern_rows); //funkcja findchessboardcorners musi znac wielkosc szachownicy - podaje ja tutaj

float bok_kwad=8.0; //w centymetrach

vector<Point3f> corners_3d;
int licznik_przebiegow = 1;


bool first_run=true;
image_transport::Subscriber image_sub_; //do subskrypcji obrazu
image_transport::Publisher image_pub_;	//do publikacji przerobionego obrazu


int main(int argc, char** argv)
{
	std::cout<<"Image_receiver started!"<<std::endl;
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;
	image_sub_ = nh.subscribe("/camera/image", 10,	imageCb);
	image_pub_ = nh.advertise<sensor_msgs::Image>("/image_converter/output_video", 1);

	cv::namedWindow("Window with detection");

	ros::spin();

	cv::destroyWindow("Window with detection");
	std::cout<<std::endl<<"Image_receiver closed!"<<std::endl;
	return 0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	//przekopiowanie wiadomości do wskaźnika (?)
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	{
		if(first_run)
		{
			first_run=false;
			frame = cv_ptr->image;
			focal_length =frame.cols; // aproksymacja ogniskowej
			center = Point2d(frame.cols/2,frame.rows/2);
			camera_matrix = (Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
			dist_coeffs =Mat::zeros(4,1,DataType<double>::type); // Assuming no lens distortion
			cout << "Camera Matrix " << endl << camera_matrix << endl;
			l_punktow=pattern_rows*pattern_colls;
			//zeby wpisac do wektora, musze najpierw stworzyc 54 puste elementy - tu chyba lezal blad

			for (int p=0; p<l_punktow; p++)
			{

				corners_3d.push_back(Point3d(0,0,0));

			}
			corners_3d[0].x=0;
			corners_3d[0].y=0;
			corners_3d[0].z=0;

			for (int i=1; i<l_punktow; i++)
			{
				corners_3d[i].x=corners_3d[i-1].x+bok_kwad;

				if ((i+1)%9==0) // (jak dojedzie do konca wiersza to zeruj x, zwiększ y o wysokosc kwadratu - wypelniamy wyższy wiersz od lewej
				{
					corners_3d[i].x=0;
					corners_3d[i].y=corners_3d[i-1].y+bok_kwad;

				}

				else

				{
					corners_3d[i].y=corners_3d[i-1].y; //jak nie dojechal, to nie zmieniaj wysokosci y
				}

				corners_3d[i].z=0; //bo szachownica jest plaska

			}
		}
		imgOriginal=cv_ptr->image;
		imgOriginal.convertTo(imgOriginal_8bit, CV_8U); //konwersja do 8-bitowej glebi - takie obrazy przyjmuje funkcja znajdujaca findchessboardcorners()

		bool patternfound = findChessboardCorners(imgOriginal_8bit, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		//corners wypelnione znalezionymi wspolrzednymi

		if (!patternfound)
		{
			cout<<"Nie znalazlem szachownicy!"<< licznik_przebiegow++ <<endl;
		}

		if (patternfound)

		{

			solvePnP(corners_3d, corners, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

			//wyrzucenie do konsoli

			cout<<"Wektor rotacji"<<rotation_vector<<endl;
		    //cout<<"Wektor translacji "<<translation_vector<<endl;

		    cout<<"x: "<<translation_vector.at<double>(0,0) << " y: "<<translation_vector.at<double>( 1,0) << " z: "<<translation_vector.at<double>(2,0) << endl;


		}

		drawChessboardCorners(imgOriginal_8bit, patternsize, corners, patternfound); //rysowanie linii

		imshow("Window with detection", cv_ptr->image);
		imshow ("Original", imgOriginal_8bit);//wyrzucenie do okna


		image_pub_.publish(cv_ptr->toImageMsg());

		cv::waitKey(3);

	}
}
