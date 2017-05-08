#include "image_receiver_chessboard_header.h"

//zmienna przechowująca informacje o odnalezieniu obiektu
bool found=false;

//Zmienne pomocnicze do wyswietlania komunikatów
int fontFace = FONT_HERSHEY_DUPLEX;
Point hor_text_base(0, 0);
Point vert_text_base(0, 0);
Point dist_text_base(0, 0);
string hor_msg;
string vert_msg;
string dist_msg;
int hor_state=-1;
int vert_state=-1;
int dist_state=-1;

double current_x=0;
double current_y=0;
double current_z=0;

double desired_x=0;
double desired_y=0;
double desired_z=150;

//zmienne do odczytywania i identyfikacji obrazu
bool first_run=true;
int height,width;			//zmienne do przechowywania wymiarów obrazu
Mat camera_matrix;
Mat dist_coeffs;
Mat imgOriginal_8bit; //obiekt, w którym bedzie przechowana skonwertowana klatka - wymaga tego jedna z funkcji
Mat rotation_vector(3,1,DataType<double>::type); // Rotation in axis-angle form
Mat rotation_array(3,3,DataType<double>::type);
Mat translation_vector(3,1,DataType<double>::type);

Point2d center;

vector<Point2f> corners; //wektor elementow point2f - przechowa wspolrzedne znalezionych krancow pol szachownicy
vector<Point3f> corners_3d;

//dane dot. planszy
Size patternsize(pattern_colls,pattern_rows); //funkcja findchessboardcorners musi znac wielkosc szachownicy - podaje ja tutaj

image_transport::Subscriber image_sub_; //do subskrypcji obrazu
image_transport::Publisher image_pub_;	//do publikacji przerobionego obrazu
ros::Publisher control_pub;


int main(int argc, char** argv)
{
	std::cout<<"Image_receiver_chessboard started!"<<std::endl;
	ros::init(argc, argv, "image_receiver_chessboard");
	ros::NodeHandle nh;
	ros::Subscriber image_sub_;
	ros::Publisher image_pub_;

	//image_sub_ = nh.subscribe("/ardrone/front/image_raw",	1, imageCb);  //fizyczny AR.DRONE 2.0
	image_sub_ = nh.subscribe("/camera/image",	1,	imageCb); //Wewnęrzna kamera web, imitowana przez image_publisher

	image_pub_ = nh.advertise<sensor_msgs::Image>("/image_converter/output_video", 1);

	ros::spin();

	std::cout<<std::endl<<"Image_receiver_chessboard closed!"<<std::endl;
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
			//ustalenie parametrów obrazu
			height = cv_ptr->image.rows;
			width = cv_ptr->image.cols;
			center = Point2d(cv_ptr->image.cols/2,cv_ptr->image.rows/2);
			int GCD_param = gcd(width,height);
			int ratio_hor= width/GCD_param;
			int ratio_vert=height/GCD_param;
			cout << "gcd("<<width<<","<<height<<")="<<GCD_param<<" ratio=" << ratio_hor<<":"<<ratio_vert<<endl;
			//camera_matrix = (Mat_<double>(3,3) << 692.0438330806977, 0, 311.165894880295, 0, 691.6883188833331, 170.3563054332899, 0, 0, 1);
			camera_matrix = (Mat_<double>(3,3) << width, 0, center.x, 0 , width, center.y, 0, 0, 1);
			//dist_coeffs=(Mat_ <double>(5,1)<<-0.9770726329851025, 3.097064250216448, -0.0170009823405978, 0.01192658072486791, -6.338134825259841);
			dist_coeffs =Mat::zeros(4,1,DataType<double>::type); // Assuming no lens distortion
			cout << "Camera Matrix " << endl << camera_matrix << endl;

			//ustalenie miejsca wyświetlania komunikatów
			hor_text_base.x=0;
			hor_text_base.y=height-10;
			vert_text_base.x=0;
			vert_text_base.y=height-40;
			dist_text_base.x=0;
			dist_text_base.y=height-70;
			//ustalenie parametrów szachownicy, wypełnienie wektorów punktami
			chessboardParam();
		}

		cv_ptr->image.convertTo(imgOriginal_8bit, CV_8U); //konwersja do 8-bitowej glebi - takie obrazy przyjmuje funkcja znajdujaca findchessboardcorners()
		//poszukiwanie szachownicy
		bool patternfound = findChessboardCorners(imgOriginal_8bit, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

		if (!patternfound)
		{
			found=false;
			hor_state=vert_state=dist_state=-1;
			hor_msg="";
			vert_msg="Chessboard not found";
			dist_msg="";
			//cout<<"Nie znalazlem szachownicy!"<<endl;
		}
		if (patternfound)
		{
			found=true;
			solvePnP(corners_3d, corners, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
			Rodrigues(rotation_vector, rotation_array);

			//cout<<"Wektor rotacji"<<rotation_vector<<endl;
			//cout<<"Wektor translacji "<<translation_vector<<endl;
			current_x=translation_vector.at<double>(0,0);
			current_y=translation_vector.at<double>(1,0);
			current_z=translation_vector.at<double>(2,0);
			//korekta
			//current_z=(1-0.24)*current_z;
			current_z=(1-0.221)*current_z-5.6;

			double phi, theta, psi;
			phi=rad2deg(translation_vector.at<double>(0,0));
			theta = rad2deg(translation_vector.at<double>(1,0));
			psi=rad2deg(translation_vector.at<double>(2,0));
			//cout << "phi " << phi <<" theta " << theta << " psi " <<  psi<<endl;
			//cout<<"x: "<< current_x << " y: "<< current_y << " z: "<< current_z << endl;
			findControl(cv_ptr);

		}
		drawChessboardCorners(imgOriginal_8bit, patternsize, corners, patternfound); //rysowanie linii
		//rysowanie komunikatów i siatki
		writeMsg(imgOriginal_8bit);
		drawGrid(imgOriginal_8bit);

		imshow ("Original", imgOriginal_8bit);//wyrzucenie do okna
		image_pub_.publish(cv_ptr->toImageMsg());

		waitKey(3);

	}

}

double deg2rad(double angle_in_degrees)
{
	angle_in_degrees=fmod((angle_in_degrees+360),360);
	return angle_in_degrees * PI / 180.0;
}

double rad2deg(double angle_in_radians)
{
	angle_in_radians=fmod((angle_in_radians+2*PI),(2*PI));
	return angle_in_radians * 180.0 / PI;
}


void drawGrid(Mat &imgThresholded)
{
	Point left(0, height/2);
	Point right(width, height/2);
	Point top(width/2, 0);
	Point bottom(width/2, height);
	//line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)

	line(imgThresholded, left, right, Scalar(160, 160, 160), 0.5);
	line(imgThresholded, top, bottom, Scalar(160, 160, 160), 0.5);
}
void writeMsg(Mat &imgThresholded)
{
	if(hor_state==-1)
	{
		putText(imgThresholded, hor_msg, hor_text_base, fontFace, 1, CV_RGB(204,0,0), 1, 8);
	}
	else if(hor_state==0)
	{
		putText(imgThresholded, hor_msg, hor_text_base, fontFace, 1, CV_RGB(0,204,0), 1, 8);
	}
	else if(hor_state==1)
	{
		putText(imgThresholded, hor_msg, hor_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
	}

	if(vert_state==-1)
	{
		putText(imgThresholded, vert_msg, vert_text_base, fontFace, 1, CV_RGB(204,0,0), 1, 8);
	}
	else if(vert_state==0)
	{
		putText(imgThresholded, vert_msg, vert_text_base, fontFace, 1, CV_RGB(0,204,0), 1, 8);
	}
	else if(vert_state==1)
	{
		putText(imgThresholded, vert_msg, vert_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
	}

	if(dist_state==-1)
	{
		putText(imgThresholded, dist_msg, dist_text_base, fontFace, 1, CV_RGB(204,0,0), 1, 8);
	}
	else if(dist_state==0)
	{
		putText(imgThresholded, dist_msg, dist_text_base, fontFace, 1, CV_RGB(0,204,0), 1, 8);
	}
	else if(dist_state==1)
	{
		putText(imgThresholded, dist_msg, dist_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
	}
}
void chessboardParam()
{
	//l_punktow=pattern_rows*pattern_colls;

	for (int p=0; p<l_punktow; p++)
	{

		corners_3d.push_back(Point3d(0,0,0));

	}
	corners_3d[0].x=-bok_kwad*((pattern_colls-1.0)/2.0);
	corners_3d[0].y=-bok_kwad*((pattern_rows-1.0)/2.0);
	corners_3d[0].z=0;

	for (int i=1; i<l_punktow; i++)
	{

		corners_3d[i].x=corners_3d[i-1].x+bok_kwad;

		if ((i+1)%pattern_colls==0) // (jak dojedzie do konca wiersza to zeruj x, zwiększ y o wysokosc kwadratu - wypelniamy wyższy wiersz od lewej
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
void findControl(cv_bridge::CvImagePtr &cv_ptr)
{

	if (current_x< (desired_x-5))
	{
		hor_state=1;
		hor_msg="go right! x=";
		stringstream stream;
		stream << hor_msg << fixed << setprecision(2) << current_x;
		hor_msg = stream.str();
	}
	else if (current_x> (desired_x+5))
	{
		hor_state=1;
		hor_msg="go left! x=";
		stringstream stream;
		stream << hor_msg << fixed << setprecision(2) << current_x;
		hor_msg = stream.str();
	}
	else
	{
		hor_state=0;
		hor_msg="Hor - OK! x=";
		stringstream stream;
		stream << hor_msg << fixed << setprecision(2) << current_x;
		hor_msg = stream.str();
	}

	if (current_y< (desired_y-5))
	{
		vert_state=1;
		vert_msg="go up! y=";
		stringstream stream;
		stream << vert_msg << fixed << setprecision(2) << current_y;
		vert_msg = stream.str();

	}
	else if (current_y> (desired_y+5))
	{
		vert_state=1;
		vert_msg="go down! y=";
		stringstream stream;
		stream << vert_msg << fixed << setprecision(2) << current_y;
		vert_msg = stream.str();
	}
	else
	{
		vert_state=0;
		vert_msg="Vert - OK! y=";
		stringstream stream;
		stream << vert_msg << fixed << setprecision(2) << current_y;
		vert_msg = stream.str();
	}

	if (current_z< (desired_z-5))
	{
		dist_state=1;
		dist_msg="too close! z=";
		stringstream stream;
		stream << dist_msg << fixed << setprecision(2) << current_z;
		dist_msg = stream.str();

	}
	else if (current_z> (desired_z+5))
	{
		dist_state=1;
		dist_msg="too far! z=";
		stringstream stream;
		stream << dist_msg << fixed << setprecision(2) << current_z;
		dist_msg = stream.str();
	}
	else
	{
		dist_state=0;
		dist_msg="Dist - OK! z=";
		stringstream stream;
		stream << dist_msg << fixed << setprecision(2) << current_z;
		dist_msg = stream.str();
	}
}
static int gcd (int a, int b) {
	return (b == 0) ? a : gcd (b, a%b);
}
