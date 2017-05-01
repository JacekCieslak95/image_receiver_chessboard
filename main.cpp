#include <iostream>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

 int main( int argc, char** argv )
 {
	 //sprawdzenie dzialania kamery

	 VideoCapture cap(0); //przechwyt wideo z kamery

	  if ( !cap.isOpened() )  //zamknięcie programu, gdy nieudane otwarcie kamery

	    {
	         cout << "Nie moge otworzyc kamery!" << endl;
	         return -1;
	    }

	 //estymacja parametrow kamery

	 Mat frame; //tu przechowam pojedyncza klatke w celu zbadanie parametrow

	 cap.read(frame);//czytaj pojedyncza klatke

	 double focal_length =frame.cols; // Approximate focal length
	 Point2d center = Point2d(frame.cols/2,frame.rows/2);
	 Mat camera_matrix = (Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
	 Mat dist_coeffs =Mat::zeros(4,1,DataType<double>::type); // Assuming no lens distortion

	 cout << "Camera Matrix " << endl << camera_matrix << endl;

	 //tworze zmienne potrzebne w petli while

	 Mat imgOriginal; //obiekt, w ktorym beda przechowane kolejne klatki z kamery
	 Mat imgOriginal_8bit; //obiekt, w którym bedzie przechowana skonwertowana klatka - wymaga tego jedna z funkcji

	 vector<Point2f> corners; //wektor elementow point2f - przechowa wspolrzedne znalezionych krancow pol szachownicy
	 vector<Point3f> corners_3d; //ten obiekt przyjmuje solvePnP
	 Size patternsize(9,6); //funkcja findchessboardcorners musi znac wielkosc szachownicy - podaje ja tutaj

	 // Output rotation and translation
	 Mat rotation_vector; // Rotation in axis-angle form
	 Mat translation_vector;

    while (true)
    {

        bool bSuccess = cap.read(imgOriginal); // ile odczyt sie udal - przypisanie klatk do imgOriginal


         if (!bSuccess) //jeśli nie - przerwij
        {
             cout << "Nie moge odczytać klatki z kamery" << endl;
             break;
        }


         imgOriginal.convertTo(imgOriginal_8bit, CV_8U); //konwersja do 8-bitowej glebi - takie obrazy przyjmuje funkcja znajdujaca
         //cvtColor(imgOriginal_8bit, imgOriginal_8bit, CV_BGR2GRAY); - konwersja na szarosc tez nic nie daje

         bool patternfound = findChessboardCorners(imgOriginal_8bit, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

         //corners wypelnione znalezionymi wspolrzednymi

         //jak to zostawie, to program wylacza sie, zwracajac -1
        // Kraft pisal - "Za object points i image points podajcie Panowie punkty na obserwowanej szachownicy 
	//(znalezione za pomocą findchessboardconers)."
        //ta funkcja wypluwa tylko wektory punktów o dwoch wspolrzednych <point2f> (czy tam 2d)
	//wiec tutaj dokladam sobie zero (?) - solvePnP wymaga punktow o trzech wspolrzednych - a jednak nie dziala

         /*for (int i=0; i<corners.size();i++)
         {

        	 corners_3d[i].x=corners[i].x;
        	 corners_3d[i].y=corners[i].y;
        	 corners_3d[i].z=0;

         }*/
	    
	 //sprawdzam czy szachownica znaleziona   

         if (!patternfound)
         {
        	 cout<<"Nie znalazlem szachownicy!"<<endl;
         }

         if (patternfound)

         {

	 //wlasciwe znalezienie translacji i rotacji 
		 
         solvePnP(corners_3d, corners, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

        //wyrzucenie do konsoli

        cout<<rotation_vector<<endl;

        cout<<translation_vector<<endl;

         }

         drawChessboardCorners(imgOriginal_8bit, patternsize, corners, patternfound);

         imshow ("Original", imgOriginal_8bit);

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            {
                 cout << "Nacisnieto esc - wyjscie" << endl;
                  break;
           }

       }

    return 0;

   }
