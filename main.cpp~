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

	 double focal_length =frame.cols; // aproksymacja ogniskowej
	 Point2d center = Point2d(frame.cols/2,frame.rows/2);
	 Mat camera_matrix = (Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
	 Mat dist_coeffs =Mat::zeros(4,1,DataType<double>::type); // Assuming no lens distortion

	 cout << "Camera Matrix " << endl << camera_matrix << endl;

	 //tworze zmienne potrzebne w petli while

	 Mat imgOriginal; //obiekt, w ktorym beda przechowane kolejne klatki z kamery
	 Mat imgOriginal_8bit; //obiekt, w którym bedzie przechowana skonwertowana klatka - wymaga tego jedna z funkcji

	 vector<Point2f> corners; //wektor elementow point2f - przechowa wspolrzedne znalezionych krancow pol szachownicy

	 Size patternsize(9,6); //funkcja findchessboardcorners musi znac wielkosc szachownicy - podaje ja tutaj

	 // Output rotation and translation
	 Mat rotation_vector; // Rotation in axis-angle form
	 Mat translation_vector;

	 int l_punktow=54;

	 vector<Point3f> corners_3d;

	 //zeby wpisac do wektora, musze najpierw stworzyc 54 puste elementy - tu chyba lezal blad

	 for (int p=0; p<54; p++)
	 {

	 corners_3d.push_back(Point3d(0,0,0));

	 }

	 float bok_kwad=2.7; //w centymetrach

	 //skrajny lewy dolny punkt niech ma wspolrzedne (0,0,0)

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

    while (true)
    {

        bool bSuccess = cap.read(imgOriginal); // ile odczyt sie udal - przypisanie klatk do imgOriginal


         if (!bSuccess) //jeśli nie - przerwij
        {
             cout << "Nie moge odczytać klatki z kamery" << endl;
             break;
        }


         imgOriginal.convertTo(imgOriginal_8bit, CV_8U); //konwersja do 8-bitowej glebi - takie obrazy przyjmuje funkcja znajdujaca findchessboardcorners()

         bool patternfound = findChessboardCorners(imgOriginal_8bit, patternsize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

         //corners wypelnione znalezionymi wspolrzednymi

         if (!patternfound)
         {
        	 cout<<"Nie znalazlem szachownicy!"<<endl;
         }

         if (patternfound)

         {

         solvePnP(corners_3d, corners, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

        //wyrzucenie do konsoli

        cout<<"Wektor rotacji"<<rotation_vector<<endl;

        cout<<"Wektor translacji"<<translation_vector<<endl;

         }

         drawChessboardCorners(imgOriginal_8bit, patternsize, corners, patternfound); //rysowanie linii


         imshow ("Original", imgOriginal_8bit);//wyrzucenie do okna



        if (waitKey(30) == 27) //konczymy program eskejpem
            {
                 cout << "Nacisnieto esc - wyjscie" << endl;
                  break;
           }

       }

    return 0;

   }
