#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

#define DATA(A,i,j,k)  A.data[A.step*i + A.channels()*j + k]

RNG rng(12345);

int main()

{
	
    Mat frame,foreground,image, gameover, lineimg;
	
	char key;

	Size size(400, 600);
	//namedWindow( "Lines ", CV_WINDOW_AUTOSIZE);
	namedWindow( "Capture ", CV_WINDOW_AUTOSIZE);
    namedWindow( "Camera ", CV_WINDOW_AUTOSIZE );
	frame=cvLoadImage("test.jpg");
	gameover=cvLoadImageM("gameover.jpg");

    VideoCapture capture("VirtualArena.avi");
	if(!capture.isOpened()){
		return -1;
	}

    VideoWriter outputVideo("VirtualArena_withoutpaddle.avi", CV_FOURCC('M','J','P','G'), 10, size, true);

    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " << endl;
        return -1;
    }

	while( key != 'q' ){
	 	capture >> frame;



		key=waitKey(33);
		image = Mat::zeros( frame.size(), CV_8UC3 );

		for(int i=0;i<600;i++){
			for(int j=0;j<400;j++){
				if(i>500&&DATA(frame, i, j, 2)>10&&DATA(frame, i, j, 1)<50&&DATA(frame, i, j, 0)<50){
					DATA(image, i, j,2)=0;
				DATA(image, i, j ,1)=0;
				DATA(image, i, j ,0)=0;
				}
				else{

					DATA(image, i, j ,2)=DATA(frame, i, j , 2);
				DATA(image, i, j ,1)=DATA(frame, i, j, 1);
				DATA(image, i, j ,0)=DATA(frame, i, j, 0);
				}
			}
		}
		imshow( "Capture ", frame);
		imshow("Camera ", image);
		//imshow("Camera ", threshold_output);
		outputVideo<<image;
	}
	waitKey(2000);
	capture.release();
}
