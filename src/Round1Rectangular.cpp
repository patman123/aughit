#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Box2D/Box2D.h>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>
#include <cmath>
#include "Constants.h"
#include "ClassesR.h"

#define FACTOR 0.0176
#define PI 3.14159265

using namespace std;
using namespace cv;

RNG rng(12345);
#define DATA(A,i,j,k)  A.data[A.step*i + A.channels()*j + k]

#include "Functions.h"
 
int main( int argc, const char** argv )
{
	int CameraIndex = 0;
	if(argc > 1)
		CameraIndex = argv[1][0]-48;
	b2Body* floorBody;
	b2BodyDef floorDef;
	b2FixtureDef floorFixtureDef;
	b2PolygonShape floorBox;
	b2Fixture *_floorFixture;
	MyContactListener *_contactListener;
	_contactListener = new MyContactListener();
	world.SetContactListener(_contactListener);
	int ballX , ballY;
	cout << "Enter Start Position of Ball(x,y):";
	cin >> ballX >> ballY;
	b2Vec2 position;
	ball Ball(ballX, ballY);
	int paddleX;
	cout << "\nEnter Start position of Paddle (x): ";
	cin >> paddleX;
	paddle Player(paddleX);
	b2Vec2 force = b2Vec2(0.3,2);
	float32 timeStep = 3.0f / 30.0f;
	int32 velocityIterations = 20;
	int32 positionIterations = 5;
	block Blocks[BLOCKCOUNT];
	corner Corners[CORNERCOUNT];
	bool flag = true , flag_x = true;
	char VidName = 'a';
	char fname[5];
	int destroyed[BLOCKCOUNT];
	for(int dest=0;dest<BLOCKCOUNT;dest++)destroyed[dest]=0;
	char key = 0;
	int bA=0, bB=0;
	b2Vec2 locationWorld;
	b2MouseJoint *_mouseJoint;
	Size size(WORLDW*pixel, WORLDH*pixel);
	int c1=50, c2=26;
	int thresh1=35, thresh2=42,thresh3=33;
	double angle_val = 0 ;
	RotatedRect rRect;
	Point2f vertices[4];
	vector<int> pospad;
	b2Fixture *posfix;
	b2Shape *poshape;
	b2PolygonShape* pospoly;
	b2Vec2 pos[4];
	b2Vec2 *posp;
	Point rook_points[1][4];
	int npt[]={4};
	b2Vec2 newcoord;
	double *angle;
	namedWindow("Capture",CV_WINDOW_AUTOSIZE);
	angle=(double *)malloc(sizeof(double));
	int bottomhitcount=0;
	double drawxoff=0, drawyoff=0;
	double drawxcorner=0 , drawycorner=0;
    Mat frame,foreground,image, gameover, lineimg;
    Mat threshold_output=Mat::zeros( size, CV_8UC1 );
	b2Body* bodyA;
	b2Body* bodyB;
	int frm=0;
	KalmanFilter KF(4, 2, 0);
	std::vector<b2Body *>toDestroy;
	// intialization of KF...
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
	 
	KF.statePre.at<float>(0) = WORLDW/2*pixel;
	KF.statePre.at<float>(1) = (WORLDH-2)*pixel;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(10));
	setIdentity(KF.errorCovPost, Scalar::all(.1));


	char* output="";

	floorBox.SetAsBox(WORLDW/2,THICKNESS/2);
	floorFixtureDef.shape = &floorBox;
	floorFixtureDef.restitution = 1;
	floorFixtureDef.density = 10000;
	floorFixtureDef.friction = 0;
	floorDef.position.Set(WORLDW/2, WORLDH-THICKNESS/2);
	floorDef.type=b2_staticBody;
	floorBody = world.CreateBody(&floorDef);
	_floorFixture=floorBody->CreateFixture(&floorFixtureDef);

	b2Body* leftBody;
	floorBox.SetAsBox(THICKNESS/2, WORLDH/2-THICKNESS);
	floorDef.position.Set(THICKNESS/2,WORLDH/2);
	leftBody = world.CreateBody(&floorDef);
	leftBody->CreateFixture(&floorFixtureDef);

	b2Body* rightBody;
	floorDef.position.Set(WORLDW-THICKNESS/2,WORLDH/2);
	rightBody = world.CreateBody(&floorDef);
	rightBody->CreateFixture(&floorFixtureDef);
	
	b2Body* topBody;
	floorBox.SetAsBox(WORLDW/2,THICKNESS/2);
	floorDef.position.Set(WORLDW/2, THICKNESS/2);
	topBody = world.CreateBody(&floorDef);
	topBody->CreateFixture(&floorFixtureDef);

	Ball.body->ApplyLinearImpulse(force, Ball.body->GetPosition());
	force = b2Vec2(-40,-30);
	setMouseCallback("Capture", mouse_callback, NULL);
	frame=cvLoadImage("test.jpg");
	gameover=cvLoadImageM("gameover.jpg");

    vector<blob> blobs;
	VideoWriter outputVideo("./Video/a.avi", CV_FOURCC('M','J','P','G'), 10, size, true);
	printf("%d %d\n", (int)600, (int)800);
	while( key != 'q' )
	{
		if(flag_x == false)
		{
		 sprintf(fname, "./Video/%c.avi" , VidName);
			outputVideo.open(fname, CV_FOURCC('M','J','P','G'), 10, size, true);
   	 		flag_x = true;
   	 	}	
    	if (!outputVideo.isOpened())
   		{
        	cout  << "Could not open the output video for write: " << endl;
        	return -1;
   		}
	 	
		key=waitKey(33);
		image = Mat::zeros( size, CV_8UC3 );
		double max = 0;
		int num = -1;
		Scalar color = CV_RGB( 255, 220, 0);
		rectangle( image , Point(0 * pixel, 0 * pixel),Point(WORLDW*pixel, THICKNESS*pixel),color,-2);
		rectangle( image , Point(0 * pixel, 0 * pixel),Point(THICKNESS*pixel,WORLDH*pixel),color, -2);
		rectangle( image , Point(0 * pixel, WORLDH * pixel),Point(pixel*WORLDW, (WORLDH-THICKNESS)*pixel),color, -2);
		rectangle( image , Point(WORLDW * pixel, 0 * pixel),Point((WORLDW-THICKNESS)*pixel,WORLDH*pixel),color, -2);
		
		world.Step(timeStep, velocityIterations, positionIterations);

		position = Ball.body->GetPosition();
		circle( image , Point(position.x * pixel, position.y * pixel), 0.4*pixel , CV_RGB( 0, 255, 0 ) , -7,8,0);
		locationWorld = b2Vec2(Threshx/pixel, Threshy/pixel);
		position = Player.body->GetPosition();
		posfix=Player.body->GetFixtureList();
		poshape = posfix->GetShape();
		// Rectangular
   		if(poshape->GetType() == b2Shape::e_polygon){
     		pospoly = (b2PolygonShape*)poshape;
  			 }
  		pos[0]=pospoly->GetVertex(0);
  		pos[1]=pospoly->GetVertex(1);
  		pos[2]=pospoly->GetVertex(2);
  		pos[3]=pospoly->GetVertex(3);
	 	 newcoord.x=Threshx;
		*angle=angle_val;
	 	 newcoord.y=WORLDH-2;
		measurement(0)=newcoord.x;
		measurement(1)=*angle;
		Mat prediction = KF.predict();
		Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
		Mat estimated = KF.correct(measurement);
 
		Point statePt(estimated.at<float>(0),estimated.at<float>(1));
		Point measPt(measurement(0),measurement(1));
		newcoord.x=measPt.x;

		newcoord.x=newcoord.x/pixel;
		// Rectangular
		posp=getPoints(pos, *angle);

  		for(int c=0;c<4;c++)
  		{
  			rook_points[0][c]=Point((position.x-posp[c].x)*pixel, (position.y-posp[c].y)*pixel);
  		}
		const Point* ppt[1]= {rook_points[0]};
		newcoord.x=Threshx/pixel;
	 	if(newcoord.x<(WORLDW-1.7-THICKNESS)&&newcoord.x>(1.7+THICKNESS))
	 	{
	 		Player.body->SetTransform(newcoord, (float)*angle);
		}
	 	// Rectangular
	 	fillPoly( image,ppt,npt, 1, CV_RGB(255,0,0) );
		drawxoff=0; 
		drawyoff=0;
		drawxcorner=0; 
		drawycorner=0;
		for(int bc=0;bc<BLOCKCOUNT;bc++)
		{
			if(destroyed[bc]==0)

			{
				rectangle( image , Point((drawxoff+WORLDW/12)*pixel, (drawyoff+WORLDH/15)*pixel),Point((drawxoff+WORLDW/6)*pixel, (drawyoff+2*WORLDH/15)*pixel),CV_RGB(0,0,255), -2);	
			}
			else if(destroyed[bc]==2)
			{
				rectangle( image , Point((drawxoff+WORLDW/12)*pixel, (drawyoff+WORLDH/15)*pixel),Point((drawxoff+WORLDW/6)*pixel, (drawyoff+2*WORLDH/15)*pixel),CV_RGB(255,255,0), -2);
			}
			drawxoff+=WORLDW/8;
			if(bc%7==6){
				drawxoff=0;
				drawyoff+=2*WORLDH/15;
			}
		}
		Point a[3],b[3];
		a[0]=Point(THICKNESS*pixel, (THICKNESS)*pixel);
		a[1]=Point( cornersize*THICKNESS*pixel , (THICKNESS)*pixel);
		a[2]=Point(THICKNESS*pixel , cornersize*THICKNESS*pixel);
		b[0]=Point((WORLDW-THICKNESS)*pixel , (THICKNESS)*pixel);
		b[1]=Point((WORLDW-cornersize*THICKNESS)*pixel, (THICKNESS)*pixel);
		b[2]=Point((WORLDW-THICKNESS)*pixel ,(cornersize*THICKNESS)*pixel);
		fillConvexPoly(image , a, 3, CV_RGB(255,220,0));
		fillConvexPoly(image , b, 3, CV_RGB(255,220,0));
		outputVideo << image;
		position=Ball.body->GetPosition();
		if(position.y>=(WORLDH-0.4-THICKNESS-0.005))		//The ball has hit the bottom floor
			{
				bottomhitcount++;
			}
		if(position.y < WORLDH-6)
			flag = true;
		if(position.y >= WORLDH-6 && flag==true)		//Have to fix this now
		{
		 	VidName++;
		 	flag_x = false;
		 	outputVideo.release();
			while(1){
					double sum_x = 0 , sum_y = 0 , cen_x = 0 , cen_y = 0;
		 			int n_pixels = 0;
				    VideoCapture capture(CameraIndex);
					if(!capture.isOpened()){
					return -1;
					}
				capture >> frame;
				//imshow("camearcapture",frame);
				resize(frame, frame, size);
    			namedWindow( "Canny", CV_WINDOW_AUTOSIZE );
				cvCreateTrackbar("Threshold 1","Canny",&c1,255); 
				cvCreateTrackbar("Threshold 2","Canny",&c2,255); 
				cvCreateTrackbar("Threshold R","Canny",&thresh1,255); 
				cvCreateTrackbar("Threshold G","Canny",&thresh2,255); 
				cvCreateTrackbar("Threshold B","Canny",&thresh3,255); 
			 	frame=getbox(frame, thresh1, thresh2, thresh3, angle, &newcoord.x); 
				// Use Canny instead of threshold to catch squares with gradient shading
				cv::Mat bw;
				cv::Canny(frame, bw, c1, c2, 3);
				// Find contours
				std::vector<std::vector<cv::Point> > contours;
				cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

				std::vector<cv::Point> approx;
				cv::Mat dst = frame.clone();


				for (int i = 0; i < contours.size(); i++)
				{
					// Approximate contour with accuracy proportional
					// to the contour perimeter
					cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

					// Skip small or non-convex objects 
					if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
						continue;
					cout << approx.size() << " is approx size " << endl;
						// Number of vertices of polygonal curve
					int vtc = approx.size();

					// Get the cosines of all corners
					std::vector<double> cos;
					for (int j = 2; j < vtc+1; j++)
						cos.push_back(findangle(approx[j%vtc], approx[j-2], approx[j-1]));

					// Sort ascending the cosine values
					std::sort(cos.begin(), cos.end());

					// Get the lowest and the highest cosine
					double mincos = cos.front();
					double maxcos = cos.back();
					double paddle_angle = 0;
					// Use the degrees obtained above and the number of vertices
					// to determine the shape of the contour
					if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
						{
							cout << "Rectangular Paddle\n";
							Threshx = ( approx[0].x + approx[1].x + approx[2].x + approx[3].x ) * 0.25;
							if((approx[0].x-approx[1].x)*(approx[0].x-approx[1].x)+(approx[0].y-approx[1].y)*(approx[0].y-approx[1].y) > (approx[1].x-approx[2].x)*(approx[1].x-approx[2].x)+(approx[1].y-approx[2].y)*(approx[1].y-approx[2].y))
								paddle_angle = atan((double)(approx[0].y-approx[1].y)/(double)(approx[0].x-approx[1].x));
							else
								paddle_angle = atan((double)(approx[1].y-approx[2].y)/(double)(approx[1].x-approx[2].x));
							paddle_angle = paddle_angle*180/PI;
							if(paddle_angle<0)
								paddle_angle = -paddle_angle;
							else 
								paddle_angle = 180 - paddle_angle;
							cout << "Angle Value: " << paddle_angle << endl;
							angle_val = (180-paddle_angle) * FACTOR;
						}	
				}
				cv::imshow("dst", dst);
				imshow("Canny",bw);
				capture.release();
				char a = waitKey(33);
				if(a==27) 
				{
					flag = 0;
					break;
				}
			}
		}
		 std::vector<b2Body *>::iterator pos2;
			    for (pos2 = toDestroy.begin(); pos2 != toDestroy.end(); ++pos2) {
			        b2Body *body = *pos2;
			        world.DestroyBody(body);
			        
	    }
	    toDestroy.clear();
		std::vector<MyContact>::iterator pos;
		for(pos = _contactListener->_contacts.begin(); 
		  pos != _contactListener->_contacts.end(); ++pos) {
		    MyContact contact = *pos;
	        bodyA = contact.fixtureA->GetBody();
	        bodyB = contact.fixtureB->GetBody();
			if (bodyA->GetUserData() != NULL && bodyB->GetUserData() != NULL) {
				bA=(int)bodyA->GetUserData();
				bB=(int)bodyB->GetUserData();
			            //Sprite A = ball, Sprite B = Block
			            if (bA == 1 && bB > 2) {
			                if (std::find(toDestroy.begin(), toDestroy.end(), bodyB) == toDestroy.end()) {
			                    toDestroy.push_back(bodyB);
			                    destroyed[bB-3]=1;
			                }
			            }
			            //Sprite A = block, Sprite B = ball
			            else if (bA > 2 && bB == 1) {
			                if (std::find(toDestroy.begin(), toDestroy.end(), bodyA) == toDestroy.end()) {
			                    toDestroy.push_back(bodyA);
			                    destroyed[bA-3]=1;
			                }
			            }
			        }
			    
				 }	   
		//if(bottomhitcount>2)break;
		imshow( "Capture", image );
		Vec3b intensity;
	}
	resize(gameover, gameover ,size);
	imshow("Capture", gameover);
	waitKey(2000);

}
