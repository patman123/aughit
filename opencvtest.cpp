#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"

#include <iostream>
#include <stdio.h>

#include <Box2D/Box2D.h>
using namespace std;
using namespace cv;

int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
#define WORLDH 16 
#define WORLDW 12
#define THICKNESS 0.3
#define DATA(A,i,j,k)  A.data[A.step*i + A.channels()*j + k]

// Define the gravity vector.
b2Vec2 gravity(2.0,2.0);

// Do we want to let bodies sleep?
bool doSleep = true;

// Construct a world object, which will hold and simulate the rigid bodies.
b2World world(gravity, doSleep);
float pixel=40;
float shiftx=0;
float shifty=0;

class ball
{
public:
	ball();
	~ball() {};
	b2Body* body;
};

ball::ball()
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(WORLDW/2,WORLDH/2);
	bodyDef.angularVelocity=0.2;
	body = world.CreateBody(&bodyDef);
	b2CircleShape dynamicBox;
	dynamicBox.m_radius = 0.4;

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.density = .002f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 1;
	body->CreateFixture(&fixtureDef);
	
};
class puck
{
public:
	puck();
	~puck() {};
	b2Body* body;
};

puck::puck()
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(WORLDH/2,WORLDW/2);
	body = world.CreateBody(&bodyDef);
	b2CircleShape dynamicBox;
	dynamicBox.m_radius = 0.8;
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 0.1;
	body->CreateFixture(&fixtureDef);
	
};
puck Player;
float Newx=0, Newy=0;
b2Vec2 speed;
b2Vec2 posold;
Mat getbox(cv::Mat img, int code, int c1, int c2, int c3){

	posold=Player.body->GetPosition();
	Mat bin; //To store and return the result
	bin = img.clone();
	int counti=0;
	int countj=0;
	int totcount=1;
	for(int i=0; i < img.rows; i++) {
		for(int j=0; j < img.cols; j++) {
			if(DATA(img,i,j,2) > c3&&DATA(img,i,j,0) <c1&&DATA(img,i,j,1) <c2){
				DATA(bin,i,j,0)  = 255;//Assigned white
				DATA(bin,i,j,1)  = 255;//Assigned white
				DATA(bin,i,j,2)  = 255;//Assigned white
				counti+=i;
				countj+=j;
				totcount++;
			}
			else{
				DATA(bin,i,j,0)  = 0;//Assigned white
				DATA(bin,i,j,1)  = 0;//Assigned white
				DATA(bin,i,j,2)  = 0;//Assigned white
			}//Assigned black
		}
		Newx=counti/totcount;
		Newy=countj/totcount;
		speed.x=(Newx-posold.x)*60;
		speed.y=(Newy-posold.y)*60;
		cout<<Newx<<" "<<Newy<<"\n";
		b2Vec2 pos(Newy/pixel,Newx/pixel);
		Player.body->SetLinearVelocity(speed);
		Player.body->SetTransform(pos, 0);
	}
	return bin; //Result is returned
}
int Threshx=0, Threshy=0;

void mouse_callback(int event, int x, int y, int flags, void* param)
{
	//This is called every time a mouse event occurs in the window
	if (event == CV_EVENT_LBUTTONDOWN) { //This is executed when the left mouse button is clicked
		//Co-ordinates of the left click are assigned to global variables and flag is set to 1
		Threshx = x;
		Threshy = y;
	}

}

int main( int argc, const char** argv )
{
	b2Body* floorBody;
	b2BodyDef floorDef;
	b2FixtureDef floorFixtureDef;
	b2PolygonShape floorBox;

	floorBox.SetAsBox(WORLDW,THICKNESS);
	floorFixtureDef.shape = &floorBox;
	floorFixtureDef.restitution = 0;
	floorFixtureDef.density = 0;
	floorFixtureDef.friction = 0.3f;

	floorDef.position.Set(0,0);
	floorBody = world.CreateBody(&floorDef);
	floorBody->CreateFixture(&floorFixtureDef);

	b2Body* leftBody;
	floorBox.SetAsBox(THICKNESS,WORLDH);
	floorDef.position.Set(0,0);
	leftBody = world.CreateBody(&floorDef);
	leftBody->CreateFixture(&floorFixtureDef);

	b2Body* rightBody;
	floorDef.position.Set(WORLDW,0);
	rightBody = world.CreateBody(&floorDef);
	rightBody->CreateFixture(&floorFixtureDef);
	
	b2Body* topBody;
	floorBox.SetAsBox(WORLDW,THICKNESS);
	floorDef.position.Set(0,WORLDH);
	topBody = world.CreateBody(&floorDef);
	topBody->CreateFixture(&floorFixtureDef);
	b2Vec2 position;
	ball Ball;
	float32 timeStep = 1.0f / 15.0f;
	int32 velocityIterations = 12;
	int32 positionIterations = 15;

	int key = 0;
	
	namedWindow( "ORIGINAL ", CV_WINDOW_AUTOSIZE);
	namedWindow( "Capture ", CV_WINDOW_AUTOSIZE);
    //namedWindow( "Foreground ", CV_WINDOW_NORMAL );
    Mat frame,foreground,image;
    BackgroundSubtractorMOG2 mog(5000, 16, false);
	frame=cvLoadImage("test.jpg");
	
	VideoCapture capture(1);
	if(!capture.isOpened()){
		return -1;
	}

	Size size(WORLDW*pixel, WORLDH*pixel);
	int c1=0, c2=0, c3=0;
	cvCreateTrackbar("Threshold1","Capture ",&c1,255); 
	cvCreateTrackbar("Threshold2","Capture ",&c2,255); 
	cvCreateTrackbar("Threshold3","Capture ",&c3,255); 
	while( key != 'q' ){
	 	capture >> frame;

		flip(frame, frame,1);
		cv::transpose(frame, frame);
		resize(frame, frame,size);
		imshow("ORIGINAL ", frame);
		key = cv::waitKey(33);

		GaussianBlur( frame, frame, Size(3,3),  0, 0 );

		image=frame.clone();
		image=getbox(image, 1, c1, c2, c3);

		Mat threshold_output = image.clone();
		Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
		double max = 0;
		int num = -1;
		Scalar color = CV_RGB( 255, 125, 0 );

		world.Step(timeStep, velocityIterations, positionIterations);

		position = Ball.body->GetPosition();
		circle( image , Point(position.x * pixel, position.y * pixel), 0.4*pixel , CV_RGB( 0, 255, 0 ) , -7,8,0);
		//cout<<position.x<<" "<<position.y<<" ";
		position = Player.body->GetPosition();
		//cout<<position.x<<" "<<position.y<<"\n";
		circle( image , Point(position.x * pixel, position.y * pixel), 0.8*pixel , CV_RGB( 255,0, 0 ) , -7,8,0);
		
		//printf("%f , %f\n", position.x*pixel,position.y*pixel);

		//position=Player.body->GetPosition();
		rectangle( image , Point(0 * pixel, 0 * pixel),Point(WORLDW*pixel, THICKNESS*pixel),color,-2);
		rectangle( image , Point(0 * pixel, 0 * pixel),Point(THICKNESS*pixel,WORLDH*pixel),color, -2);
		rectangle( image , Point(0 * pixel, WORLDH * pixel),Point(pixel*WORLDW, (WORLDH-THICKNESS)*pixel),color, -2);
		rectangle( image , Point(WORLDW * pixel, 0 * pixel),Point((WORLDW-THICKNESS)*pixel,WORLDH*pixel),color, -2);
		imshow( "Capture ", image );
	}

	capture.release();
}